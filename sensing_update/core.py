"""Hardware-free geometry, fusion, tracking and warning decisions.
Frame: +x right, +y forward, +angle right. Configurable sensor extrinsics.
Absolute velocity is a forward-only estimate, not a complete odometry system.
"""
import copy
import math
import statistics
import threading


class Latest:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = None

    def set(self, value):
        with self.lock:
            self.value = value

    def get(self):
        with self.lock:
            return self.value


def pixel_angle(x, width, hfov):
    # Pinhole projection, not linear interpolation of angle over the image.
    fx = width / (2 * math.tan(math.radians(hfov / 2)))
    return math.degrees(math.atan((x - width / 2) / fx))


def calibrated_points(scan, cfg):
    points = []
    for q, raw, distance in scan:
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (q, raw, distance)):
            continue
        if q < cfg['min_quality'] or not cfg['min_range_m'] <= distance <= cfg['max_range_m']:
            continue
        angle = (raw - cfg['front_offset_deg'] + 180) % 360 - 180
        if cfg['reverse']:
            angle = -angle
        rad = math.radians(angle)
        points.append({'angle_deg': angle, 'dist_m': distance,
                       'x_m': distance * math.sin(rad), 'y_m': distance * math.cos(rad)})
    return points


def cluster_points(points, cfg):
    front = sorted((p for p in points if -90 <= p['angle_deg'] <= 90), key=lambda p: p['angle_deg'])
    groups = []
    for p in front:
        if (groups and p['angle_deg'] - groups[-1][-1]['angle_deg'] <= cfg['angle_gap_deg']
                and abs(p['dist_m'] - groups[-1][-1]['dist_m']) <= cfg['radial_gap_m']):
            groups[-1].append(p)
        else:
            groups.append([p])
    output = []
    for g in groups:
        if len(g) < cfg['min_points']:
            continue
        near = min(g, key=lambda p: p['dist_m'])
        output.append({'angle_deg': statistics.median(p['angle_deg'] for p in g),
                       'dist_m': near['dist_m'], 'range_median_m': statistics.median(p['dist_m'] for p in g), 'x_m': statistics.median(p['x_m'] for p in g),
                       'y_m': statistics.median(p['y_m'] for p in g),
                       'angle_min_deg': g[0]['angle_deg'], 'angle_max_deg': g[-1]['angle_deg'],
                       'count': len(g), 'points': g})
    return output


def associate(clusters, vision, scan_time, now, cfg):
    """One-to-one angular association. Returns do not prove semantic identity."""
    matches = {}
    if not vision or not vision.get('valid'):
        return matches
    if now - vision['timestamp'] > cfg['camera']['max_result_age_s']:
        return matches
    if abs(scan_time - vision['timestamp']) > cfg['fusion']['max_time_skew_s']:
        return matches
    candidates = []
    cam = cfg['camera']
    for i, c in enumerate(clusters):
        bearings = [math.degrees(math.atan2(p['x_m'] - cam['x_m'], p['y_m'] - cam['y_m'])) - cam['yaw_deg']
                    for p in c['points'] if p['y_m'] > cam['y_m']]
        for j, d in enumerate(vision.get('detections', [])):
            dt=d.get('timestamp',vision['timestamp'])
            if not 0<=now-dt<=cam['max_result_age_s'] or abs(scan_time-dt)>cfg['fusion']['max_time_skew_s']:
                continue
            if d['conf'] < cam['confidence']:
                continue
            lo, hi = d['angle_min_deg'], d['angle_max_deg']
            margin = cfg['fusion']['margin_deg']
            support = sum(lo - margin <= a <= hi + margin for a in bearings)
            if support >= cfg['clustering']['min_points']:
                mid = (lo + hi) / 2
                error = min(abs(a - mid) for a in bearings)
                candidates.append((error - 0.01 * d['conf'], i, j))
    used_c, used_d = set(), set()
    for _, i, j in sorted(candidates):
        if i not in used_c and j not in used_d:
            d = vision['detections'][j]
            cam = cfg['camera']
            margin = cfg['fusion']['margin_deg']
            supported = [p for p in clusters[i]['points'] if p['y_m'] > cam['y_m'] and
                         d['angle_min_deg']-margin <= math.degrees(math.atan2(
                             p['x_m']-cam['x_m'],p['y_m']-cam['y_m']))-cam['yaw_deg'] <= d['angle_max_deg']+margin]
            matches[i] = dict(d, lidar_support_points=supported)
            used_c.add(i)
            used_d.add(j)
    return matches


class LidarTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = {}
        self.next_id = 1
        self.ego_speed = 0.0
        self.ego_valid = False

    def reset(self):
        self.tracks.clear()
        self.ego_speed, self.ego_valid = 0.0, False

    def update(self, clusters, matches, timestamp, motion_uncertain=False, timing_valid=True):
        tcfg = self.cfg['tracking']
        # Expire BEFORE matching; all velocity dt uses per-track acquisition times.
        previous = {k: v for k, v in self.tracks.items() if timestamp - v['last_seen'] <= tcfg['coast_s']}
        pairs = []
        for i, c in enumerate(clusters):
            for k, old in previous.items():
                da = abs(c['angle_deg'] - old['angle_deg'])
                dd = abs(c['dist_m'] - old['dist_m'])
                if da < tcfg['angle_gate_deg'] and dd < tcfg['distance_gate_m']:
                    pairs.append((da/tcfg['angle_gate_deg'] + dd/tcfg['distance_gate_m'], i, k))
        assigned, used = {}, set()
        for _, i, k in sorted(pairs):
            if i not in assigned and k not in used:
                assigned[i] = k
                used.add(k)
        updated = {}
        ego_candidates = []
        for i, c in enumerate(clusters):
            old = previous.get(assigned.get(i))
            k = old['id'] if old else self.next_id
            if not old:
                self.next_id += 1
            state = copy.copy(c)
            state.update(id=k, last_seen=timestamp, observed=True, velocity_valid=False,
                         timing_valid=timing_valid, geometry_stable=False,
                         v_rel=0.0, vx_rel=0.0, vy_rel=0.0, stationary_since=None,
                         motion_state='UNKNOWN', semantic_class='obstacle', semantic_timestamp=None,
                         semantic_confidence=0.0, cv_confirmed=i in matches)
            if old:
                dt = timestamp - old['last_seen']
                ratio = c['count']/max(1, old['count'])
                stable = (.5 <= ratio <= 2 and
                          abs(c['angle_min_deg']-old['angle_min_deg']) <= 8 and
                          abs(c['angle_max_deg']-old['angle_max_deg']) <= 8)
                state['geometry_stable'] = stable
                if (timing_valid and old.get('timing_valid', True) and stable and
                        .05 <= dt <= tcfg['max_velocity_gap_s']):
                    state['velocity_valid'] = True
                    for key, pos in [('v_rel', 'dist_m'), ('vx_rel', 'x_m'), ('vy_rel', 'y_m')]:
                        raw = ((c.get('range_median_m',c[pos])-old.get('range_median_m',old[pos]))
                               if pos=='dist_m' else c[pos]-old[pos]) / dt
                        state[key] = 0.5*raw + 0.5*old.get(key, raw) if old.get('velocity_valid') else raw
                    state['stationary_since'] = old.get('stationary_since')
                if (old.get('semantic_timestamp') is not None and
                        timestamp - old['semantic_timestamp'] <= self.cfg['fusion']['semantic_memory_s']):
                    for key in ('semantic_class', 'semantic_timestamp', 'semantic_confidence'):
                        state[key] = old[key]
            if i in matches:
                d = matches[i]
                state.update(semantic_class=d['name'], semantic_confidence=d['conf'], semantic_timestamp=timestamp,
                             cv_points=d.get('lidar_support_points', c['points']))
            if state['velocity_valid'] and abs(c['angle_deg']) <= 60:
                # Moving-capable classes cannot serve as forward-speed references.
                excluded = {'person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck', 'dog', 'cat'}
                if state['semantic_class'] not in excluded:
                    candidate = -state['v_rel'] / math.cos(math.radians(c['angle_deg']))
                    if abs(candidate) <= 2.2:
                        ego_candidates.append((candidate, c['angle_deg']))
            updated[k] = state
        self.ego_valid = False
        self.ego_speed = 0.0
        if len(ego_candidates) >= tcfg['ego_min_tracks'] and not motion_uncertain:
            values = [v for v, a in ego_candidates]
            angles = [a for v, a in ego_candidates]
            median = statistics.median(values)
            mad = statistics.median(abs(v-median) for v in values)
            if max(angles)-min(angles) >= tcfg['ego_min_angle_span_deg'] and mad <= tcfg['ego_max_mad_mps']:
                self.ego_valid, self.ego_speed = True, median
        for c in updated.values():
            c['ego_valid'] = self.ego_valid
            c['v_abs'] = None
            if self.ego_valid and c['velocity_valid'] and not motion_uncertain:
                # Forward-only compensation; turns and centroid changes remain limitations.
                speed = math.hypot(c['vx_rel'], c['vy_rel'] + self.ego_speed)
                c['v_abs'] = c['v_rel'] + self.ego_speed*math.cos(math.radians(c['angle_deg']))
                if speed <= tcfg['stationary_speed_mps']:
                    if c['stationary_since'] is None:
                        c['stationary_since'] = timestamp
                    patience = tcfg['person_patience_s'] if c['semantic_class']=='person' else tcfg['object_patience_s']
                    c['motion_state'] = 'STATIC' if timestamp-c['stationary_since'] >= patience else 'UNKNOWN'
                elif speed >= tcfg['moving_speed_mps']:
                    c['stationary_since'] = None
                    c['motion_state'] = 'MOVING'
                else:
                    c['stationary_since'] = None
            else:
                c['stationary_since'] = None
        for k, old in previous.items():
            if k not in used:
                coast = copy.copy(old)
                coast.update(observed=False, cv_confirmed=False, motion_state='UNKNOWN', stationary_since=None)
                updated[k] = coast
        self.tracks = updated
        return list(updated.values())


def warning_threats(tracks, cfg, vision_healthy=True):
    h = cfg['haptics']
    threats = []
    for c in tracks:
        if not c['observed']:
            continue
        points = c['points']
        near = min(points, key=lambda p: p['dist_m'])
        corridor = [p for p in points if p['y_m'] >= 0 and abs(p['x_m']) <= h['corridor_half_width_m']]
        confirmed_corridor = [p for p in c.get('cv_points', points) if p['y_m'] >= 0
                              and abs(p['x_m']) <= h['corridor_half_width_m']]
        close = near['dist_m'] <= h['unknown_distance_m']
        supported = c.get('cv_confirmed', False)
        # No semantic class or motion state delays immediate collision warnings.
        if h['policy']=='lidar' or not vision_healthy:
            usable = corridor
            reason = 'lidar_fallback' if not vision_healthy else 'lidar_corridor'
        elif supported:
            usable = confirmed_corridor
            reason = 'cv_lidar_corridor'
        else:
            usable = []
            reason = 'unknown_close'
        if close:
            usable = points  # Emergency proximity bubble across the whole front sector.
            reason = 'confirmed_close' if supported else 'unknown_close'
        if not usable:
            continue
        nearest = min(usable, key=lambda p:p['dist_m'])
        if nearest['dist_m'] < h['warning_distance_m']:
            threats.append({'id': c['id'], 'angle_deg': nearest['angle_deg'], 'dist_m': nearest['dist_m'],
                            'semantic_class': c['semantic_class'], 'reason': reason})
    return threats


def motor_duties(threats, h):
    levels = [0.0]*5
    overrides = [0.0]*5
    for t in threats:
        a, d = t['angle_deg'], t['dist_m']
        if not math.isfinite(a) or not math.isfinite(d) or not -90 <= a <= 90 or d < 0:
            continue
        if 'duty_override' in t:
            duty=t['duty_override']
            if not math.isfinite(duty) or not 0<=duty<=1:continue
            for i in range(4):
                lo=-90+i*45
                if lo<=a<=lo+45:
                    r=(a-lo)/45; peak=max(r,1-r)
                    overrides[i]=max(overrides[i],duty*(1-r)/peak)
                    overrides[i+1]=max(overrides[i+1],duty*r/peak)
                    break
            continue
        if d >= h['warning_distance_m']:
            continue
        p = 1.0 if d <= h['full_power_distance_m'] else (h['warning_distance_m']-d)/(h['warning_distance_m']-h['full_power_distance_m'])
        for i in range(4):
            lo = -90+i*45
            if lo <= a <= lo+45:
                r = (a-lo)/45
                levels[i] = max(levels[i], p*(1-r))
                levels[i+1] = max(levels[i+1], p*r)
                break
    return [max(override,h['min_duty']+(1-h['min_duty'])*v if v>0.02 else 0.0)
            for v,override in zip(levels,overrides)]
