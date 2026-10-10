"""Finite warnings with many-to-many surface continuity across cluster splits.
Encounter IDs describe tracked geometry, not biometric/physical-object identity.
"""
import math
from core import motor_duties


def surface_overlap(a, b, tolerance=.18):
    """Require actual shared returns, not merely overlapping camera boxes."""
    pa, pb = a.get('points', []), b.get('points', [])
    if not pa or not pb:
        return False
    small, large = (pa, pb) if len(pa) <= len(pb) else (pb, pa)
    # Angular bins bound the search cost and prevent opposite-side close points
    # merging just because both happen to be very near the sensor.
    bins = {}
    for p in large:
        bins.setdefault(int(p['angle_deg']//3), []).append(p)
    count = 0
    required = max(2, math.ceil(len(small)*.3))
    for p in small:
        key = int(p['angle_deg']//3)
        if any(abs(p['angle_deg']-q['angle_deg']) <= 3 and
               math.hypot(p['x_m']-q['x_m'], p['y_m']-q['y_m']) <= tolerance
               for k in (key-1,key,key+1) for q in bins.get(k, [])):
            count += 1
            if count >= required:
                return True
    return False


class EncounterWarnings:
    def __init__(self, cfg):
        self.cfg = cfg
        self.encounters = {}
        self.next_id = 1
        self.last_tick = None
        self.was_healthy = True
        self.events = []
        self.track_encounters = {}
        self.clear_cues = []

    def _cue(self, k, e, now):
        if not self.cfg.get("reminder_enabled",False) or e.get("clear_notified") or not e.get("last_threat"):
            return
        e["clear_notified"] = True
        self.clear_cues.append(dict(e["last_threat"],encounter_id=k,cue_queued=now,cue_started=None,cue_stage=None,cue_pulses=[]))
        self.events.append({"event":"departure_confirmed","encounter_id":k})

    def select(self, tracks, threats, now, healthy=True, generation=0):
        self.events = []
        self.track_encounters = {}
        if self.last_tick is not None and (not healthy or not self.was_healthy):
            gap = max(0, now-self.last_tick)
            for e in self.encounters.values():
                if e.get('missing_since') is not None:
                    e['missing_since'] += gap
        self.last_tick, self.was_healthy = now, healthy
        if not healthy:
            self.clear_cues = []
            for e in self.encounters.values():
                if e['warned']:e['until']=min(e['until'],now)
                e['clear_since']=None
            return [], None, []
        for k,e in self.encounters.items():
            if e.get('missing_since') is not None and now-e['missing_since'] >= self.cfg['rearm_absence_s']:
                if e['warned']:self._cue(k,e,now)
                self.events.append({'event':'rearmed_after_absence','encounter_id':k,
                                    'healthy_absence_s':now-e['missing_since']})
        self.encounters = {k:e for k,e in self.encounters.items()
                           if e.get('missing_since') is None or
                           now-e['missing_since'] < self.cfg['rearm_absence_s']}
        observed = [t for t in tracks if t['observed']]
        parents = {k:k for k in self.encounters}
        def root(k):
            while parents[k] != k:
                parents[k] = parents[parents[k]]
                k = parents[k]
            return k
        assignments = {}
        tolerance = self.cfg.get('surface_match_distance_m', .18)
        # Existing track identity wins over incidental surface proximity.
        # Only unassigned tracks need geometric continuity (splits/new IDs).
        owners = {key:k for k,e in self.encounters.items()
                  for key in e.get('track_keys',set())}
        for i,t in enumerate(observed):
            owner = owners.get((generation,t['id']))
            if owner is not None:
                assignments[i] = owner
        live_owners = set(assignments.values())
        for i,t in enumerate(observed):
            if i in assignments:
                continue
            overlaps = [k for k,e in self.encounters.items()
                        if any(surface_overlap(t,s,tolerance) for s in e['surfaces'])]
            if overlaps:
                live = [k for k in overlaps if k in live_owners]
                if live:
                    # A new bridging cluster must not unite independently
                    # observed encounters. Attach to the nearest live surface.
                    assignments[i] = min(live, key=lambda k: (
                        min(math.hypot(t['x_m']-s['x_m'],t['y_m']-s['y_m'])
                            for s in self.encounters[k]['surfaces']), k))
                else:
                    r = min(root(k) for k in overlaps)
                    for k in overlaps:
                        parents[root(k)] = r
                    assignments[i] = r
        # Spatial continuity handles coherent motion larger than point jitter.
        # A fallback remains one-to-one; only measured surface overlap branches.
        used = {root(k) for k in assignments.values()}
        candidates = []
        for i,t in enumerate(observed):
            if i in assignments:
                continue
            for k,e in self.encounters.items():
                for s in e['surfaces']:
                    da = abs(t['angle_deg']-s['angle_deg'])
                    distance = math.hypot(t['x_m']-s['x_m'],t['y_m']-s['y_m'])
                    if da <= self.cfg['encounter_angle_gate_deg'] and distance <= self.cfg['encounter_distance_gate_m']:
                        same = (generation,t['id']) == s['track_key']
                        candidates.append((distance+da/180-(1 if same else 0),i,root(k)))
        for _,i,k in sorted(candidates):
            if i not in assignments and k not in used:
                assignments[i] = k
                used.add(k)
        merged = {}
        for k,e in self.encounters.items():
            r = root(k)
            if r != k:
                self.events.append({'event':'merge_history','from_encounter':k,'into_encounter':r})
            if r not in merged:
                merged[r] = dict(e, surfaces=list(e['surfaces']), track_keys=set(e.get('track_keys',set())))
            else:
                m = merged[r]
                if e['warned']:
                    m['until'] = min(m['until'],e['until']) if m['warned'] else e['until']
                    m['warned'] = True
                m['surfaces'].extend(e['surfaces'])
                m['track_keys'].update(e.get('track_keys',set()))
        self.encounters = merged
        by_track, surfaces = {}, {}
        for i,t in enumerate(observed):
            k = root(assignments[i]) if i in assignments else self.next_id
            if i not in assignments:
                self.next_id += 1
                self.encounters[k] = {'warned':False,'until':0,'missing_since':None,'track_keys':set(),'clear_since':None,'clear_notified':False}
                self.events.append({'event':'new_encounter','encounter_id':k,
                                    'track_id':t['id'],'generation':generation})
            by_track[t['id']] = k
            surfaces.setdefault(k,[]).append({**t,'track_key':(generation,t['id'])})
        self.track_encounters = by_track
        for k,e in self.encounters.items():
            if k in surfaces:
                e.update(surfaces=surfaces[k], missing_since=None)
                e['track_keys'].update(s['track_key'] for s in surfaces[k])
            elif e.get('missing_since') is None:
                e['missing_since'] = now
        eligible={by_track[t['id']] for t in threats if t['id'] in by_track}
        for k,e in self.encounters.items():
            if e['warned'] and k not in eligible:
                e['until']=min(e['until'],now)
        active, suppressed = [], []
        sustained = self.cfg.get('reminder_enabled', False)
        # Departure requires observed geometric clearance, not lost CV support.
        for k,e in self.encounters.items():
            if k in eligible:
                if e.get('clear_notified'):
                    e.update(warned=False, until=0, phase=None)
                    self.events.append({'event':'rearmed_after_departure','encounter_id':k})
                e['clear_since']=None
                e['clear_notified']=False
            elif sustained and e['warned'] and k in surfaces:
                pts=[p for t in surfaces[k] for p in t.get('points',[])]
                hazardous=any(p['dist_m'] <= self.cfg['unknown_distance_m'] or
                    (p['dist_m'] < self.cfg['warning_distance_m'] and p['y_m'] >= 0 and
                     abs(p['x_m']) <= self.cfg['corridor_half_width_m']) for p in pts)
                if pts and not hazardous:
                    if e.get('clear_since') is None:e['clear_since']=now
                    if now-e['clear_since'] >= self.cfg.get('departure_confirm_s',.5):self._cue(k,e,now)
                else:e['clear_since']=None
        for t in threats:
            k = by_track.get(t['id'])
            if k is None:continue
            e = self.encounters[k]
            e['last_threat']=dict(t)
            if not e['warned']:
                e.update(warned=True, until=now+self.cfg['burst_duration_s'])
                self.events.append({'event':'first_warning','encounter_id':k,
                                    'track_id':t['id'],'deadline':e['until']})
            phase='initial' if now<e['until'] else 'reminder'
            if phase=='initial' or sustained:
                command=dict(t, encounter_id=k, burst_until=e['until'], haptic_phase=phase)
                if phase=='reminder':command['duty_override']=self.cfg.get('reminder_duty',.25)
                active.append(command)
                if e.get('phase')!=phase:
                    self.events.append({'event':'haptic_phase','encounter_id':k,'phase':phase})
                e['phase']=phase
            if phase=='reminder':suppressed.append(t['id'])  # Full burst remains suppressed.
        deadline=min((t['burst_until'] for t in active if t['haptic_phase']=='initial'),default=None)
        # Departure uses one dedicated directional motor. Other bearings may
        # continue warning; a hazard owning this motor always has priority.
        pulse=self.cfg.get('departure_pulse_s',.3)
        gap=self.cfg.get('departure_gap_s',.25)
        occupied=motor_duties(threats,self.cfg)
        remaining=[]
        for cue in self.clear_cues:
            motor=max(0,min(4,int(math.floor((cue['angle_deg']+90)/45+.5))))
            bearing=-90+45*motor
            own_return=(cue['encounter_id'] in eligible or by_track.get(cue['id']) in eligible)
            if own_return:
                self.events.append({'event':'departure_cue_cancelled','reason':'same_encounter_returned','encounter_id':cue['encounter_id']})
                continue
            if cue['cue_started'] is None and now-cue['cue_queued']>=self.cfg.get('departure_wait_s',3):
                self.events.append({'event':'departure_cue_expired','encounter_id':cue['encounter_id']})
                continue
            if occupied[motor] > .02:
                if cue['cue_started'] is not None:
                    self.events.append({'event':'departure_cue_cancelled','reason':'motor_taken_by_hazard','encounter_id':cue['encounter_id']})
                    continue
                if cue['cue_stage']!='deferred':
                    self.events.append({'event':'departure_cue_deferred','encounter_id':cue['encounter_id'],'motor_index':motor})
                    cue['cue_stage']='deferred'
                remaining.append(cue)
                continue
            if cue['cue_started'] is None:cue['cue_started']=now
            elapsed=now-cue['cue_started']
            if elapsed>=2*pulse+gap:
                self.events.append({'event':'departure_cue_completed' if len(cue['cue_pulses'])==2 else 'departure_cue_incomplete',
                                    'encounter_id':cue['encounter_id'],'commanded_pulses':len(cue['cue_pulses'])})
                continue
            stage='pulse1' if elapsed<pulse else ('gap' if elapsed<pulse+gap else 'pulse2')
            if stage!=cue['cue_stage']:
                self.events.append({'event':'departure_cue_stage','stage':stage,'encounter_id':cue['encounter_id'],'motor_index':motor})
                cue['cue_stage']=stage
                if stage in ('pulse1','pulse2'):cue['cue_pulses'].append(stage)
            remaining.append(cue)
            if stage!='gap':
                edge=cue['cue_started']+(pulse if stage=='pulse1' else 2*pulse+gap)
                active.append(dict(cue,angle_deg=bearing,haptic_phase='departure',duty_override=self.cfg.get('departure_duty',1)))
                deadline=min(deadline,edge) if deadline is not None else edge
            occupied[motor]=1  # Serialize cues across both pulses AND the gap.
        self.clear_cues=remaining
        return active, deadline, suppressed
