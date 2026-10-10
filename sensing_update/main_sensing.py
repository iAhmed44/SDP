#!/usr/bin/env python3
"""EE499 sensing: independent warnings + camera/LiDAR fusion diagnostics.
Default DRY RUN, no GPIO access; --motors enables physical outputs.
Navigation is optional. Run from Sensing_SubS using its existing venv.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from core import Latest, LidarTracker, calibrated_points, cluster_points, associate, warning_threats
from haptics.motor_driver import MotorDriver
from haptics.encounters import EncounterWarnings
from cv.pose_support import pose_evidence
from ranging.lidar_tracker import LidarReader
from sensing_logger import SensingLogger, LoggedLatest
from version import VERSION


def load_config(path):
    default_path=Path(__file__).with_name('config.defaults.json')
    cfg=json.loads(default_path.read_text())
    def merge(base, override):
        for key,value in override.items():
            if isinstance(value,dict) and isinstance(base.get(key),dict):merge(base[key],value)
            else:base[key]=value
    path=Path(path)
    if path.is_file():merge(cfg,json.loads(path.read_text()))
    elif path != Path(__file__).with_name('config.json'):
        raise FileNotFoundError(path)
    c,l,h=cfg['camera'],cfg['lidar'],cfg['haptics']
    if not 1<c['hfov_deg']<179 or c['width']<=0 or c['height']<=0:
        raise ValueError('Invalid camera dimensions/FOV')
    if len(h['pins'])!=5 or len(set(h['pins']))!=5:
        raise ValueError('Five distinct motor GPIOs are required')
    if not 0<h['full_power_distance_m']<h['warning_distance_m'] or not 0<h['unknown_distance_m']<=h['warning_distance_m']:
        raise ValueError('Invalid haptic distances')
    if h.get('burst_duration_s',0)<=0 or h.get('rearm_absence_s',0)<=0:
        raise ValueError('Positive burst duration and rearm absence are required')
    if not 0<h['burst_duration_s']<=1.5:
        raise ValueError('Initial burst must be at most 1.5 seconds')
    for key in ('reminder_duty','departure_duty'):
        if not isinstance(h[key],(int,float)) or not math.isfinite(h[key]) or not 0<h[key]<=1:
            raise ValueError('Invalid haptic duty: '+key)
    for key in ('departure_confirm_s','departure_pulse_s','departure_gap_s','departure_wait_s'):
        if not isinstance(h[key],(int,float)) or not math.isfinite(h[key]) or h[key]<=0:
            raise ValueError('Invalid haptic duration: '+key)
    if c.get('pose_every',0)<0 or not isinstance(c.get('pose_every',0),int):
        raise ValueError('pose_every must be a nonnegative integer')
    if h['policy'] not in ('fusion','lidar') or h['control_hz']<=0 or h['corridor_half_width_m']<=0:
        raise ValueError('Invalid warning policy or control settings')
    if not 0<l['min_range_m']<l['max_range_m'] or l['max_scan_age_s']<=0:
        raise ValueError('Invalid LiDAR range/expiry')
    options=cfg['logging']
    if options['mode'] not in ('compact','detailed'):
        raise ValueError('logging.mode must be compact or detailed')
    for key in ('snapshot_hz','checkpoint_s','max_part_mb'):
        if not isinstance(options[key],(int,float)) or not math.isfinite(options[key]) or options[key]<=0:
            raise ValueError('Invalid positive logging setting: '+key)
    if type(options['queue_size']) is not int or options['queue_size']<1:
        raise ValueError('logging.queue_size must be a positive integer')
    nav=cfg['navigation']
    if not 0<nav['ttl_s']<=.75 or not 0<nav['max_source_age_s']<=.4 or nav['publish_hz']<=0:
        raise ValueError('Invalid navigation TTL/source-age/publish rate')
    if not 0<l['min_scan_duration_s']<l['max_scan_duration_s']:
        raise ValueError('Invalid scan timing interval')
    return cfg


class Controller:
    def __init__(self,cfg,stop,vision,state,motors=False,reader=None,logger=None):
        self.cfg,self.stop,self.vision,self.state=cfg,stop,vision,state
        self.reader=reader if reader is not None else LidarReader(cfg['lidar'])
        self.motors=motors
        self.logger=logger
        self.driver=None
        self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True)

    def run(self):
        from comms.nav_bridge import NavBridge
        tracker=LidarTracker(self.cfg)
        encounters=EncounterWarnings(self.cfg['haptics'])
        bridge=None
        last_sequence=None
        last_generation=None
        tracks=[]
        points=[]
        last_print=0
        try:
            self.driver=MotorDriver(self.cfg['haptics'],self.motors)
            if self.cfg['navigation']['enabled']:
                try:bridge=NavBridge(self.cfg['navigation'])
                except Exception as exc:print('[NAV ERROR]',exc,'; sensing continues',flush=True)
            while not self.stop.is_set():
                now=time.monotonic()
                packet=self.reader.poll(now)
                scan_age=now-packet['timestamp'] if packet else float('inf')
                lidar_healthy=packet is not None and 0<=scan_age<=self.cfg['lidar']['max_scan_age_s']
                vision=self.vision.get()
                vision_age=now-vision['timestamp'] if vision else float('inf')
                vision_healthy=bool(vision and vision.get('valid') and 0<=vision_age<=self.cfg['camera']['max_result_age_s'])
                if not lidar_healthy:
                    tracks,points=[],[]
                    tracker.reset()
                    last_sequence=None
                    threats=[]
                else:
                    if packet['generation']!=last_generation:
                        tracker.reset()
                        last_sequence=None
                        last_generation=packet['generation']
                    if packet['sequence']!=last_sequence:
                        points=calibrated_points(packet['points'],self.cfg['lidar'])
                        clusters=cluster_points(points,self.cfg['clustering'])
                        matches=associate(clusters,vision,packet['timestamp'],now,self.cfg)
                        uncertain=not vision_healthy or vision.get('flow',{}).get('uncertain',True)
                        tracks=tracker.update(clusters,matches,packet['timestamp'],uncertain,
                            timing_valid=packet.get('timing_valid',False))
                        last_sequence=packet['sequence']
                    # Old CV confirmation must never survive loss of vision.
                    current=copy.deepcopy(tracks)
                    for t in current:
                        t['pose']=None
                        t['relative_motion']='UNKNOWN'
                    if not vision_healthy:
                        for t in current:t['cv_confirmed']=False
                    else:
                        # Refresh matching against the current timestamped result, without redoing velocity.
                        indices=[i for i,t in enumerate(current) if t['observed']]
                        fresh_matches=associate([current[i] for i in indices],vision,packet['timestamp'],now,self.cfg)
                        for i,t in enumerate(current):t['cv_confirmed']=False
                        for j,d in fresh_matches.items():
                            current[indices[j]]['cv_confirmed']=True
                            current[indices[j]]['cv_points']=d['lidar_support_points']
                            target=current[indices[j]]
                            target['semantic_class']=d['name']
                            target['pose']=pose_evidence(d,now,self.cfg['camera'])
                            if (d['name']=='person' and target.get('velocity_valid') and
                                    not vision.get('flow',{}).get('uncertain',True)):
                                speed=self.cfg['tracking']['moving_speed_mps']
                                if target['v_rel'] < -speed:target['relative_motion']='APPROACHING_RELATIVE'
                                elif target['v_rel'] > speed:target['relative_motion']='RECEDING_RELATIVE'
                                elif abs(target['vx_rel'])>speed:target['relative_motion']='LATERAL_RELATIVE'
                    threats=warning_threats(current,self.cfg,vision_healthy)
                    tracks=current
                active,deadline,suppressed=encounters.select(tracks,threats,now,lidar_healthy,
                    packet['generation'] if packet else last_generation)
                self.driver.apply_feedback(active,valid_until=deadline)
                if self.driver.stop.is_set():
                    raise RuntimeError('Motor watchdog stopped')
                result={'points':points,'tracks':tracks,'threats':threats,'duties':list(self.driver.duties),
                        'encounter_events':list(encounters.events),'encounter_by_track':dict(encounters.track_encounters),
                        'active_threats':active,'suppressed_ids':suppressed,'lidar_healthy':lidar_healthy,'vision_healthy':vision_healthy,
                        'scan_timing_valid':bool(packet and packet.get('timing_valid',False)),
                        'scan_age_s':scan_age if math.isfinite(scan_age) else -1,
                        'vision_age_s':vision_age if math.isfinite(vision_age) else -1,
                        'lidar_status':self.reader.status,'ego_speed':tracker.ego_speed,'ego_valid':tracker.ego_valid}
                self.state.set(result)
                if self.logger:self.logger.record_cycle(result,packet)
                if now-last_print>=.5:
                    if not lidar_healthy:
                        print('[STATUS] LiDAR unavailable/stale: directional outputs OFF |',self.reader.status,flush=True)
                    elif not vision_healthy:
                        print('[STATUS] Vision unavailable/stale: LiDAR corridor fallback',flush=True)
                    for t in tracks:
                        if t['observed']:
                            print(f"[OBJECT] ID={t['id']} {t['semantic_class']} angle={t['angle_deg']:+.1f}deg distance={t['dist_m']:.2f}m state={t['motion_state']} cv={t.get('cv_confirmed',False)} pose={bool(t.get('pose'))} relative={t.get('relative_motion','UNKNOWN')}",flush=True)
                    for t in active:
                        print(f"[WARNING] ID={t['id']} encounter={t['encounter_id']} phase={t.get('haptic_phase','initial')} {t['reason']} {t['dist_m']:.2f}m",flush=True)
                    if suppressed:print('[HAPTICS] Already warned this encounter:',suppressed,flush=True)
                    last_print=now
                if bridge:
                    bridge.publish(tracks,lidar_healthy,tracker.ego_speed,tracker.ego_valid,packet['timestamp'] if packet else None)
                self.stop.wait(max(0,now+1/self.cfg['haptics']['control_hz']-time.monotonic()))
        except Exception as exc:
            self.error=exc
            print('[CONTROL ERROR]',repr(exc),flush=True)
            self.stop.set()
        finally:
            # Actuation stops BEFORE waiting for sensing workers or network cleanup.
            if self.driver:
                try:
                    zero_ok=self.driver.close()
                    if self.logger:self.logger.record_shutdown(self.motors,zero_ok,repr(self.error) if self.error else 'normal_stop')
                except Exception as exc:
                    print('[HAPTIC CLEANUP]',exc,flush=True)
                    if self.logger:self.logger.record_shutdown(self.motors,False,str(exc))
            elif self.logger:
                self.logger.record_shutdown(self.motors,not self.motors,'motor_driver_not_initialized')
            if bridge:
                try:bridge.close()
                except Exception as exc:print('[NAV CLEANUP]',exc,flush=True)

    def close(self):
        # Immediately clear from the owner, even if acquisition supervision is waiting.
        if self.driver:
            try:
                self.driver.clear()
            except Exception as exc:
                print('[HAPTIC CLEANUP]', exc, flush=True)
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise RuntimeError('Controller did not stop; watchdog must keep motor outputs off')
        self.reader.close()


class DemoReader:
    """No hardware; provides deterministic scans for the display and tests."""
    status='SIMULATED scan'
    def __init__(self):self.n=0
    def poll(self,now=None):
        now=time.monotonic() if now is None else now
        self.n+=1
        points=[(40,float(a%360),.5 if 18<=a<=24 else 2.3) for a in range(-90,91)]
        return {'sequence':self.n,'generation':1,'timestamp':now,'scan_start':now-.1,'scan_duration':.1,'timing_valid':True,'points':points}
    def close(self):pass


def main():
    base=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version',action='version',version='Sensing '+VERSION)
    parser.add_argument('--config',default=str(base/'config.json'))
    parser.add_argument('--models',default=str(base.parent/'cv'),help='Existing folder with yolo26n.pt')
    parser.add_argument('--nav',action='store_true',help='Enable the local ZeroMQ NavBridge')
    parser.add_argument('--log-mode',choices=['compact','detailed'],help='Compact is the default; detailed includes raw scans')
    parser.add_argument('--motors',action='store_true',help='Enable physical GPIO output; default is DRY RUN')
    parser.add_argument('--no-display',action='store_true')
    parser.add_argument('--lidar-only',action='store_true')
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--snapshot',help='Save one simulated diagnostic PNG; no hardware')
    parser.add_argument('--imgsz',type=int,help='Override YOLO input size (default 416)')
    args=parser.parse_args()
    cfg=load_config(args.config)
    if args.nav:cfg['navigation']['enabled']=True
    if args.log_mode:cfg['logging']['mode']=args.log_mode
    if args.demo or args.snapshot:cfg['navigation']['enabled']=False
    if args.imgsz:
        if not 160<=args.imgsz<=1280:parser.error('--imgsz must be between 160 and 1280')
        cfg['camera']['imgsz']=args.imgsz
    if (args.demo or args.snapshot) and args.motors:
        parser.error('Demo/snapshot cannot enable motors')
    stop=threading.Event()
    logger=SensingLogger(base,cfg,args.motors) if not args.snapshot else None
    state=Latest()
    vision=LoggedLatest(logger) if logger else Latest()
    ctrl=None
    camera=None
    # Resolve the previously observed missing Wayland Qt plugin without changing the venv.
    if not args.no_display or args.snapshot:
        if os.environ.get('QT_QPA_PLATFORM') in (None,'wayland'):
            os.environ['QT_QPA_PLATFORM']='xcb'
        from ui import render,show,cv2,WINDOW
        font_dir='/usr/share/fonts/truetype/dejavu'
        if Path(font_dir).is_dir():os.environ['QT_QPA_FONTDIR']=font_dir
    if args.snapshot:
        reader=DemoReader()
        packet=reader.poll()
        points=calibrated_points(packet['points'],cfg['lidar'])
        clusters=cluster_points(points,cfg['clustering'])
        tracks=LidarTracker(cfg).update(clusters,{},packet['timestamp'],True)
        from core import motor_duties
        threats=warning_threats(tracks,cfg,False)
        active,_,suppressed=EncounterWarnings(cfg['haptics']).select(tracks,threats,packet['timestamp'])
        result={'points':points,'tracks':tracks,'threats':threats,'active_threats':active,'suppressed_ids':suppressed,'duties':motor_duties(active,cfg['haptics']),
                'lidar_healthy':True,'vision_healthy':False,'lidar_status':'SIMULATED','scan_age_s':0}
        if not cv2.imwrite(args.snapshot,render(result,None,cfg,False)):raise RuntimeError('Snapshot write failed')
        return
    if args.demo:cfg['navigation']['enabled']=False
    def stop_signal(signum,frame):stop.set()
    signal.signal(signal.SIGTERM,stop_signal)
    signal.signal(signal.SIGINT,stop_signal)
    try:
        if not args.no_display:
            cv2.namedWindow(WINDOW,cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW,1500,860)
        ctrl=Controller(cfg,stop,vision,state,args.motors,DemoReader() if args.demo else None,logger=logger)
        ctrl.thread.start()
        if not (args.demo or args.lidar_only):
            from cv.yolo_multiplexer import YOLOMultiplexer
            camera=YOLOMultiplexer(cfg['camera'],args.models,stop,vision)
            camera.start()
        print(f"[RUN] Sensing {VERSION} | FOV={cfg['camera']['hfov_deg']}deg; capture={cfg['camera']['width']}x{cfg['camera']['height']} requested; inference size={cfg['camera']['imgsz']}",flush=True)
        print('[RUN] '+('MOTORS ENABLED' if args.motors else 'DRY RUN; motors OFF')+' | navigation '+('ENABLED' if cfg['navigation']['enabled'] else 'OFF'),flush=True)
        while not stop.is_set():
            if args.no_display:
                stop.wait(.1)
            else:
                if not show(state.get() or {},vision.get(),cfg,args.motors):stop.set()
    finally:
        stop.set()
        for owner in (ctrl, camera):
            if owner:
                try:
                    owner.close()
                except Exception as exc:
                    print('[CLEANUP ERROR]', repr(exc), flush=True)
                    if ctrl:ctrl.error = ctrl.error or exc
        if not args.no_display:cv2.destroyAllWindows()
        print('[STOP] Sensing stopped; see the HAPTICS shutdown record for the zero-command result.',flush=True)
        if logger:logger.close()
    if ctrl and ctrl.error:raise SystemExit(1)


if __name__=='__main__':
    main()
