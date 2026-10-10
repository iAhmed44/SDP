"""Synthetic geometry, dropout, tracking and motor lifetime regression tests."""
import copy
import json
import math
from pathlib import Path
import threading
import tempfile
import types
from unittest.mock import patch
import time
import unittest

from core import (Latest, LidarTracker, pixel_angle, calibrated_points, cluster_points,
                  associate, warning_threats, motor_duties)
from ranging.lidar_tracker import Revolution, LidarReader
from haptics.motor_driver import MotorDriver
from comms.nav_bridge import payload
from main_sensing import Controller, DemoReader, load_config

CONFIG = Path(__file__).resolve().parents[1]/'config.json'


def cluster(angle=0, distance=1):
    points=[]
    for a in (angle-.2, angle+.2):
        points.append({'angle_deg':a,'dist_m':distance,'x_m':distance*math.sin(math.radians(a)),
                       'y_m':distance*math.cos(math.radians(a))})
    return {'angle_deg':angle,'dist_m':distance,'x_m':sum(p['x_m'] for p in points)/2,
            'y_m':sum(p['y_m'] for p in points)/2,'angle_min_deg':angle-.2,
            'angle_max_deg':angle+.2,'count':2,'points':points}


def detection(name='person', angle=0):
    return {'name':name,'conf':.9,'angle_min_deg':angle-5,'angle_max_deg':angle+5,'bbox':[0,0,20,20]}


def track(angle=0, distance=1, confirmed=False):
    t=cluster(angle,distance)
    t.update(id=1,observed=True,cv_confirmed=confirmed,semantic_class='person',motion_state='STATIC')
    return t


def stalled_worker(cfg, out, stop):
    time.sleep(20)


class ConfigCase(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config(CONFIG)
        self.cfg["haptics"]["reminder_enabled"]=False


class GeometryTests(ConfigCase):
    def test_perspective_resolution_independence(self):
        for width in (640,1280,1920):
            self.assertAlmostEqual(pixel_angle(0,width,82.1),-41.05)
            self.assertAlmostEqual(pixel_angle(width,width,82.1),41.05)
            self.assertEqual(pixel_angle(width/2,width,82.1),0)
            self.assertGreater(abs(pixel_angle(width/4,width,82.1)),82.1/4)

    def test_invalid_returns_filtered(self):
        scan=[(40,0,1),(0,0,1),(40,0,None),(40,float('nan'),1),(40,0,float('inf'))]
        self.assertEqual(len(calibrated_points(scan,self.cfg['lidar'])),1)

    def test_orientation(self):
        self.cfg['lidar']['front_offset_deg']=90
        self.assertEqual(calibrated_points([(40,90,1)],self.cfg['lidar'])[0]['angle_deg'],0)
        self.cfg['lidar']['reverse']=True
        self.assertEqual(calibrated_points([(40,100,1)],self.cfg['lidar'])[0]['angle_deg'],-10)

    def test_nearest_return_preserved(self):
        points=calibrated_points([(40,0,1.4),(40,1,1.6),(40,2,1.7)],self.cfg['lidar'])
        c=cluster_points(points,self.cfg['clustering'])
        self.assertEqual(len(c),1)
        self.assertEqual(c[0]['dist_m'],1.4)
        self.assertEqual(len(c[0]['points']),3)


class FusionTests(ConfigCase):
    def vision(self,time=1):return {'valid':True,'timestamp':time,'detections':[detection()]}
    def test_fresh_supported(self):
        self.assertEqual(list(associate([cluster()],self.vision(),1,1.1,self.cfg)),[0])
    def test_stale_rejected(self):
        self.assertEqual(associate([cluster()],self.vision(),1,2,self.cfg),{})
    def test_time_skew_rejected(self):
        self.assertEqual(associate([cluster()],self.vision(),1.5,1.5,self.cfg),{})
    def test_outside_camera_rejected(self):
        self.assertEqual(associate([cluster(60)],self.vision(),1,1.1,self.cfg),{})
    def test_one_to_one(self):
        self.assertEqual(len(associate([cluster(),cluster(2)],self.vision(),1,1.1,self.cfg)),1)
    def test_translation_applied(self):
        self.cfg['camera']['x_m']=.5
        self.assertEqual(associate([cluster()],self.vision(),1,1.1,self.cfg),{})
    def test_confirmation_only_uses_supported_returns(self):
        c=cluster(0,1)
        c['points'].extend(cluster(25,.7)['points'])
        matched=associate([c],self.vision(),1,1.1,self.cfg)[0]
        self.assertEqual(len(matched['lidar_support_points']),2)
        t=track(0,1,True)
        t['points']=c['points'];t['cv_points']=matched['lidar_support_points']
        threat=warning_threats([t],self.cfg,True)[0]
        self.assertEqual(threat['dist_m'],1)
    def test_yaw_applied(self):
        self.cfg['camera']['yaw_deg']=20
        self.assertEqual(len(associate([cluster(20)],self.vision(),1,1.1,self.cfg)),1)


class WarningTests(ConfigCase):
    def threats(self,t,healthy=True):return warning_threats([t],self.cfg,healthy)
    def test_confirmed_static_warns(self):self.assertTrue(self.threats(track(0,1,True)))
    def test_unknown_normal_range_suppressed(self):self.assertFalse(self.threats(track(0,1)))
    def test_unknown_close_side_warns(self):self.assertTrue(self.threats(track(80,.4)))
    def test_confirmed_outside_corridor(self):self.assertFalse(self.threats(track(60,1,True)))
    def test_camera_dropout_fallback(self):self.assertTrue(self.threats(track(0,1),False))
    def test_coasting_never_actuates(self):
        t=track(0,.3,True);t['observed']=False
        self.assertFalse(self.threats(t))
    def test_pwm_interpolation_and_far_off(self):
        levels=motor_duties([{'angle_deg':22.5,'dist_m':.4}],self.cfg['haptics'])
        for a,b in zip(levels,[0,0,.59,.59,0]):self.assertAlmostEqual(a,b)
        self.assertEqual(motor_duties([{'angle_deg':0,'dist_m':1.5}],self.cfg['haptics']),[0]*5)


class TrackingTests(ConfigCase):
    def setUp(self):
        super().setUp();self.tracker=LidarTracker(self.cfg)
    def test_one_to_one_track_assignment(self):
        self.tracker.update([cluster()],{},0)
        output=self.tracker.update([cluster(0,.9),cluster(1,1.1)],{},.1)
        self.assertEqual(len({t['id'] for t in output}),2)
    def test_semantic_memory_then_expiry(self):
        self.tracker.update([cluster()],{0:detection()},0)
        t=self.tracker.update([cluster()],{},.5)[0]
        self.assertEqual(t['semantic_class'],'person')
        t=self.tracker.update([cluster()],{},1.1)[0]
        self.assertEqual(t['semantic_class'],'obstacle')
    def test_expire_before_matching(self):
        old=self.tracker.update([cluster()],{},0)[0]['id']
        self.assertNotEqual(old,self.tracker.update([cluster()],{},3)[0]['id'])
    def test_single_approaching_person_not_ego_reference(self):
        self.tracker.update([cluster(0,2)],{0:detection()},0)
        t=self.tracker.update([cluster(0,1.9)],{0:detection()},.1)[0]
        self.assertFalse(self.tracker.ego_valid)
        self.assertLess(t['v_rel'],0)
        self.assertEqual(t['motion_state'],'UNKNOWN')
    def test_guarded_ego_and_empty_reset(self):
        angles=(-25,0,25)
        self.tracker.update([cluster(a,2) for a in angles],{},0)
        self.tracker.update([cluster(a,1.9) for a in angles],{},.1)
        self.assertTrue(self.tracker.ego_valid)
        self.assertGreater(self.tracker.ego_speed,.9)
        self.tracker.update([],{},.2)
        self.assertFalse(self.tracker.ego_valid)
        self.assertEqual(self.tracker.ego_speed,0)
    def test_static_patience_and_uncertainty_reset(self):
        clusters=[cluster(a,2) for a in (-25,0,25)]
        for i in range(24):output=self.tracker.update(clusters,{},i*.1)
        self.assertTrue(all(t['motion_state']=='STATIC' for t in output))
        output=self.tracker.update(clusters,{},2.4,True)
        self.assertTrue(all(t['motion_state']=='UNKNOWN' and t['stationary_since'] is None for t in output))
    def test_person_waits_seven_seconds(self):
        clusters=[cluster(a,2) for a in (-25,0,25,50)]
        for i in range(73):
            output=self.tracker.update(clusters,{3:detection()},i*.1)
            person=next(t for t in output if t['semantic_class']=='person')
            if i<71:self.assertEqual(person['motion_state'],'UNKNOWN')
        self.assertEqual(person['motion_state'],'STATIC')
    def test_velocity_uses_track_timestamp_after_coast(self):
        self.tracker.update([cluster()],{},0)
        self.tracker.update([],{},.1)
        t=self.tracker.update([cluster(0,.8)],{},.2)[0]
        self.assertAlmostEqual(t['v_rel'],-1)
    def test_long_velocity_gap_unknown(self):
        self.tracker.update([cluster()],{},0)
        t=self.tracker.update([cluster()],{},.8)[0]
        self.assertFalse(t['velocity_valid'])
        self.assertEqual(t['motion_state'],'UNKNOWN')


class AcquisitionTests(ConfigCase):
    def test_invalid_returns_still_complete_empty_scan(self):
        r=Revolution(.4,3)
        r.feed({'a_deg':350,'d_mm':500,'q':40},0)
        self.assertIsNone(r.feed({'a_deg':0,'d_mm':None,'q':0},.01))
        for a,t in ((120,.02),(240,.03),(350,.04)):
            self.assertIsNone(r.feed({'a_deg':a,'d_mm':None,'q':0},t))
        packet=r.feed({'a_deg':0,'d_mm':None,'q':0},.05)
        self.assertEqual(packet['points'],[])
        self.assertEqual(packet['sequence'],1)
        self.assertAlmostEqual(packet['timestamp'],.03)
    def test_stalled_partial_scan_discarded(self):
        r=Revolution(.4,3)
        r.feed({'a_deg':350,'d_mm':500},0)
        r.feed({'a_deg':0,'d_mm':500},.01)
        r.feed({'a_deg':350,'d_mm':500},.5)
        self.assertIsNone(r.feed({'a_deg':0,'d_mm':500},.6))
    def test_hung_child_close_bounded(self):
        r=LidarReader(self.cfg['lidar'],target=stalled_worker)
        r.start();start=time.monotonic()
        r.close()
        self.assertLess(time.monotonic()-start,3)
        self.assertIsNone(r.process)


class FakeMotor:
    def __init__(self,pin):self.value=0;self.closed=False
    def close(self):self.closed=True


class MotorTests(ConfigCase):
    def test_independent_watchdog_clears_without_controller(self):
        h=copy.deepcopy(self.cfg['haptics']);h['heartbeat_timeout_s']=.05
        d=MotorDriver(h,True,FakeMotor)
        try:
            d.apply_feedback([{'angle_deg':0,'dist_m':.4}])
            self.assertEqual(d.motors[2].value,1)
            deadline=time.monotonic()+.5
            while d.duties!=[0]*5 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(d.duties,[0]*5)
            self.assertTrue(all(m.value==0 for m in d.motors))
        finally:d.close()
    def test_partial_initialization_closes_created_devices(self):
        devices=[]
        def factory(pin):
            if len(devices)==2:raise RuntimeError('GPIO failure')
            m=FakeMotor(pin);devices.append(m);return m
        with self.assertRaises(RuntimeError):MotorDriver(self.cfg['haptics'],True,factory)
        self.assertTrue(all(m.closed and m.value==0 for m in devices))


class ControllerTests(ConfigCase):
    def run_controller(self,reader):
        stop=threading.Event();vision,state=Latest(),Latest()
        ctrl=Controller(self.cfg,stop,vision,state,reader=reader)
        ctrl.thread.start()
        try:
            deadline=time.monotonic()+1
            while state.get() is None and time.monotonic()<deadline:time.sleep(.01)
            result=state.get()
            self.assertIsNotNone(result)
            return result
        finally:stop.set();ctrl.close()
    def test_warning_loop_independent_of_absent_inference(self):
        state=self.run_controller(DemoReader())
        self.assertTrue(state['lidar_healthy'])
        self.assertFalse(state['vision_healthy'])
        self.assertGreater(max(state['duties']),0)
    def test_stale_lidar_clears_outputs(self):
        class Stale(DemoReader):
            def poll(self,now=None):
                p=super().poll(now);p['timestamp']-=2;return p
        state=self.run_controller(Stale())
        self.assertFalse(state['lidar_healthy'])
        self.assertEqual(state['duties'],[0]*5)
        self.assertEqual(state['tracks'],[])


class NavigationTests(ConfigCase):
    def test_empty_snapshot_and_validity(self):
        p=payload([],True,0,False,.75)
        self.assertTrue(p['snapshot']);self.assertTrue(p['valid'])
        self.assertEqual(p['static_obstacles'],[])
        json.dumps(p,allow_nan=False)
        self.assertEqual(payload([track()],False,0,False,.75)['static_obstacles'],[])
    def test_coasting_excluded_and_default_disabled(self):
        t=track();t['observed']=False
        self.assertEqual(payload([t],True,0,True,.75)['static_obstacles'],[])
        self.assertFalse(self.cfg['navigation']['enabled'])


class CameraTests(ConfigCase):
    def fake_inference(self, fail_capture=False, delay=0):
        import numpy as np
        from cv.yolo_multiplexer import YOLOMultiplexer
        stop=threading.Event();slot=Latest()
        class Tensor:
            def __init__(self, value):self.value=value
            def cpu(self):return self
            def tolist(self):return self.value
            def item(self):return self.value
        box=types.SimpleNamespace(xyxy=[Tensor([320,0,960,720])],cls=[Tensor(0)],conf=[Tensor(.9)])
        result=types.SimpleNamespace(boxes=[box],names={0:'person'})
        seen={}
        with tempfile.TemporaryDirectory() as directory:
            Path(directory,'yolo26n.pt').touch()
            worker=YOLOMultiplexer(self.cfg['camera'],directory,stop,slot)
            worker.frames.set({'sequence':1,'timestamp':time.monotonic(),'frame':np.zeros((720,1280,3),np.uint8)})
            def model(frame,**kwargs):
                seen['shape']=frame.shape;seen['kwargs']=kwargs
                if delay:time.sleep(delay)
                if fail_capture:worker.frames.set(None)
                stop.set()
                return [result]
            with patch.dict('sys.modules',{'ultralytics':types.SimpleNamespace(YOLO=lambda path:model)}):
                worker._infer()
        return slot.get(),seen
    def test_actual_720p_frame_and_angles(self):
        result,seen=self.fake_inference()
        self.assertTrue(result['valid'])
        self.assertEqual(seen['shape'],(720,1280,3))
        self.assertEqual(result['width'],1280)
        self.assertGreaterEqual(result['frame_to_result_ms'],result['inference_ms'])
        self.assertGreaterEqual(result['postprocess_ms'],0)
        self.assertIsNone(result['result_interval_ms'])
        self.assertAlmostEqual(result['detections'][0]['angle_min_deg'],pixel_angle(320,1280,82.1))
        self.assertEqual(seen['kwargs']['imgsz'],416)
    def test_disconnect_during_inference_invalidates_result(self):
        result,_=self.fake_inference(fail_capture=True)
        self.assertFalse(result['valid'])
    def test_slow_inference_expires_source_frame(self):
        self.cfg['camera']['max_result_age_s']=.001
        result,_=self.fake_inference(delay=.02)
        self.assertFalse(result['valid'])



class BurstTests(ConfigCase):
    def setUp(self):
        super().setUp()
        from haptics.encounters import EncounterWarnings
        self.bursts=EncounterWarnings(self.cfg['haptics'])
    def select(self,t,now,healthy=True,generation=1):
        tracks=[t] if t else []
        threats=warning_threats(tracks,self.cfg,False) if t else []
        return self.bursts.select(tracks,threats,now,healthy,generation)
    def test_one_point_five_seconds_then_silent(self):
        t=track(0,.5)
        active,end,_=self.select(t,0)
        self.assertTrue(active);self.assertEqual(end,1.5)
        self.assertTrue(self.select(t,1.49)[0])
        active,end,suppressed=self.select(t,1.5)
        self.assertEqual(active,[]);self.assertEqual(suppressed,[1])
        self.assertFalse(self.select(t,2.9)[0])
        self.assertFalse(self.select(t,10)[0])
    def test_id_change_and_generation_change_do_not_repeat(self):
        t=track(0,.5);self.select(t,0)
        t['id']=99
        self.assertFalse(self.select(t,1.6,generation=2)[0])
    def test_short_dropout_no_new_burst(self):
        t=track(0,.5);self.select(t,0)
        self.select(None,1)
        self.assertFalse(self.select(t,2)[0])
    def test_actual_absence_rearms(self):
        t=track(0,.5);self.select(t,0)
        self.select(None,1)
        self.select(None,4.1)
        self.assertTrue(self.select(t,4.2)[0])
    def test_sensor_outage_does_not_rearm(self):
        t=track(0,.5);self.select(t,0)
        self.select(None,1,False)
        self.select(None,10,False)
        self.assertFalse(self.select(t,10.1)[0])
    def test_new_obstacle_warns_even_if_previous_suppressed(self):
        a,b=track(-30,.5),track(30,.5);b['id']=2
        self.select(a,0)
        tracks=[a,b];threats=warning_threats(tracks,self.cfg,False)
        active,_,suppressed=self.bursts.select(tracks,threats,2,True,1)
        self.assertEqual([t['id'] for t in active],[2]);self.assertEqual(suppressed,[1])
    def test_burst_stops_early_if_hazard_disappears(self):
        self.select(track(0,.5),0)
        self.assertFalse(self.select(None,.1)[0])
    def test_outside_warning_distance_does_not_rearm(self):
        t=track(0,.5);self.select(t,0)
        for i in range(1,25):
            t=track(0,min(2,.5+i*.1));self.select(t,i*.1)
        for i in range(25,40):
            t=track(0,max(.5,2-(i-24)*.1));output=self.select(t,i*.1)
        self.assertFalse(output[0])
    def test_motor_deadline_enforced_independently(self):
        h=copy.deepcopy(self.cfg['haptics']);h['heartbeat_timeout_s']=2
        driver=MotorDriver(h,True,FakeMotor)
        try:
            driver.apply_feedback([{'angle_deg':0,'dist_m':.4}],time.monotonic()+.05)
            deadline=time.monotonic()+.5
            while max(driver.duties)>0 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(driver.duties,[0]*5)
        finally:driver.close()


class PoseTests(ConfigCase):
    def pose(self,timestamp=1):
        return dict(detection(),pose_timestamp=timestamp,pose_keypoints=[[i+1,i+2] for i in range(17)],pose_confidence=[.9]*17)
    def test_pose_association_and_expiry(self):
        from cv.pose_support import attach_pose,pose_evidence
        p=self.pose()
        d=attach_pose([detection()], [p],1.1,.7,.3)[0]
        self.assertEqual(pose_evidence(d,1.1,self.cfg['camera'])['visible_keypoints'],17)
        self.assertIsNone(pose_evidence(d,2,self.cfg['camera']))
        self.assertNotIn('pose_timestamp',attach_pose([detection()],[p],2,.7,.3)[0])
    def test_one_to_one_pose_assignment(self):
        from cv.pose_support import attach_pose
        d=attach_pose([detection(),detection()], [self.pose()],1.1,.7,.3)
        self.assertEqual(sum('pose_timestamp' in x for x in d),1)
    def test_keypoint_low_confidence_does_not_imply_orientation(self):
        from cv.pose_support import pose_evidence
        d=self.pose();d['pose_confidence']=[.1]*17
        evidence=pose_evidence(d,1.1,self.cfg['camera'])
        self.assertEqual(evidence['visible_keypoints'],0)
        self.assertEqual(evidence['facing'],'UNKNOWN')
        self.assertEqual(evidence['intent'],'UNKNOWN')
    def test_cached_object_timestamp_not_rejuvenated_by_pose(self):
        v={'valid':True,'timestamp':1.5,'detections':[dict(detection(),timestamp=1)]}
        self.assertFalse(associate([cluster()],v,1.5,1.5,self.cfg))
    def test_default_pose_enabled_every_third_cycle(self):self.assertEqual(self.cfg['camera']['pose_every'],3)

    def test_real_worker_schedules_od_od_pose_and_reuses_person_box(self):
        import numpy as np
        from cv.yolo_multiplexer import YOLOMultiplexer
        class Tensor:
            def __init__(self,v):self.v=v
            def cpu(self):return self
            def tolist(self):return self.v
            def item(self):return self.v
        def box(coords,cls):
            return types.SimpleNamespace(xyxy=[Tensor(coords)],cls=[Tensor(cls)],conf=[Tensor(.9)])
        human=box([100,100,300,600],0);chair=box([500,200,700,600],1)
        od_result=types.SimpleNamespace(boxes=[human,chair],names={0:'person',1:'chair'})
        kp=types.SimpleNamespace(xy=Tensor([[[110+i,150+i] for i in range(17)]]),conf=Tensor([[.9]*17]))
        pose_result=types.SimpleNamespace(boxes=[human],names={0:'person'},keypoints=kp)
        stop=threading.Event();slot=Latest();records=[];tasks=[]
        original_set=slot.set
        def record(value):records.append(value);original_set(value)
        slot.set=record
        with tempfile.TemporaryDirectory() as directory:
            for name in ('yolo26n.pt','yolo26n-pose.pt'):Path(directory,name).touch()
            worker=YOLOMultiplexer(self.cfg['camera'],directory,stop,slot)
            frame=np.zeros((720,1280,3),np.uint8)
            worker.frames.set({'sequence':1,'timestamp':time.monotonic(),'frame':frame})
            def run(task):
                def model(frame,**kwargs):
                    tasks.append(task)
                    worker.frames.set({'sequence':len(tasks)+1,'timestamp':time.monotonic(),'frame':frame})
                    if len(tasks)==4:stop.set()
                    return [pose_result if task=='pose' else od_result]
                return model
            def load(path):return run('pose' if '-pose' in path else 'detect')
            with patch.dict('sys.modules',{'ultralytics':types.SimpleNamespace(YOLO=load)}):worker._infer()
        self.assertEqual(tasks,['detect','detect','pose','detect'])
        third=records[2]
        self.assertEqual(sorted(d['name'] for d in third['detections']),['chair','person'])
        person=next(d for d in third['detections'] if d['name']=='person')
        self.assertEqual(len(person['pose_keypoints']),17)
        old_chair=next(d for d in third['detections'] if d['name']=='chair')
        self.assertLess(old_chair['timestamp'],third['timestamp'])
        self.assertIn('pose_keypoints',next(d for d in records[3]['detections'] if d['name']=='person'))



class BurstControllerTests(ConfigCase):
    def test_persistent_demo_obstacle_goes_silent_after_burst(self):
        stop=threading.Event();vision,state=Latest(),Latest()
        controller=Controller(self.cfg,stop,vision,state,reader=DemoReader())
        controller.thread.start()
        try:
            deadline=time.monotonic()+1
            while state.get() is None and time.monotonic()<deadline:time.sleep(.01)
            self.assertGreater(max(state.get()['duties']),0)
            time.sleep(1.65)
            result=state.get()
            self.assertTrue(result['threats'])
            self.assertFalse(result['active_threats'])
            self.assertTrue(result['suppressed_ids'])
            self.assertEqual(result['duties'],[0]*5)
        finally:stop.set();controller.close()


if __name__=='__main__':unittest.main()
