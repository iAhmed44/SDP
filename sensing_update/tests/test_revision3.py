"""Regression tests for repeat warnings, timing, log recovery and IPC isolation."""
import copy
import gzip
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import types
import uuid
from unittest.mock import patch
from test_sensing import ConfigCase,track,cluster,FakeMotor,DemoReader
from core import Latest,LidarTracker,warning_threats
from haptics.encounters import EncounterWarnings
from haptics.motor_driver import MotorDriver
from comms.nav_bridge import NavBridge,SnapshotReceiver
from main_sensing import Controller
from sensing_logger import SensingLogger


class ContinuityTests(ConfigCase):
    def test_split_merge_preserves_warned_deadline_from_recorded_geometry(self):
        fixture=json.loads(Path(__file__).with_name('recorded_split.json').read_text())
        e=EncounterWarnings(self.cfg['haptics']);seen=[]
        for row in fixture:
            active,_,_=e.select(row['tracks'],row['threats'],row['time'],True,1)
            seen.extend((t['encounter_id'],t['burst_until']) for t in active)
        self.assertEqual(len(set(seen)),1)
        self.assertAlmostEqual(seen[0][1],4.685)
        row=fixture[-1]
        self.assertFalse(e.select(row['tracks'],row['threats'],5,True,1)[0])

    def test_back_and_forth_with_id_changes_does_not_repeat(self):
        e=EncounterWarnings(self.cfg['haptics']);deadlines=set()
        for i in range(101):
            distance=.7+(i%20 if i%40<20 else 40-i%40)*.03
            t=track(0,distance,True);t['id']=i+1
            active,_,_=e.select([t],warning_threats([t],self.cfg),i*.1,True,1)
            deadlines.update(a['burst_until'] for a in active)
            if i>=15:self.assertFalse(active)
        self.assertEqual(deadlines,{1.5})

    def test_early_end_cannot_resume_as_a_second_pulse(self):
        e=EncounterWarnings(self.cfg['haptics']);t=track(0,.5)
        threats=warning_threats([t],self.cfg)
        self.assertTrue(e.select([t],threats,0)[0])
        self.assertFalse(e.select([t],[],.1)[0])
        self.assertFalse(e.select([t],threats,.2)[0])

    def test_two_new_disjoint_obstacles_warn_independently(self):
        e=EncounterWarnings(self.cfg['haptics']);a,b=track(-50,.4),track(50,.4);b['id']=2
        e.select([a],warning_threats([a],self.cfg),0)
        active,_,suppressed=e.select([a,b],warning_threats([a,b],self.cfg),2)
        self.assertEqual([t['id'] for t in active],[2]);self.assertEqual(suppressed,[1])

    def test_bad_timing_and_next_scan_have_no_velocity(self):
        tracker=LidarTracker(self.cfg)
        tracker.update([cluster()],{},0)
        self.assertFalse(tracker.update([cluster(0,.9)],{},.1,timing_valid=False)[0]['velocity_valid'])
        self.assertFalse(tracker.update([cluster(0,.8)],{},.2,timing_valid=True)[0]['velocity_valid'])
        self.assertTrue(tracker.update([cluster(0,.7)],{},.3,timing_valid=True)[0]['velocity_valid'])

    def test_cluster_shape_change_invalidates_velocity(self):
        tracker=LidarTracker(self.cfg);tracker.update([cluster()],{},0)
        c=cluster();c['count']=10;c['angle_max_deg']=15
        result=tracker.update([c],{},.1)[0]
        self.assertFalse(result['velocity_valid']);self.assertEqual(result['motion_state'],'UNKNOWN')


class LogTests(ConfigCase):
    def test_grouping_compression_shutdown_and_raw_omission(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger=SensingLogger(tmp,self.cfg,capture_terminal=False)
            state={'tracks':[track()],'duties':[0,0,.4,0,0],'active_threats':[], 'lidar_healthy':True,'vision_healthy':True}
            logger.record_cv({'valid':True,'timestamp':1,'detections':[]})
            logger.record_cycle(state,{'sequence':1,'generation':1,'points':[(40,0,1)]})
            logger.record_shutdown(False,True,'test');logger.close()
            text=gzip.decompress(logger.path.read_bytes()).decode()
            self.assertIn('END OF RUN',text);self.assertIn('"event":"shutdown"',text)
            self.assertNotIn('raw_completed_scan',text)
            self.assertLess(text.index('--- CV'),text.index('--- LIDAR'))
            self.assertFalse(logger.pending.exists());self.assertFalse(logger.failed)

    def test_detailed_includes_raw_and_bones(self):
        self.cfg['logging']['mode']='detailed'
        with tempfile.TemporaryDirectory() as tmp:
            logger=SensingLogger(tmp,self.cfg,capture_terminal=False)
            logger.record_cv({'detections':[{'pose_keypoints':[[1,2]]}]})
            logger.record_cycle({},dict(sequence=1,generation=1,points=[(40,0,1)]))
            logger.close();text=gzip.decompress(logger.path.read_bytes()).decode()
            self.assertIn('raw_completed_scan',text);self.assertIn('pose_keypoints',text)

    def test_checkpoint_survives_process_kill(self):
        with tempfile.TemporaryDirectory() as tmp:
            script="""import time,json,sys
from pathlib import Path
from main_sensing import load_config
from sensing_logger import SensingLogger
c=load_config(Path('config.json').resolve());c['logging']['checkpoint_s']=.05
s=SensingLogger(sys.argv[1],c,capture_terminal=False)
s.record_cv({'valid':True,'detections':[]})
while True:time.sleep(.1)
"""
            proc=subprocess.Popen([sys.executable,'-u','-c',script,tmp],stdout=subprocess.DEVNULL)
            try:
                deadline=time.monotonic()+3;pending=[]
                while time.monotonic()<deadline:
                    pending=list(Path(tmp).rglob('*.pending'))
                    if pending:break
                    time.sleep(.02)
                self.assertTrue(pending)
                proc.kill();proc.wait(timeout=2)
                self.assertIn('"valid":true',pending[0].read_text())
            finally:
                if proc.poll() is None:proc.kill();proc.wait()

    def test_disk_error_is_reported_without_raising_in_control(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger=SensingLogger(tmp,self.cfg,capture_terminal=False)
            with patch.object(logger,'_finish_block',side_effect=OSError('disk full')):
                logger.record_shutdown(False,True,'test');logger.close()
            self.assertTrue(logger.failed)
            logger.record_cycle({})  # Failed logger must not break the caller.

    def test_overload_is_bounded_and_reported(self):
        self.cfg['logging']['queue_size']=2
        with tempfile.TemporaryDirectory() as tmp:
            logger=SensingLogger(tmp,self.cfg,capture_terminal=False)
            # Block checkpoint writer briefly; producers must not wait for disk.
            entered=threading.Event();release=threading.Event()
            original=logger._finish_block
            def delayed(*args,**kwargs):
                entered.set();release.wait(2);return original(*args,**kwargs)
            logger._finish_block=delayed
            logger._put('CV',{},logger.started+11)
            self.assertTrue(entered.wait(1))
            start=time.monotonic()
            for _ in range(50):logger.record_cv({'detections':[]})
            self.assertLess(time.monotonic()-start,.2)
            self.assertLessEqual(logger.events.qsize(),2)
            self.assertGreater(logger.stats['overload_dropped'],0)
            release.set();logger.close()


class BridgeTests(ConfigCase):
    def message(self,seq=1,session='one'):
        return 'NAV_STATIC '+json.dumps(dict(schema_version=3,frame='body_x_right_y_forward',snapshot=True,
            session_id=session,sequence=seq,ttl_s=.75,valid=True,source_age_s=.1,source_monotonic_s=0,static_obstacles=[]))

    def test_receiver_expiry_order_and_restart(self):
        r=SnapshotReceiver();self.assertTrue(r.accept(self.message(),0))
        self.assertTrue(r.current(.3)['valid']);self.assertFalse(r.current(.41)['valid'])
        self.assertFalse(r.accept(self.message(),.8))
        self.assertTrue(r.accept(self.message(1,'two'),1))
        self.assertFalse(r.accept(self.message(2,'one'),1.1))

    def test_same_host_receiver_rejects_queued_old_source(self):
        r=SnapshotReceiver();self.assertTrue(r.accept(self.message(),2))
        self.assertFalse(r.current(2)['valid'])
        self.assertEqual(r.current(2)['status'],'source_expired_in_transit')

    def test_receiver_rejects_bad_payload(self):
        r=SnapshotReceiver()
        for text in (self.message().replace('0.75','NaN'),self.message().replace('body_x_right_y_forward','map')):
            with self.assertRaises(ValueError):r.accept(text)

    def test_publisher_and_receiver_real_zmq(self):
        import zmq
        ctx=zmq.Context();sub=ctx.socket(zmq.SUB)
        sub.setsockopt_string(zmq.SUBSCRIBE,'NAV_STATIC ');sub.setsockopt(zmq.LINGER,0)
        cfg=dict(self.cfg['navigation'],bind='inproc://'+str(uuid.uuid4()),publish_hz=30)
        context_patch=patch('zmq.Context',return_value=types.SimpleNamespace(socket=ctx.socket,term=lambda:None));context_patch.start()
        bridge=NavBridge(cfg);sub.connect(cfg['bind']);receiver=SnapshotReceiver()
        try:
            end=time.monotonic()+2;received=False
            while time.monotonic()<end:
                bridge.publish([track()],True,0,True,time.monotonic())
                if sub.poll(30):
                    receiver.accept(sub.recv_string())
                    if receiver.current()['valid']:
                        received=True;break
            self.assertTrue(received);self.assertEqual(len(receiver.current()['static_obstacles']),1)
            # A stopped source becomes invalid even while PUB continues heartbeats.
            time.sleep(.5)
            end=time.monotonic()+1
            while time.monotonic()<end:
                if sub.poll(30):
                    receiver.accept(sub.recv_string())
                    if not receiver.current()['valid']:break
            self.assertFalse(receiver.current()['valid'])
        finally:bridge.close();context_patch.stop();sub.close();ctx.term()

    def test_occupied_port_does_not_stop_sensing_or_burst_expiry(self):
        import zmq
        ctx=zmq.Context();busy=ctx.socket(zmq.PUB);address='inproc://'+str(uuid.uuid4());busy.bind(address)
        context_patch=patch('zmq.Context',return_value=types.SimpleNamespace(socket=ctx.socket,term=lambda:None));context_patch.start()
        self.cfg['navigation'].update(enabled=True,bind=address)
        stop=threading.Event();state=Latest();ctrl=Controller(self.cfg,stop,Latest(),state,reader=DemoReader())
        ctrl.thread.start()
        try:
            deadline=time.monotonic()+2
            while time.monotonic()<deadline and not stop.is_set():time.sleep(.02)
            self.assertIsNone(ctrl.error);self.assertTrue(ctrl.thread.is_alive())
            self.assertTrue(state.get()['lidar_healthy']);self.assertEqual(state.get()['duties'],[0]*5)
        finally:stop.set();ctrl.close();context_patch.stop();busy.close();ctx.term()

    def test_slow_subscriber_cannot_block_control_handoff(self):
        import zmq
        ctx=zmq.Context();sub=ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.RCVHWM,1);sub.setsockopt_string(zmq.SUBSCRIBE,'NAV_STATIC ')
        cfg=dict(self.cfg['navigation'],bind='inproc://'+str(uuid.uuid4()),publish_hz=100)
        context_patch=patch('zmq.Context',return_value=types.SimpleNamespace(socket=ctx.socket,term=lambda:None));context_patch.start()
        bridge=NavBridge(cfg);sub.connect(cfg['bind'])
        try:
            start=time.monotonic()
            for _ in range(10000):bridge.publish([track()],True,0,True,time.monotonic())
            self.assertLess(time.monotonic()-start,1)
        finally:bridge.close();context_patch.stop();sub.close(linger=0);ctx.term()

    def test_zero_failure_not_logged_as_success(self):
        class Bad(FakeMotor):
            @property
            def value(self):return 0
            @value.setter
            def value(self,v):
                if hasattr(self,'closed'):raise RuntimeError('failed write')
        d=MotorDriver(self.cfg['haptics'],True,Bad)
        self.assertFalse(d.close())
