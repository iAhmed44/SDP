import json
from pathlib import Path
from test_sensing import ConfigCase, track
from haptics.encounters import EncounterWarnings

class IdentityTests(ConfigCase):
    def test_same_track_cannot_be_stolen_by_spatial_competitor(self):
        e=EncounterWarnings(self.cfg['haptics'])
        a=track(0,.4); a['id']=55
        e.select([a],[a],0)
        b=track(0,.4); b['id']=99
        a=track(25,1.4); a['id']=55
        active,_,suppressed=e.select([b,a],[a,b],2)
        self.assertNotIn(55,[t['id'] for t in active])
        self.assertIn(55,suppressed)

    def test_recorded_track55_observations_keep_deadline(self):
        rows=json.loads(Path(__file__).with_name('recorded_track55.json').read_text())
        e=EncounterWarnings(self.cfg['haptics']);deadlines=set();count=0
        for row in rows:
            if not 17<=row['time']<=24:continue
            tracks=row['tracks']
            threats=[t for t in tracks if t['observed'] and t['id']==55]
            count+=bool(threats)
            active,_,_=e.select(tracks,threats,row['time'],row['healthy'],1)
            deadlines.update(t['burst_until'] for t in active if t['id']==55)
        self.assertGreater(count,5)
        self.assertEqual(len(deadlines),1)

    def test_healthy_absence_rearms(self):
        e=EncounterWarnings(self.cfg['haptics']);a=track()
        e.select([a],[a],0)
        e.select([],[],2)
        self.assertTrue(e.select([a],[a],5.1)[0])

    def test_generation_change_does_not_bind_distant_reused_id(self):
        e=EncounterWarnings(self.cfg['haptics']);a=track(-60,.4)
        e.select([a],[a],0,True,1)
        b=track(60,.4)
        self.assertTrue(e.select([b],[b],2,True,2)[0])

    def test_control_metrics_cover_every_cycle(self):
        import tempfile,gzip
        from sensing_logger import SensingLogger
        with tempfile.TemporaryDirectory() as tmp:
            logger=SensingLogger(tmp,self.cfg,capture_terminal=False)
            t=logger.started
            state={'tracks':[], 'lidar_healthy':True,'vision_healthy':True}
            logger.record_cycle(state,now=t)
            for dt in [.05,.1,.15,.6]:logger.record_cycle(state,now=t+dt)
            logger.close()
            text=gzip.decompress(logger.path.read_bytes()).decode()
            self.assertIn('"control_cycles_since_record":4',text)
            self.assertIn('"max_control_gap_ms":',text)
