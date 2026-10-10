from test_sensing import ConfigCase,track
from haptics.encounters import EncounterWarnings
from core import motor_duties

class DepartureTests(ConfigCase):
    def setUp(self):
        super().setUp();self.cfg['haptics']['reminder_enabled']=True
        self.e=EncounterWarnings(self.cfg['haptics'])

    def prepare(self):
        t=track(0,.3);self.e.select([t],[t],0)
        far=track(0,2);self.e.select([far],[],2)
        return far

    def test_long_double_pulse(self):
        far=self.prepare()
        for now,value in [(2.51,1),(2.79,1),(2.82,0),(3.05,0),(3.07,1),(3.35,1),(3.37,0)]:
            active,_,_=self.e.select([far],[],now)
            self.assertEqual(motor_duties(active,self.cfg['haptics'])[2],value)

    def test_same_motor_defers_until_free(self):
        far=self.prepare();b=track(0,.4);b['id']=2
        # Different existing encounter ID/surface at an isolated bearing first.
        self.e.select([far,dict(b,angle_deg=80,x_m=.4,y_m=.1)],[],2.1)
        active,_,_=self.e.select([far,b],[b],2.6)
        self.assertFalse(any(t.get('haptic_phase')=='departure' for t in active))
        self.assertTrue(self.e.clear_cues)
        active,_,_=self.e.select([far],[],2.7)
        self.assertTrue(any(t.get('haptic_phase')=='departure' for t in active))

    def test_other_motor_keeps_warning_while_departure_plays(self):
        far=self.prepare();b=track(90,.3);b['id']=2
        active,_,_=self.e.select([far,b],[b],2.6)
        levels=motor_duties(active,self.cfg['haptics'])
        self.assertEqual(levels[2],1);self.assertEqual(levels[4],1)

    def test_started_cue_stops_when_motor_needed_by_hazard(self):
        far=self.prepare();self.e.select([far],[],2.6)
        b=track(0,.3);b['id']=2
        active,_,_=self.e.select([far,b],[b],2.7)
        self.assertFalse(any(t.get('haptic_phase')=='departure' for t in active))
        self.assertFalse(self.e.clear_cues)

    def test_skipped_second_pulse_is_logged_as_incomplete(self):
        far=self.prepare();self.e.select([far],[],2.6)
        self.e.select([far],[],4)
        self.assertTrue(any(e['event']=='departure_cue_incomplete' for e in self.e.events))
