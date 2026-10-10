"""Re-entry and independent encounter regression tests; synthetic geometry."""
from test_sensing import ConfigCase, track
from haptics.encounters import EncounterWarnings

class ReturnTests(ConfigCase):
    def setUp(self):
        super().setUp()
        self.cfg['haptics']['reminder_enabled']=True
        self.e=EncounterWarnings(self.cfg['haptics'])
        self.near=track(0,.3)

    def test_confirmed_departure_rearms_same_track(self):
        self.e.select([self.near],[self.near],0)
        far=track(0,2)
        self.e.select([far],[],12.4)
        self.e.select([far],[],12.99)
        a,d,_=self.e.select([self.near],[self.near],16.22)
        self.assertEqual(a[0]['haptic_phase'],'initial')
        self.assertAlmostEqual(d,17.72)
        self.assertTrue(any(e['event']=='rearmed_after_departure' for e in self.e.events))
        self.assertFalse(self.e.clear_cues)
        a,_,_=self.e.select([self.near],[self.near],17.73)
        self.assertEqual(a[0]['duty_override'],.25)

    def test_brief_clearance_does_not_rearm(self):
        self.e.select([self.near],[self.near],0)
        self.e.select([track(0,2)],[],2)
        a,_,_=self.e.select([self.near],[self.near],2.2)
        self.assertEqual(a[0]['haptic_phase'],'reminder')

    def test_dropout_without_departure_does_not_rearm(self):
        self.e.select([self.near],[self.near],0)
        self.e.select([],[],2,False)
        a,_,_=self.e.select([self.near],[self.near],10)
        self.assertEqual(a[0]['haptic_phase'],'reminder')

    def test_label_change_does_not_rearm(self):
        self.e.select([self.near],[self.near],0)
        for i,label in enumerate(('chair','bottle','person')):
            t=dict(self.near,semantic_class=label)
            a,_,_=self.e.select([t],[t],2+i)
            self.assertEqual(a[0]['haptic_phase'],'reminder')

    def test_known_overlapping_tracks_keep_independent_deadlines(self):
        a=self.near;b=dict(track(80,.3),id=2)
        self.e.select([a,b],[a],0)
        b=dict(a,id=2,semantic_class='chair')
        active,_,_=self.e.select([a,b],[a,b],2)
        self.assertEqual([t['haptic_phase'] for t in active],['reminder','initial'])
        self.assertNotEqual(self.e.track_encounters[1],self.e.track_encounters[2])
        self.assertFalse(any(e['event']=='merge_history' for e in self.e.events))
        # A third new cluster touching both cannot bridge their histories.
        c=dict(a,id=3)
        self.e.select([a,b,c],[a,b,c],2.1)
        self.assertNotEqual(self.e.track_encounters[1],self.e.track_encounters[2])

    def test_confirmed_departure_rearms_replacement_track(self):
        self.e.select([self.near],[self.near],0)
        far=track(0,2)
        self.e.select([far],[],2);self.e.select([far],[],2.6)
        replacement=dict(far,id=2)
        self.e.select([replacement],[],2.7)
        near=dict(self.near,id=2)
        active,_,_=self.e.select([near],[near],3)
        self.assertEqual(active[0]['haptic_phase'],'initial')
        self.assertTrue(any(e['event']=='rearmed_after_departure' for e in self.e.events))
