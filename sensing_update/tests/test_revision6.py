from test_sensing import ConfigCase, track
from haptics.encounters import EncounterWarnings
from core import motor_duties

class ReminderTests(ConfigCase):
    def setUp(self):
        super().setUp()
        self.cfg['haptics'].update(reminder_enabled=True,departure_pulse_s=.15,departure_gap_s=.15,departure_duty=.5)
        self.e=EncounterWarnings(self.cfg['haptics'])

    def select(self,t,now,threat=True,healthy=True):
        return self.e.select([t] if t else [],[t] if t and threat else [],now,healthy,1)

    def test_initial_then_exact_reminder_without_new_full_burst(self):
        t=track(0,.3)
        a,_,_=self.select(t,0)
        self.assertEqual(motor_duties(a,self.cfg['haptics'])[2],1)
        for now in [1.5,2,10]:
            a,_,suppressed=self.select(t,now)
            self.assertEqual(a[0]['haptic_phase'],'reminder')
            self.assertEqual(motor_duties(a,self.cfg['haptics']),[0,0,.25,0,0])
            self.assertEqual(suppressed,[1])
            self.assertFalse(any(e['event']=='first_warning' for e in self.e.events))

    def test_two_departure_pulses_and_gap(self):
        t=track(0,.3);self.select(t,0)
        # Retain the same LiDAR track and observe it beyond warning distance.
        far=track(0,2);self.select(far,2,False)
        for now,expected in [(2.51,.5),(2.60,.5),(2.70,0),(2.82,.5),(2.95,.5),(2.97,0),(4,0)]:
            a,_,_=self.select(far,now,False)
            self.assertAlmostEqual(motor_duties(a,self.cfg['haptics'])[2],expected)
        self.assertTrue(self.e.encounters[1]['clear_notified'])

    def test_lost_cv_match_near_obstacle_does_not_signal_departure(self):
        t=track(0,.8);self.select(t,0)
        for now in [2,3,4]:
            a,_,_=self.select(t,now,False)
            self.assertFalse(a)
            self.assertFalse(any(e['event']=='departure_confirmed' for e in self.e.events))

    def test_sensor_loss_stops_output_and_cancels_departure(self):
        t=track(0,.3);self.select(t,0)
        far=track(0,2);self.select(far,2,False);self.select(far,2.6,False)
        self.assertFalse(self.select(None,2.65,False,False)[0])
        self.assertFalse(self.e.clear_cues)
        a,_,_=self.select(t,2.8)
        self.assertEqual(a[0]['haptic_phase'],'initial')

    def test_other_bearing_does_not_cancel_clear_pattern(self):
        a=track(0,.3);self.select(a,0)
        far=track(0,2);self.select(far,2,False)
        b=track(60,.4);b['id']=2
        active,_,_=self.e.select([far,b],[b],2.6,True,1)
        self.assertEqual([t['haptic_phase'] for t in active],['initial','departure'])
        self.assertTrue(self.e.clear_cues)

    def test_absence_rearm_after_healthy_three_seconds(self):
        t=track(0,.3);self.select(t,0)
        self.select(None,2,False)
        self.assertFalse(self.select(None,4.9,False)[0])
        active,_,_=self.select(None,5.1,False)
        self.assertEqual(active[0]['haptic_phase'],'departure')

    def test_direction_interpolation_never_exceeds_reminder_duty(self):
        for angle in [-90,-67,-45,-22,0,22,45,67,90]:
            a,_,_=self.select(track(angle,.3),100+angle)
            for t in a:t['duty_override']=.25
            levels=motor_duties(a,self.cfg['haptics'])
            self.assertEqual(max(levels),.25)

    def test_id_and_surface_change_do_not_restart_initial(self):
        t=track(0,.3);self.select(t,0)
        t['id']=99
        a,_,_=self.select(t,2)
        self.assertEqual(a[0]['haptic_phase'],'reminder')

    def test_reminder_watchdog_still_clears_stalled_updates(self):
        import time
        from haptics.motor_driver import MotorDriver
        h=dict(self.cfg['haptics'],heartbeat_timeout_s=.03)
        driver=MotorDriver(h,False)
        try:
            a,_,_=self.select(track(0,.3),0)
            a,_,_=self.select(track(0,.3),2)
            driver.apply_feedback(a)
            self.assertEqual(driver.duties[2],.25)
            time.sleep(.09)
            self.assertEqual(driver.duties,[0]*5)
        finally:driver.close()

    def test_absence_clock_pauses_during_sensor_failure(self):
        t=track(0,.3);self.select(t,0)
        self.select(None,1,False)
        self.select(None,2,False,False)
        self.select(None,20,False,False)
        self.assertFalse(self.select(None,20.1,False,True)[0])
        self.assertFalse(self.e.clear_cues)
