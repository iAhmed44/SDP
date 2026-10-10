import copy
import gzip
import tempfile
from unittest.mock import patch
from test_sensing import ConfigCase, track
from cv.display_labels import numbered_detections, label_color
from haptics.encounters import EncounterWarnings
from sensing_logger import SensingLogger


class PresentationTests(ConfigCase):
    def test_numbering_is_per_class_left_to_right_without_mutation(self):
        ds=[{'name':'person','bbox':[300,0,400,200]},
            {'name':'chair','bbox':[100,0,150,200]},
            {'name':'person','bbox':[20,0,60,200]}]
        original=copy.deepcopy(ds)
        result=numbered_detections(ds)
        self.assertEqual([d['display_label'] for d in result],['person2','chair1','person1'])
        self.assertEqual(ds,original)
        self.assertEqual([d['name'] for d in result],['person','chair','person'])
        self.assertEqual(result[0]['class_color_bgr'],result[2]['class_color_bgr'])
        self.assertNotEqual(label_color('person'),label_color('chair'))

    def test_renderer_uses_requested_names_and_colors(self):
        import numpy as np
        from ui import annotate_camera
        ds=[{'name':'person','bbox':[20,40,70,100],'angle_deg':0},
            {'name':'person','bbox':[100,40,150,100],'angle_deg':10},
            {'name':'chair','bbox':[180,40,230,100],'angle_deg':20}]
        frame=np.zeros((140,260,3),dtype=np.uint8)
        with patch('ui.cv2.putText') as draw:
            annotate_camera(frame,{'detections':ds},self.cfg)
        self.assertEqual([c.args[1].split()[0] for c in draw.call_args_list],['person1','person2','chair1'])
        self.assertEqual(tuple(frame[40,20]),label_color('person'))
        self.assertEqual(tuple(frame[40,180]),label_color('chair'))

    def test_log_contains_version_and_unsampled_encounter_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger=SensingLogger(tmp,self.cfg,capture_terminal=False)
            t=logger.started
            state={'tracks':[], 'lidar_healthy':True,'vision_healthy':True}
            logger.record_cycle(state,now=t)
            state.update(encounter_events=[{'event':'first_warning','encounter_id':8}],encounter_by_track={300:8})
            logger.record_cycle(state,now=t+.01)
            logger.close()
            text=gzip.decompress(logger.path.read_bytes()).decode()
            self.assertIn('Software version: 8.0',text)
            self.assertIn('"event":"first_warning"',text)
            self.assertIn('"300":8',text)

    def test_encounter_reports_new_warning_and_rearm(self):
        e=EncounterWarnings(self.cfg['haptics']);a=track()
        e.select([a],[a],0)
        self.assertEqual([x['event'] for x in e.events],['new_encounter','first_warning','haptic_phase'])
        eid=e.track_encounters[a['id']]
        e.select([a],[a],2)
        self.assertFalse(e.events)
        self.assertEqual(e.track_encounters[a['id']],eid)
        e.select([],[],3)
        e.select([a],[a],6.1)
        self.assertIn('rearmed_after_absence',[x['event'] for x in e.events])
