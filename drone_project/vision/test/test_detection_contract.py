from types import SimpleNamespace
import unittest

from drone_vision.detection_contract import valid_observation, valid_targets


class DetectionContractTests(unittest.TestCase):
    def test_provenance_age_order_and_frame(self):
        valid = dict(stamp=10, now=10.1, max_age=0.5, frame='optical', optical='optical',
                     calibrated={10}, last_stamp=9, publishers=[('/', 'apriltag')])
        self.assertTrue(valid_observation(**valid))
        for change in (dict(now=11), dict(now=9), dict(last_stamp=10),
                       dict(frame='body'), dict(calibrated=set()),
                       dict(publishers=[('/', 'apriltag'), ('/', 'other')]),
                       dict(publishers=[('/', 'other')])):
            self.assertFalse(valid_observation(**(valid | change)))

    def test_tag_allowlist_and_duplicate(self):
        tag = SimpleNamespace(id=1, family='tag36h11')
        self.assertTrue(valid_targets([tag], {1}, '36h11'))
        self.assertFalse(valid_targets([tag, tag], {1}, '36h11'))
        self.assertFalse(valid_targets([tag], {2}, '36h11'))
        self.assertFalse(valid_targets([tag], {1}, '25h9'))
