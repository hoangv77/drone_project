"""Pure geometry and recording storage tests."""
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from drone_vision.calibration import calibration_error
from drone_vision.recording_store import (MIB, atomic_json, attach_ulog, hash_file,
                                           new_session, read_metadata, storage_status)


def calibration():
    header = NS(frame_id='camera_optical_frame')
    image = NS(width=640, height=480, header=header)
    info = NS(width=640, height=480, header=header, binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0),
              k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
              r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
              p=[500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.],
              d=[0.] * 5, distortion_model='plumb_bob')
    return image, info


class CalibrationTest(unittest.TestCase):
    def test_valid_monocular_calibration(self):
        image, info = calibration()
        self.assertEqual(calibration_error(image, info, 'camera_optical_frame'), '')

    def test_invalid_resolution_intrinsics_and_frame(self):
        for key, value in [('width', 320), ('k', [0.] * 9), ('d', [float('nan')] * 5),
                           ('header', NS(frame_id='wrong_frame')), ('r', [0.] * 9)]:
            with self.subTest(key=key):
                image, info = calibration()
                setattr(info, key, value)
                self.assertTrue(calibration_error(image, info, 'camera_optical_frame'))


class StoreTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.session = new_session(self.root)
        self.config = dict(session_mib=1, total_mib=2, free_mib=1)

    def test_limits(self):
        (self.session / 'large').write_bytes(bytes(2 * MIB))
        with patch('drone_vision.recording_store.shutil.disk_usage', return_value=NS(free=0)):
            state = storage_status(self.root, self.session, self.config)
        self.assertEqual(set(state['limits_reached']), {'session', 'total', 'free'})

    def test_ulog_integrity_and_no_overwrite(self):
        atomic_json(self.session / 'metadata.json', {'session': self.session.name})
        original = self.root / 'flight.ulg'
        with self.assertRaises(ValueError):
            attach_ulog(self.session, original)
        original.write_bytes(b'original test log')
        copy = attach_ulog(self.session, original)
        self.assertEqual(hash_file(original), hash_file(copy))
        self.assertEqual(read_metadata(self.session)['ulogs'][0]['sha256'], hash_file(copy))
        with self.assertRaises(FileExistsError):
            attach_ulog(self.session, original)

    def test_metadata_atomic_replacement(self):
        path = self.session / 'metadata.json'
        atomic_json(path, {'state': 'running', 'end_utc': None})
        atomic_json(path, {'state': 'finished', 'end_utc': '2026-11-02T16:00:00+00:00'})
        self.assertEqual(read_metadata(self.session)['state'], 'finished')
        self.assertFalse(path.with_suffix('.tmp').exists())
