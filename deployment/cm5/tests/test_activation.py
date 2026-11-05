import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from activation import Activation


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux deployment filesystem')
class ActivationTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.transaction = Activation(root)
        self.transaction.state.mkdir()
        (self.transaction.state / 'update.lock').touch()
        self.old, self.new = (str(root / 'releases' / name) for name in ('old', 'new'))
        for value in (self.old, self.new):
            path = Path(value)
            path.mkdir(parents=True)
            (path / 'verified').touch()

    def activate_old(self):
        t = self.transaction
        t.run('prepare', self.old)
        t.run('switch', None)
        t.run('commit', None)

    def test_success_and_initial_rollback(self):
        t = self.transaction
        t.run('prepare', self.new)
        t.run('switch', None)
        with self.assertRaisesRegex(ValueError, 'No runtime-verified'):
            t.boot()
        self.assertFalse(t.current.is_symlink())
        self.activate_old()
        t.boot()
        self.assertEqual(t.read()['phase'], 'verified')

    def test_reconcile_interrupted_phases(self):
        for phase in ('prepared', 'switched_before_verifying', 'verifying', 'rollback_required'):
            with self.subTest(phase=phase):
                self.activate_old()
                t = self.transaction
                t.run('prepare', self.new)
                if phase == 'switched_before_verifying':
                    t.point(self.new)
                if phase in ('verifying', 'rollback_required'):
                    t.run('switch', None)
                if phase == 'rollback_required':
                    t.write(t.read(), 'rollback_required')
                t.boot()
                self.assertEqual(t.read()['phase'], 'rolled_back')
                self.assertEqual(os.readlink(t.current), self.old)

    def test_corrupt_or_missing_previous_release_denies_boot(self):
        self.activate_old()
        t = self.transaction
        t.run('prepare', self.new)
        t.run('switch', None)
        (Path(self.old) / 'verified').unlink()
        with self.assertRaisesRegex(ValueError, 'unverified'):
            t.boot()
        self.assertEqual(t.read()['phase'], 'rollback_required')
        t.journal.write_text('{}')
        with self.assertRaisesRegex(ValueError, 'Invalid activation'):
            t.boot()

    def test_insufficient_storage_preserves_committed_state(self):
        self.activate_old()
        t = self.transaction
        with patch('activation.os.fsync', side_effect=OSError(28, 'No space left')):
            with self.assertRaises(OSError):
                t.run('prepare', self.new)
        self.assertEqual(t.read()['phase'], 'verified')
        self.assertEqual(os.readlink(t.current), self.old)

    def test_killed_updater_releases_lock_and_boot_rolls_back(self):
        self.activate_old()
        t = self.transaction
        ready = t.state / 'candidate-selected'
        helper = Path(__file__).resolve().parents[1]
        script = """
import fcntl, sys, time
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from activation import Activation
t = Activation(Path(sys.argv[2]))
with (t.state / 'update.lock').open('rb') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    t.run('prepare', sys.argv[3])
    t.run('switch', None)
    (t.state / 'candidate-selected').touch()
    time.sleep(30)
"""
        with subprocess.Popen([sys.executable, '-c', script, helper, t.root, self.new]) as updater:
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and updater.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(ready.exists())
                t.boot()
                self.assertEqual(os.readlink(t.current), self.new)
            finally:
                updater.kill()
                updater.wait(timeout=5)
        t.boot()
        self.assertEqual(os.readlink(t.current), self.old)
        self.assertEqual(t.read()['phase'], 'rolled_back')
