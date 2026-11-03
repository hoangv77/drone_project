"""Optional restart test using a temporary service in the current user manager."""
import configparser
import os
from pathlib import Path
import subprocess
import time
import unittest
from uuid import uuid4

CM5 = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('CM5_TEST_SYSTEMD') == '1', 'requires systemd user manager')
class SystemdTests(unittest.TestCase):
    def test_failed_process_restarts_with_unit_policy(self):
        config = configparser.ConfigParser()
        config.read(CM5 / 'systemd/drone-agent.service')
        unit = f'cm5-agent-restart-test-{uuid4().hex}.service'
        self.addCleanup(subprocess.run, ['systemctl', '--user', 'stop', unit], check=False)
        subprocess.run([
            'systemd-run', '--user', '--quiet', f'--unit={unit}',
            '--property=Type=exec', '--property=StartLimitIntervalSec=0',
            f'--property=Restart={config["Service"]["Restart"]}',
            f'--property=RestartSec={config["Service"]["RestartSec"]}',
            '/bin/false',
        ], check=True, timeout=10)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            count = subprocess.check_output(
                ['systemctl', '--user', 'show', '-p', 'NRestarts', '--value', unit], text=True)
            if int(count) > 0:
                return
            time.sleep(0.25)
        self.fail('systemd did not restart the failed process')
