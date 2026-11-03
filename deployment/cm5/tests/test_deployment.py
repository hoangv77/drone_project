"""Portable unit checks and Linux shell transaction tests using isolated fixtures."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

CM5 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CM5))
from health import telemetry_advancing
from schema import prepare, validate_topics


class DataTests(unittest.TestCase):
    def test_missing_and_stale_telemetry(self):
        fresh = {name + '_fresh': True for name in (
            'status', 'land_detected', 'local_position', 'odometry', 'gps', 'battery', 'failsafe_flags')}
        first = SimpleNamespace(last_update_time=1, **fresh)
        last = SimpleNamespace(last_update_time=2, **fresh)
        self.assertFalse(telemetry_advancing([]))
        self.assertFalse(telemetry_advancing([first, first]))
        self.assertTrue(telemetry_advancing([first, last]))
        last.failsafe_flags_fresh = False
        self.assertFalse(telemetry_advancing([first, last]))

    def test_wrong_firmware_pin(self):
        try:
            import yaml
        except ImportError:
            self.skipTest('PyYAML required for firmware preparation')
        with tempfile.TemporaryDirectory() as work:
            with self.assertRaisesRegex(ValueError, 'full lowercase'):
                prepare(Path(work), work, 'main')

    def test_wrong_dds_topic_type(self):
        with tempfile.TemporaryDirectory() as work:
            with self.assertRaisesRegex(ValueError, 'vehicle_status'):
                validate_topics([{'topic': '/fmu/out/vehicle_status', 'type': 'wrong'}], Path(work))

    def test_wrong_topic_version(self):
        with tempfile.TemporaryDirectory() as work:
            staged = Path(work)
            (staged / 'msg').mkdir()
            (staged / 'msg/VehicleStatus.msg').write_text('uint32 MESSAGE_VERSION = 5\n')
            with self.assertRaisesRegex(ValueError, 'version 5, expected 4'):
                validate_topics([{'topic': '/fmu/out/vehicle_status',
                                  'type': 'px4_msgs::msg::VehicleStatus'}], staged)

    def test_units_retry_and_stack_starts_after_agent(self):
        for name in ('agent', 'stack'):
            unit = (CM5 / f'systemd/drone-{name}.service').read_text()
            self.assertIn('Restart=always\n', unit)
            self.assertIn('RestartSec=5s\n', unit)
            self.assertIn('User=drone-runtime\n', unit)
        self.assertIn('After=drone-agent.service\n', unit)


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux shell fixtures required')
class LinuxShellTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.work = Path(directory.name)
        self.opt = self.work / 'opt'
        self.etc = self.work / 'etc'
        self.bin = self.work / 'bin'
        for path in (self.bin, self.opt / 'state', self.opt / 'releases', self.etc):
            path.mkdir(parents=True)
        self.env = dict(os.environ, PATH=f'{self.bin}:{os.environ["PATH"]}')
        self.env['TEST_LOG'] = str(self.work / 'calls')
        for name in ('lib.sh', 'run.sh', 'update.sh'):
            fixture = (CM5 / name).read_text().replace('/opt/drone', str(self.opt))
            fixture = fixture.replace('/etc/drone', str(self.etc))
            fixture = fixture.replace('/etc/os-release', str(self.etc / 'os-release'))
            fixture = fixture.replace('/opt/ros/humble', str(self.work / 'ros'))
            (self.work / name).write_text(fixture)
        (self.etc / 'os-release').write_text('ID=ubuntu\nVERSION_ID=22.04\n')
        (self.etc / 'fastdds.xml').touch()
        self.stub('uname', 'echo aarch64')
        self.stub('dpkg', 'echo arm64')
        self.stub('sudo', 'exec "$@"')

    def stub(self, name, body):
        path = self.bin / name
        path.write_text('#!/bin/bash\n' + body + '\n')
        path.chmod(0o755)

    def shell(self, *args):
        return subprocess.run(['bash', *args], cwd=self.work, env=self.env,
                              text=True, capture_output=True, timeout=15)

    def test_unsupported_os(self):
        (self.etc / 'os-release').write_text('ID=debian\nVERSION_ID=12\n')
        result = self.shell('-c', '. ./lib.sh; require_platform')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Expected Ubuntu 22.04', result.stderr)

    def test_arm64_kernel_and_userspace_required(self):
        self.assertEqual(self.shell('-c', '. ./lib.sh; require_platform').returncode, 0)
        for command, wrong, correct in (('uname', 'x86_64', 'aarch64'), ('dpkg', 'amd64', 'arm64')):
            self.stub(command, f'echo {wrong}')
            result = self.shell('-c', '. ./lib.sh; require_platform')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Expected ARM64', result.stderr)
            self.stub(command, f'echo {correct}')

    def test_missing_dependency(self):
        result = self.shell('-c', '. ./lib.sh; need absent-drone-command')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Missing absent-drone-command', result.stderr)

    def agent(self, transport, body):
        (self.work / 'ros').mkdir()
        (self.work / 'ros/setup.bash').touch()
        (self.etc / 'drone.env').write_text(
            f'ROS_DOMAIN_ID=42\nPX4_TRANSPORT={transport}\n'
            'PX4_SERIAL_DEVICE=/dev/nonexistent-cm5-test\nPX4_AGENT_UDP_PORT=8888\n')
        path = self.opt / 'agent-v2.4.2/bin'
        path.mkdir(parents=True)
        agent = path / 'MicroXRCEAgent'
        agent.write_text('#!/bin/bash\n' + body + '\n')
        agent.chmod(0o755)
        return self.shell('run.sh', 'agent')

    def test_missing_serial_device(self):
        result = self.agent('serial', 'echo started')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Serial device missing', result.stderr)
        self.assertNotIn('started', result.stdout)

    def test_agent_failure_propagates(self):
        result = self.agent('udp', '[[ "$*" == "udp4 -p 8888" ]] && exit 7')
        self.assertEqual(result.returncode, 7, result.stderr)

    def test_install_defaults_preserve_existing_configuration(self):
        destination = self.etc / 'drone.env'
        default = self.work / 'default'
        default.write_text('default\n')
        self.assertEqual(self.shell('-c', f'. ./lib.sh; install_default default "{destination}"').returncode, 0)
        self.assertEqual(destination.read_text(), 'default\n')
        destination.write_text('operator edit\n')
        self.assertEqual(self.shell('-c', f'. ./lib.sh; install_default default "{destination}"').returncode, 0)
        self.assertEqual(destination.read_text(), 'operator edit\n')

    def test_failed_update_health_check_rolls_back_running_services(self):
        old = self.opt / 'releases/old'
        old.mkdir()
        current = self.opt / 'state/current'
        current.symlink_to(old, target_is_directory=True)
        candidate = self.opt / 'releases/candidate'
        scripts = candidate / 'source/deployment/cm5'
        scripts.mkdir(parents=True)
        (candidate / 'verified').touch()
        (candidate / 'commit').write_text('candidate\n')
        (scripts / 'verify.sh').write_text('[[ "$1" == build ]]\n')
        self.stub('systemctl', 'echo "$* $(readlink "' + str(current) + '")" >> "$TEST_LOG"\n'
                               'if [[ $1 == show ]]; then echo active; fi')
        result = self.shell('update.sh', '--activate', str(candidate))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(current.resolve(), old)
        self.assertIn('rollback completed', result.stdout)
        calls = (self.work / 'calls').read_text()
        self.assertIn(f'start drone-agent drone-stack {candidate}', calls)
        self.assertIn(f'start drone-agent drone-stack {old}', calls)


if __name__ == '__main__':
    unittest.main()
