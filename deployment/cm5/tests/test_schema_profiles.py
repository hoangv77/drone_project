"""Firmware-derived contract tests. Set source paths to exercise real Git inputs.

No legacy .msg fixture is invented here. Installation copies below emulate
the schema-file portion of an overlay; ROS compilation is tested separately.
"""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

CM5 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CM5))
import schema
from health import telemetry_advancing
from types import SimpleNamespace


class ProfileTests(unittest.TestCase):
    def test_unknown_profile_and_legacy_pin_fail_closed(self):
        for profile, revision in [('auto', schema.LEGACY_COMMIT),
                                  ('legacy_1_14_passive', '0' * 40)]:
            with self.assertRaises(ValueError):
                schema.check_profile(profile, revision)

    def test_cli_requires_explicit_profile(self):
        result = subprocess.run([sys.executable, str(CM5 / 'schema.py'), 'verify', '.', '.'],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--profile', result.stderr)

    def test_passive_health_never_requires_unavailable_streams(self):
        flags = {name + '_fresh': True for name in ('status', 'local_position', 'odometry', 'gps', 'failsafe_flags')}
        flags.update(battery_fresh=False, land_detected_fresh=False)
        samples = [SimpleNamespace(last_update_time=i, **flags) for i in (1, 2)]
        self.assertTrue(telemetry_advancing(samples, 'legacy_1_14_passive'))
        self.assertFalse(telemetry_advancing(samples, 'modern_px4'))
        samples[-1].status_fresh = False
        self.assertFalse(telemetry_advancing(samples, 'legacy_1_14_passive'))


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = os.environ.get('PX4_LEGACY_SOURCE')
        if not source:
            raise unittest.SkipTest('PX4_LEGACY_SOURCE required: exact vendor Git source, no fabricated schemas')
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.base = Path(cls.directory.name)
        cls.legacy = cls.base / 'legacy'
        cls.legacy.mkdir()
        schema.prepare(cls.legacy, source, schema.LEGACY_COMMIT, 'legacy_1_14_passive')
        cls.install(cls.legacy, 'legacy_1_14_passive')

    @staticmethod
    def install(root, profile):
        target = root / 'install/share/px4_msgs'
        shutil.copytree(root / 'px4_msgs', target)
        marker = root / 'install/share/px4_interface_node/compatibility_profile'
        marker.parent.mkdir(parents=True)
        marker.write_text(profile + '\n')

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / 'release'
        shutil.copytree(self.legacy, self.root)
        self.staged = self.root / 'px4_msgs'

    def publications(self):
        import yaml
        return yaml.safe_load((self.root / 'firmware' / schema.DDS_TOPICS).read_text())['publications']

    def verify(self, profile='legacy_1_14_passive', revision=schema.LEGACY_COMMIT):
        schema.verify(self.root, self.root / 'install', profile, revision)

    @contextmanager
    def changed(self, path, contents):
        previous = path.read_bytes()
        path.write_text(contents)
        try:
            yield
        finally:
            path.write_bytes(previous)

    def test_exact_legacy_contract_and_unversioned_schemas(self):
        self.verify()
        manifest = json.loads((self.root / 'schema.json').read_text())
        self.assertEqual(len(manifest['schemas']), 184)
        self.assertEqual(len(manifest['required_publications']), 5)
        self.assertEqual(manifest['unavailable_publications'], schema.UNAVAILABLE)
        self.assertTrue(all(v is None for v in manifest['telemetry_message_versions'].values()))
        self.assertIn('PositionSetpoint', manifest['message_dependencies']['PositionSetpointTriplet'])

    def test_wrong_commit_rejected_before_preparation(self):
        with self.assertRaisesRegex(ValueError, 'exact firmware commit'):
            schema.prepare(self.root / 'wrong', '.', '0' * 40, 'legacy_1_14_passive')

    def test_missing_required_publications(self):
        for topic in schema.required_publications('legacy_1_14_passive', self.staged):
            with self.subTest(topic=topic), self.assertRaisesRegex(ValueError, topic):
                schema.validate_topics([p for p in self.publications() if p['topic'] != topic],
                                       self.staged, 'legacy_1_14_passive')

    def test_wrong_dds_types(self):
        for topic in schema.required_publications('legacy_1_14_passive', self.staged):
            publications = self.publications()
            next(p for p in publications if p['topic'] == topic)['type'] = 'px4_msgs::msg::Wrong'
            with self.subTest(topic=topic), self.assertRaisesRegex(ValueError, topic):
                schema.validate_topics(publications, self.staged, 'legacy_1_14_passive')

    def test_duplicates_and_unexpected_optional_publications(self):
        publications = self.publications()
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            schema.validate_topics(publications + [publications[0]], self.staged, 'legacy_1_14_passive')
        for topic, kind in schema.UNAVAILABLE.items():
            with self.subTest(topic=topic), self.assertRaisesRegex(ValueError, 'Unexpected legacy'):
                schema.validate_topics(publications + [dict(topic=topic, type=kind)], self.staged,
                                       'legacy_1_14_passive')

    def test_legacy_does_not_weaken_modern_versions(self):
        with self.assertRaisesRegex(ValueError, 'version 0, expected 4'):
            schema.validate_topics(self.publications(), self.staged, 'modern_px4')

    def test_modern_version_declaration_keeps_its_type_contract(self):
        path = self.staged / 'msg/VehicleStatus.msg'
        with self.changed(path, path.read_text() + '\nuint8 MESSAGE_VERSION = 4\n'):
            with self.assertRaisesRegex(ValueError, 'MESSAGE_VERSION must be uint32'):
                schema.validate_topics(self.publications(), self.staged, 'modern_px4')

    def test_profile_firmware_and_compiled_profile_mismatches(self):
        with self.assertRaisesRegex(ValueError, 'profile differs'):
            self.verify('modern_px4')
        with self.assertRaisesRegex(ValueError, 'Firmware commit differs'):
            self.verify(revision='0' * 40)
        marker = self.root / 'install/share/px4_interface_node/compatibility_profile'
        marker.write_text('modern_px4\n')
        with self.assertRaisesRegex(ValueError, 'Compiled interface'):
            self.verify()

    def test_installed_staged_and_archived_schema_changes_fail(self):
        for folder in ['install/share/px4_msgs', 'px4_msgs', 'firmware']:
            path = self.root / folder / 'msg/VehicleStatus.msg'
            with self.subTest(folder=folder), self.changed(path, path.read_text() + '\nbool surprise\n'):
                with self.assertRaises(ValueError):
                    self.verify()

    def test_dds_config_changes_fail(self):
        path = self.root / 'firmware' / schema.DDS_TOPICS
        path.write_text(path.read_text() + '\n# unexpected change\n')
        with self.assertRaisesRegex(ValueError, 'exact legacy firmware contract'):
            self.verify()

    def test_missing_versions_are_not_proof_of_compatibility(self):
        for name, old, new in [
            ('VehicleStatus', 'uint8 nav_state_user_intention', 'uint16 nav_state_user_intention'),
            ('SensorGps', 'int32 lat', 'float64 lat'),
            ('FailsafeFlags', 'primary_geofence_breached', 'geofence_breached'),
            ('VehicleCommandAck', 'bool from_external', 'bool unrelated'),
        ]:
            path = self.staged / 'msg' / (name + '.msg')
            with self.subTest(name=name), self.changed(path, path.read_text().replace(old, new)):
                with self.assertRaises(ValueError):
                    schema.validate_fields(self.staged, 'legacy_1_14_passive')

    def test_referenced_constants_and_nested_dependencies(self):
        path = self.staged / 'msg/VehicleStatus.msg'
        with self.changed(path, path.read_text().replace('ARMING_STATE_ARMED = 2', 'ARMING_STATE_ARMED = 3')):
            with self.assertRaisesRegex(ValueError, 'ARMING_STATE_ARMED'):
                schema.validate_fields(self.staged, 'legacy_1_14_passive')
        (self.staged / 'msg/EscReport.msg').unlink()
        with self.assertRaisesRegex(ValueError, 'missing message dependency EscReport'):
            schema.validate_fields(self.staged, 'legacy_1_14_passive')

    def test_rewritten_manifest_cannot_redefine_pinned_legacy_inputs(self):
        for folder in ['firmware', 'px4_msgs', 'install/share/px4_msgs']:
            path = self.root / folder / 'msg/VehicleStatus.msg'
            path.write_text(path.read_text() + '\nbool unexpected\n')
        path = self.root / 'schema.json'
        manifest = json.loads(path.read_text())
        manifest['schemas'] = schema.schema_hashes(self.staged)
        manifest['firmware_inputs'] = schema.firmware_inputs(self.root / 'firmware')
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'exact legacy firmware contract'):
            self.verify()


class ModernFirmwareTests(unittest.TestCase):
    def test_real_modern_preparation_and_verification(self):
        source = os.environ.get('PX4_MODERN_SOURCE')
        revision = os.environ.get('PX4_MODERN_COMMIT')
        if not source or not revision:
            self.skipTest('PX4_MODERN_SOURCE and PX4_MODERN_COMMIT required')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            schema.prepare(root, source, revision, 'modern_px4')
            FirmwareTests.install(root, 'modern_px4')
            schema.verify(root, root / 'install', 'modern_px4', revision)
            self.assertEqual(json.loads((root / 'schema.json').read_text())[
                'telemetry_message_versions']['VehicleStatus'], 4)
            # Modern source consistency still rejects modified archives.
            path = next((root / 'firmware/msg').rglob('VehicleOdometry.msg'))
            path.write_text(path.read_text() + '\n# modified\n')
            with self.assertRaises(ValueError):
                schema.verify(root, root / 'install', 'modern_px4', revision)


if __name__ == '__main__':
    unittest.main()
