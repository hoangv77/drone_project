"""Validate explicit PX4 contracts and the firmware inputs used by the compiler."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

DDS_TOPICS = 'src/modules/uxrce_dds_client/dds_topics.yaml'
LEGACY_COMMIT = '4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c'
PROFILES = ('modern_px4', 'legacy_1_14_passive')
UNAVAILABLE = {
    '/fmu/out/battery_status': 'px4_msgs::msg::BatteryStatus',
    '/fmu/out/vehicle_land_detected': 'px4_msgs::msg::VehicleLandDetected',
    '/fmu/out/vehicle_command_ack': 'px4_msgs::msg::VehicleCommandAck',
}
PRIMITIVES = set('bool byte char float32 float64 int8 uint8 int16 uint16 int32 uint32 int64 uint64 string'.split())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_profile(profile, revision=None):
    if profile not in PROFILES:
        raise ValueError(f'Unknown compatibility profile: {profile}')
    if revision is not None:
        if not re.fullmatch(r'[0-9a-f]{40}', revision):
            raise ValueError('PX4 revision must be the full lowercase firmware commit SHA')
        if profile == 'legacy_1_14_passive' and revision != LEGACY_COMMIT:
            raise ValueError(f'legacy_1_14_passive requires exact firmware commit {LEGACY_COMMIT}')


def required_publications(profile, staged):
    check_profile(profile)
    expected = {
        'vehicle_status': ('VehicleStatus', 4),
        'vehicle_local_position': ('VehicleLocalPosition', 1),
        'vehicle_odometry': ('VehicleOdometry', 0),
        'failsafe_flags': ('FailsafeFlags', 0),
    }
    if profile == 'legacy_1_14_passive':
        expected = {topic: (name, 0) for topic, (name, _) in expected.items()}
        expected['vehicle_gps_position'] = ('SensorGps', 0)
    else:
        expected.update(vehicle_land_detected=('VehicleLandDetected', 0),
                        battery_status=('BatteryStatus', 1))
        gnss = ('vehicle_gps_position', 'SensorGps') if (staged / 'msg/SensorGps.msg').exists() else (
            'vehicle_gnss', 'VehicleGnss')
        expected[gnss[0]] = (gnss[1], 0)
    return {f'/fmu/out/{topic}': (name, version) for topic, (name, version) in expected.items()}


def publication_map(publications):
    if not isinstance(publications, list):
        raise ValueError('DDS publications must be a list')
    topics = {}
    for entry in publications:
        if not isinstance(entry, dict) or not isinstance(entry.get('topic'), str) or not isinstance(entry.get('type'), str):
            raise ValueError('Invalid DDS publication entry')
        if entry['topic'] in topics:
            raise ValueError(f'Duplicate DDS publication {entry["topic"]}')
        topics[entry['topic']] = entry['type']
    return topics


def declarations(path):
    """Parse fields/constants only; never generate or rewrite message definitions."""
    fields, constants = {}, {}
    for raw in path.read_text().splitlines():
        line = raw.split('#', 1)[0].strip()
        match = re.fullmatch(r'(\w+(?:\[\d*\])?)\s+(\w+)\s*(?:=\s*(.+))?', line)
        if match:
            kind, name, value = match.groups()
            if value is None:
                fields[name] = kind
            else:
                constants[name] = (kind, value.strip())
    return fields, constants


def validate_topics(publications, staged, profile='modern_px4'):
    topics = publication_map(publications)
    versions = {}
    for topic, (name, required_version) in required_publications(profile, staged).items():
        if topics.get(topic) != f'px4_msgs::msg::{name}':
            raise ValueError(f'Missing or incompatible DDS publication {topic} ({name})')
        _, constants = declarations(staged / 'msg' / f'{name}.msg')
        if 'MESSAGE_VERSION' in constants and constants['MESSAGE_VERSION'][0] != 'uint32':
            raise ValueError(f'{name} MESSAGE_VERSION must be uint32')
        version = int(constants['MESSAGE_VERSION'][1]) if 'MESSAGE_VERSION' in constants else 0
        if version != required_version:
            raise ValueError(f'{name} DDS schema version {version}, expected {required_version}')
        # Keep absence distinct from an explicit v0 declaration in the manifest.
        versions[name] = version if 'MESSAGE_VERSION' in constants else None
    if profile == 'legacy_1_14_passive':
        for topic, kind in topics.items():
            if topic in UNAVAILABLE or kind in UNAVAILABLE.values():
                raise ValueError(f'Unexpected legacy publication {topic}; passive contract changed')
    return versions


def validate_fields(staged, profile):
    """Fields and constants referenced by the interface, including unpublished types."""
    legacy = profile == 'legacy_1_14_passive'
    required = {
        'VehicleStatus': dict(timestamp='uint64', arming_state='uint8', nav_state='uint8',
            nav_state_user_intention='uint8', failsafe='bool', pre_flight_checks_pass='bool',
            gcs_connection_lost='bool', safety_off='bool', calibration_enabled='bool',
            system_id='uint8', component_id='uint8'),
        'VehicleLocalPosition': dict(timestamp='uint64', xy_valid='bool', z_valid='bool',
            v_xy_valid='bool', v_z_valid='bool', **{n: 'float32' for n in ('x','y','z','vx','vy','vz')}),
        'VehicleOdometry': {'timestamp': 'uint64'},
        'BatteryStatus': dict(timestamp='uint64', connected='bool', remaining='float32',
            warning='uint8', faults='uint16', is_powering_off='bool'),
        'VehicleLandDetected': dict(timestamp='uint64', landed='bool'),
        'VehicleCommandAck': dict(timestamp='uint64', command='uint32', result='uint8',
            result_param1='uint8', result_param2='int32', target_system='uint8',
            target_component='uint8' if legacy else 'uint16', from_external='bool'),
        'VehicleCommand': dict(timestamp='uint64', command='uint32', param1='float32',
            param2='float32', param3='float32', target_system='uint8',
            target_component='uint8', source_system='uint8',
            source_component='uint8' if legacy else 'uint16', confirmation='uint8', from_external='bool'),
        'FailsafeFlags': dict(timestamp='uint64', battery_warning='uint8', **{n: 'bool' for n in (
            'gcs_connection_lost', 'manual_control_signal_lost', 'offboard_control_signal_lost',
            'local_position_invalid', 'local_velocity_invalid', 'global_position_invalid',
            'home_position_invalid', 'battery_unhealthy', 'battery_low_remaining_time',
            'angular_velocity_invalid', 'attitude_invalid', 'fd_critical_failure',
            'fd_esc_arming_failure', 'fd_motor_failure', 'mission_failure',
            'primary_geofence_breached' if legacy else 'geofence_breached')}),
    }
    constants = {
        'VehicleStatus': {'ARMING_STATE_ARMED': 2, 'NAVIGATION_STATE_MANUAL': 0,
            'NAVIGATION_STATE_ALTCTL': 1, 'NAVIGATION_STATE_POSCTL': 2,
            'NAVIGATION_STATE_AUTO_MISSION': 3, 'NAVIGATION_STATE_AUTO_LOITER': 4,
            'NAVIGATION_STATE_AUTO_RTL': 5, 'NAVIGATION_STATE_OFFBOARD': 14,
            'NAVIGATION_STATE_STAB': 15, 'NAVIGATION_STATE_AUTO_TAKEOFF': 17,
            'NAVIGATION_STATE_AUTO_LAND': 18, 'NAVIGATION_STATE_AUTO_PRECLAND': 20},
        'VehicleCommand': {'VEHICLE_CMD_DO_SET_MODE': 176,
            'VEHICLE_CMD_COMPONENT_ARM_DISARM': 400, 'VEHICLE_CMD_RUN_PREARM_CHECKS': 401},
        'VehicleCommandAck': {'VEHICLE_CMD_RESULT_ACCEPTED': 0, 'VEHICLE_CMD_RESULT_IN_PROGRESS': 5},
    }
    if (staged / 'msg/SensorGps.msg').exists():
        required['SensorGps'] = dict(timestamp='uint64', fix_type='uint8', **(
            dict(lat='int32', lon='int32') if legacy else dict(latitude_deg='float64', longitude_deg='float64')))
        if not legacy:
            constants['SensorGps'] = {'FIX_TYPE_3D': 3}
    else:
        required['VehicleGnss'] = dict(timestamp='uint64', usable='bool', receiver='SensorGnss')
        required['SensorGnss'] = dict(fix_type='uint8', latitude='float64', longitude='float64')
        constants['SensorGnss'] = {'FIX_TYPE_3D': 3}
    for name, expected in required.items():
        fields, values = declarations(staged / 'msg' / f'{name}.msg')
        for field, kind in expected.items():
            if fields.get(field) != kind:
                raise ValueError(f'{name}.{field}: expected field type {kind}, got {fields.get(field)}')
        for key, value in constants.get(name, {}).items():
            if key not in values or int(values[key][1]) != value:
                raise ValueError(f'{name}.{key}: expected constant {value}')
    # Every staged schema is compiled by rosidl; check all nested message dependencies.
    dependencies = {}
    paths = sorted((staged / 'msg').glob('*.msg')) + sorted((staged / 'srv').glob('*.srv'))
    for path in paths:
        fields, _ = declarations(path)
        nested = sorted({kind.split('[', 1)[0] for kind in fields.values()} - PRIMITIVES)
        for name in nested:
            if not (staged / 'msg' / f'{name}.msg').is_file():
                raise ValueError(f'{path.name}: missing message dependency {name}')
        if nested:
            key = path.stem if path.suffix == '.msg' else path.relative_to(staged).as_posix()
            dependencies[key] = nested
    return dependencies


def schema_hashes(directory):
    return {path.relative_to(directory).as_posix(): digest(path)
            for pattern in ('msg/*.msg', 'srv/*.srv') for path in sorted(directory.glob(pattern))}


def firmware_inputs(firmware):
    return {path.relative_to(firmware).as_posix(): digest(path)
            for pattern in ('msg/**/*.msg', 'srv/**/*.srv', DDS_TOPICS)
            for path in sorted(firmware.glob(pattern))}


def check_firmware(firmware, staged, profile, revision):
    check_profile(profile, revision)
    inputs = firmware_inputs(firmware)
    if profile == 'legacy_1_14_passive':
        trusted = json.loads(Path(__file__).with_name('legacy_contract.json').read_text())
        if revision != trusted['firmware_commit'] or inputs != trusted['firmware_inputs']:
            raise ValueError('Firmware inputs differ from the exact legacy firmware contract')
    expected = {}
    for folder in (firmware / 'msg', firmware / 'msg/versioned'):
        expected.update({f'msg/{path.name}': digest(path) for path in folder.glob('*.msg')})
    expected.update({f'srv/{path.name}': digest(path) for path in (firmware / 'srv').glob('*.srv')})
    if not expected or schema_hashes(staged) != expected:
        raise ValueError('Compilation schemas differ from the selected firmware source')
    return inputs


def prepare(root, checkout, revision, profile='modern_px4'):
    import yaml

    check_profile(profile, revision)
    if subprocess.check_output(
        ['git', '-C', checkout, 'rev-parse', f'{revision}^{{commit}}'], text=True
    ).strip() != revision:
        raise ValueError('The PX4 revision must identify a commit')
    firmware = root / 'firmware'
    firmware.mkdir()
    paths = ['msg', DDS_TOPICS]
    for optional in ('srv', 'LICENSE'):
        if subprocess.run(['git', '-C', checkout, 'cat-file', '-e', f'{revision}:{optional}'],
                          stderr=subprocess.DEVNULL).returncode == 0:
            paths.append(optional)
    archive = subprocess.check_output(['git', '-C', checkout, 'archive', revision, '--', *paths])
    subprocess.run(['tar', '-xf', '-', '-C', firmware], input=archive, check=True)
    here = Path(__file__).resolve().parent
    tool = here.parents[1] / 'drone_project/simulation/tools/prepare_px4_msgs.py'
    staged = root / 'px4_msgs'
    subprocess.run([sys.executable, tool, '--px4', firmware, '--template', here / 'px4_msgs',
                    '--output', staged], check=True)
    if (firmware / 'LICENSE').exists():
        (staged / 'LICENSE').write_bytes((firmware / 'LICENSE').read_bytes())
    inputs = check_firmware(firmware, staged, profile, revision)
    dds = firmware / DDS_TOPICS
    versions = validate_topics(yaml.safe_load(dds.read_text())['publications'], staged, profile)
    dependencies = validate_fields(staged, profile)
    manifest = dict(manifest_version=2, compatibility_profile=profile, firmware_commit=revision,
        dds_topics_sha256=digest(dds), telemetry_message_versions=versions,
        schemas=schema_hashes(staged), firmware_inputs=inputs, message_dependencies=dependencies,
        required_publications={topic: f'px4_msgs::msg::{name}' for topic, (name, _) in
                               required_publications(profile, staged).items()},
        unavailable_publications=UNAVAILABLE if profile == 'legacy_1_14_passive' else {})
    (root / 'schema.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')


def verify(root, prefix, profile, revision=None):
    import yaml

    manifest = json.loads((root / 'schema.json').read_text())
    check_profile(profile, manifest['firmware_commit'])
    if manifest.get('manifest_version') != 2 or not manifest['schemas']:
        raise ValueError('Incomplete or obsolete schema manifest; rebuild with an explicit profile')
    if manifest['compatibility_profile'] != profile:
        raise ValueError('Compatibility profile differs from the manifest')
    if revision is not None and revision != manifest['firmware_commit']:
        raise ValueError('Firmware commit differs from the manifest')
    if prefix.resolve() != (root / 'install').resolve():
        raise ValueError(f'px4_msgs resolved outside the requested overlay: {prefix}')
    marker = prefix / 'share/px4_interface_node/compatibility_profile'
    if marker.read_text().strip() != profile:
        raise ValueError('Compiled interface compatibility profile differs from the manifest')
    installed = prefix / 'share/px4_msgs'
    if schema_hashes(installed) != manifest['schemas']:
        raise ValueError('Installed px4_msgs differ from the pinned firmware manifest')
    firmware = root / 'firmware'
    inputs = check_firmware(firmware, root / 'px4_msgs', profile, manifest['firmware_commit'])
    if inputs != manifest['firmware_inputs']:
        raise ValueError('Firmware inputs differ from the pinned firmware manifest')
    dds = firmware / DDS_TOPICS
    if digest(dds) != manifest['dds_topics_sha256']:
        raise ValueError('DDS topic mapping differs from the pinned firmware manifest')
    versions = validate_topics(yaml.safe_load(dds.read_text())['publications'], installed, profile)
    publications = {topic: f'px4_msgs::msg::{name}' for topic, (name, _) in
                    required_publications(profile, installed).items()}
    unavailable = UNAVAILABLE if profile == 'legacy_1_14_passive' else {}
    if (versions != manifest['telemetry_message_versions'] or
            publications != manifest['required_publications'] or
            unavailable != manifest['unavailable_publications'] or
            validate_fields(installed, profile) != manifest['message_dependencies']):
        raise ValueError('Installed telemetry contract differs from the manifest')
    print(f"PX4 {manifest['firmware_commit']}, profile={profile}, {len(manifest['schemas'])} verified schemas")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('root', type=Path)
    prep.add_argument('checkout')
    prep.add_argument('revision')
    prep.add_argument('--profile', choices=PROFILES, required=True)
    check = sub.add_parser('verify')
    check.add_argument('root', type=Path)
    check.add_argument('prefix', type=Path)
    check.add_argument('--profile', choices=PROFILES, required=True)
    check.add_argument('--firmware-commit')
    args = parser.parse_args()
    try:
        if args.mode == 'prepare':
            prepare(args.root, args.checkout, args.revision, args.profile)
        else:
            verify(args.root, args.prefix, args.profile, args.firmware_commit)
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as exc:
        sys.exit(f'PX4 schema check failed: {exc}')
