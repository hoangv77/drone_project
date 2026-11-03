"""Validate the onboard DDS contract and record the exact input schemas."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

DDS_TOPICS = 'src/modules/uxrce_dds_client/dds_topics.yaml'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_topics(publications, staged):
    topics = {entry['topic']: entry['type'] for entry in publications}
    gnss = ('vehicle_gps_position', 'SensorGps') if (staged / 'msg/SensorGps.msg').exists() else (
        'vehicle_gnss', 'VehicleGnss')
    expected = {
        'vehicle_status': ('VehicleStatus', 4),
        'vehicle_local_position': ('VehicleLocalPosition', 1),
        'vehicle_odometry': ('VehicleOdometry', 0),
        'vehicle_land_detected': ('VehicleLandDetected', 0),
        'battery_status': ('BatteryStatus', 1),
        'failsafe_flags': ('FailsafeFlags', 0),
        gnss[0]: (gnss[1], 0),
    }
    versions = {}
    for topic, (name, required_version) in expected.items():
        if topics.get(f'/fmu/out/{topic}') != f'px4_msgs::msg::{name}':
            raise ValueError(f'Missing or incompatible DDS publication /fmu/out/{topic} ({name})')
        schema = (staged / 'msg' / f'{name}.msg').read_text()
        version = re.search(r'^\s*uint32\s+MESSAGE_VERSION\s*=\s*(\d+)', schema, re.M)
        version = int(version[1]) if version else 0
        if version != required_version:
            raise ValueError(f'{name} DDS schema version {version}, expected {required_version}')
        versions[name] = version
    return versions


def prepare(root, checkout, revision):
    import yaml

    if not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('PX4 revision must be the full lowercase firmware commit SHA')
    if subprocess.check_output(
        ['git', '-C', checkout, 'rev-parse', f'{revision}^{{commit}}'], text=True
    ).strip() != revision:
        raise ValueError('The PX4 revision must identify a commit')
    firmware = root / 'firmware'
    firmware.mkdir()
    paths = ['msg', DDS_TOPICS]
    for optional in ['srv', 'LICENSE']:
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
    dds = firmware / DDS_TOPICS
    versions = validate_topics(yaml.safe_load(dds.read_text())['publications'], staged)
    hashes = {str(path.relative_to(staged)): digest(path)
              for pattern in ('msg/*.msg', 'srv/*.srv') for path in sorted(staged.glob(pattern))}
    (root / 'schema.json').write_text(json.dumps({
        'firmware_commit': revision, 'dds_topics_sha256': digest(dds),
        'telemetry_message_versions': versions, 'schemas': hashes,
    }, indent=2, sort_keys=True) + '\n')


def verify(root, prefix):
    import yaml

    manifest = json.loads((root / 'schema.json').read_text())
    if not re.fullmatch(r'[0-9a-f]{40}', manifest['firmware_commit']) or not manifest['schemas']:
        raise ValueError('Incomplete schema manifest')
    if prefix.resolve() != (root / 'install').resolve():
        raise ValueError(f'px4_msgs resolved outside the requested overlay: {prefix}')
    installed = prefix / 'share/px4_msgs'
    actual = {str(path.relative_to(installed)): digest(path)
              for pattern in ('msg/*.msg', 'srv/*.srv') for path in installed.glob(pattern)}
    if actual != manifest['schemas']:
        raise ValueError('Installed px4_msgs differ from the pinned firmware manifest')
    dds = root / 'firmware' / DDS_TOPICS
    if digest(dds) != manifest['dds_topics_sha256']:
        raise ValueError('DDS topic mapping differs from the pinned firmware manifest')
    versions = validate_topics(yaml.safe_load(dds.read_text())['publications'], installed)
    if versions != manifest['telemetry_message_versions']:
        raise ValueError('Installed telemetry versions differ from the manifest')
    print(f"PX4 {manifest['firmware_commit']}, {len(actual)} verified schemas")
    print(json.dumps(manifest['telemetry_message_versions'], sort_keys=True))


if __name__ == '__main__':
    try:
        mode, root, *args = sys.argv[1:]
        if mode == 'prepare':
            prepare(Path(root), *args)
        elif mode == 'verify':
            verify(Path(root), Path(args[0]))
        else:
            raise ValueError(f'Unknown schema mode {mode}')
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        sys.exit(f'PX4 schema check failed: {exc}')
