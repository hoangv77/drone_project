"""Recording limits, atomic metadata and immutable ULog copies."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
from uuid import uuid4

MIB = 1024 * 1024


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, record):
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        json.dump(record, stream, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def size_bytes(path):
    size = 0
    for file in path.rglob('*'):
        try:
            if not file.is_symlink() and file.is_file():
                size += file.stat().st_size
        except FileNotFoundError:
            # SQLite can remove temporary files while the monitor is scanning.
            continue
    return size


def storage_status(root, session, config):
    used = size_bytes(root)
    current = size_bytes(session) if session else 0
    free = shutil.disk_usage(root).free
    remaining = {
        'total': config['total_mib'] * MIB - used,
        'session': config['session_mib'] * MIB - current,
        'free': free - config['free_mib'] * MIB,
    }
    critical = [name for name, budget in remaining.items() if budget <= 0]
    warning = [name for name, budget in remaining.items() if 0 < budget < 128 * MIB]
    return {'used_bytes': used, 'session_bytes': current, 'free_bytes': free,
            'limits_reached': critical, 'near_limits': warning}


def new_session(root):
    name = datetime.now(timezone.utc).strftime('%Y-%m-%d_%H-%M-%S') + '_' + uuid4().hex[:8]
    session = root / name
    session.mkdir(mode=0o750)
    (session / 'logs').mkdir()
    return session


def read_metadata(session):
    return json.loads((session / 'metadata.json').read_text())


def hash_file(path):
    with path.open('rb') as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(MIB), b''):
            digest.update(block)
    return digest.hexdigest()


def attach_ulog(session, source):
    if source.suffix.lower() != '.ulg' or not source.is_file():
        raise ValueError('Supply an existing PX4 .ulg file')
    metadata = read_metadata(session)
    logs = session / 'logs'
    destination = logs / source.name
    with source.open('rb') as original, destination.open('xb') as copy:
        try:
            shutil.copyfileobj(original, copy)
        except OSError:
            destination.unlink()
            raise
    metadata.setdefault('ulogs', []).append({
        'path': str(destination.relative_to(session)), 'sha256': hash_file(destination),
        'attached_utc': utc_now(),
    })
    atomic_json(session / 'metadata.json', metadata)
    return destination


def selected_session(root, name):
    path = root / name
    if Path(name).name != name or path.is_symlink() or not (path / 'metadata.json').is_file():
        raise ValueError(f'Unknown session {name}')
    return path
