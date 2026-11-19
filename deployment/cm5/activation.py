"""Durable passive-release journal. The installed boot copy executes no release code."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


class Activation:
    def __init__(self, root):
        self.root = root.resolve()
        self.state = self.root / 'state'
        self.journal = self.state / 'activation.json'
        self.current = self.state / 'current'

    def sync_directory(self):
        fd = os.open(self.state, os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def release(self, value):
        if not isinstance(value, str):
            raise ValueError('Invalid release path')
        path = Path(value)
        if (path.parent != self.root / 'releases' or path.resolve() != path
                or not (path / 'verified').is_file()):
            raise ValueError(f'Release unavailable or unverified: {value}')
        return path

    def read(self):
        value = json.loads(self.journal.read_text())
        if (not isinstance(value, dict) or set(value) != {'phase', 'candidate', 'previous'}
                or value['phase'] not in ('prepared', 'verifying', 'verified',
                                          'rollback_required', 'rolled_back')
                or not isinstance(value['candidate'], str)
                or value['previous'] is not None and not isinstance(value['previous'], str)):
            raise ValueError('Invalid activation journal')
        return value

    def write(self, value, phase):
        with tempfile.NamedTemporaryFile(mode='w', dir=self.state, delete=False) as file:
            temporary = Path(file.name)
            try:
                file.write(json.dumps(dict(value, phase=phase)) + '\n')
                file.flush()
                os.fchmod(file.fileno(), 0o644)
                os.fsync(file.fileno())
                os.replace(temporary, self.journal)
                self.sync_directory()
            finally:
                temporary.unlink(missing_ok=True)

    def point(self, release):
        if release is None:
            self.current.unlink(missing_ok=True)
        else:
            with tempfile.TemporaryDirectory(dir=self.state) as directory:
                link = Path(directory) / 'current'
                link.symlink_to(release)
                os.replace(link, self.current)
        self.sync_directory()

    def prepare(self, candidate):
        self.release(candidate)
        previous = None
        if self.journal.exists():
            self.reconcile_locked()
            if self.current.is_symlink():
                previous = str(self.release(os.readlink(self.current)))
        # A legacy symlink has no runtime verification record.
        self.write(dict(candidate=candidate, previous=previous), 'prepared')

    def rollback(self, record):
        self.write(record, 'rollback_required')
        previous = self.release(record['previous']) if record['previous'] is not None else None
        self.point(previous)
        self.write(record, 'rolled_back')

    def reconcile_locked(self):
        record = self.read()
        if record['phase'] not in ('verified', 'rolled_back'):
            self.rollback(record)
            record = self.read()
        selected = record['candidate'] if record['phase'] == 'verified' else record['previous']
        if selected is None:
            if self.current.is_symlink():
                raise ValueError('Unexpected selection after initial rollback')
            return None
        if not self.current.is_symlink() or os.readlink(self.current) != str(self.release(selected)):
            raise ValueError('Selection differs from the verified activation journal')
        return selected

    def boot(self):
        # The updater holds this lock throughout live verification. A systemd
        # start then permits just the journaled candidate in VERIFYING.
        with (self.state / 'update.lock').open('rb') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                record = self.read()
                if (record['phase'] == 'verifying' and self.current.is_symlink()
                        and os.readlink(self.current) == str(self.release(record['candidate']))):
                    return
                selected = record['candidate'] if record['phase'] == 'verified' else record['previous']
                if (record['phase'] in ('verified', 'rolled_back') and selected is not None
                        and self.current.is_symlink()
                        and os.readlink(self.current) == str(self.release(selected))):
                    return
                raise ValueError('Activation in progress without a startable release')
            if self.reconcile_locked() is None:
                raise ValueError('No runtime-verified release. Run activation explicitly.')

    def check_running(self, release):
        record = self.read()
        if record['phase'] == 'verifying' and record['candidate'] == str(release):
            with (self.state / 'update.lock').open('rb') as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    return
                # The updater may have committed before releasing the lock.
                record = self.read()
        if (record['phase'] == 'verified' and record['candidate'] == str(release)
                or record['phase'] == 'rolled_back' and record['previous'] == str(release)):
            return
        raise ValueError('Running release differs from the verified or in-progress activation')

    def launch(self, role):
        if role not in ('agent', 'stack'):
            raise ValueError('Invalid service role')
        release = self.release(os.readlink(self.current))
        self.check_running(release)
        with subprocess.Popen(['/bin/bash', release / 'source/deployment/cm5/run.sh', role]) as child:
            try:
                while True:
                    self.check_running(release)
                    try:
                        return child.wait(timeout=0.25)
                    except subprocess.TimeoutExpired:
                        continue
            finally:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()

    def run(self, action, candidate):
        if action == 'boot':
            self.boot()
        elif action == 'prepare':
            self.prepare(candidate)
        else:
            record = self.read()
            if action == 'switch' and record['phase'] == 'prepared':
                self.point(self.release(record['candidate']))
                self.write(record, 'verifying')
            elif action == 'commit' and record['phase'] == 'verifying':
                if os.readlink(self.current) != str(self.release(record['candidate'])):
                    raise ValueError('Candidate selection changed during verification')
                self.write(record, 'verified')
            elif action == 'rollback' and record['phase'] != 'verified':
                self.rollback(record)
            else:
                raise ValueError('Invalid activation transition')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/opt/drone'))
    parser.add_argument('action', choices=('prepare', 'switch', 'commit', 'rollback', 'boot', 'launch'))
    parser.add_argument('candidate', nargs='?')
    args = parser.parse_args()
    try:
        activation = Activation(args.root)
        if args.action == 'launch':
            sys.exit(activation.launch(args.candidate))
        activation.run(args.action, args.candidate)
    except (OSError, ValueError) as error:
        sys.exit(f'Activation refused: {error}')


if __name__ == '__main__':
    main()
