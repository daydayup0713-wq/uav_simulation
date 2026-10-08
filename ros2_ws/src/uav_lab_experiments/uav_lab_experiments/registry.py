"""Backend contracts and immutable artifact-backed qualification.

Selection writes the next-run configuration; live backend swapping is deliberately
absent. The bridge calls selection while holding its flight-policy lock.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile

STAGES = ('configured', 'installed', 'replay_passed', 'realtime_passed', 'closed_loop_qualified')


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


class Registry:
    def __init__(self, catalog_path, evidence_dir):
        self.catalog_path, self.evidence_dir = Path(catalog_path), Path(evidence_dir)
        catalog = json.loads(self.catalog_path.read_text())
        if catalog.get('schema') != 1:
            raise ValueError('unsupported backend catalog schema')
        self.groups, self.backends = catalog['groups'], {}
        for item in catalog['backends']:
            key = item['id']
            if not re.fullmatch(r'[a-z][a-z0-9_]*', key) or key in self.backends:
                raise ValueError('invalid or duplicate backend id')
            if item['role'] not in ('localization', 'planning'):
                raise ValueError('invalid backend role')
            if not item['groups'] or any(group not in self.groups for group in item['groups']):
                raise ValueError('unknown input group')
            self.backends[key] = item

    def _valid(self, report, backend, group):
        try:
            spec = self.backends[backend]
            if (report['schema'] != 1 or report['backend'] != backend
                    or report['source_ref'] != spec['source_ref']
                    or not re.fullmatch(r'[0-9a-f]{40}', report['source_ref'] or '')
                    or report['input_group'] != group or group not in spec['groups']
                    or report['success'] is not True or report['checks']['all_passed'] is not True
                    or report['stage'] not in STAGES[1:] or not report['artifacts']):
                return False
            return all(Path(a['path']).is_file() and digest(a['path']) == a['sha256']
                       for a in report['artifacts'])
        except (KeyError, ValueError, TypeError, OSError):
            return False

    def describe(self, backend, group=None):
        if backend not in self.backends:
            raise ValueError('unknown backend: ' + backend)
        spec = self.backends[backend]
        groups = [group] if group is not None else spec['groups']
        best, witnesses = 0, []
        for input_group in groups:
            reports = []
            for path in (self.evidence_dir / backend).glob('*.json'):
                try:
                    report = json.loads(path.read_text())
                    if self._valid(report, backend, input_group):
                        reports.append((report, str(path)))
                except (OSError, ValueError):
                    continue
            current, proof = 0, []
            for stage in STAGES[1:]:
                matching = [path for r, path in reports if r['stage'] == stage]
                if not matching:
                    break
                current += 1
                proof.extend(matching)
            if current > best:
                best, witnesses = current, proof
        return {**spec, 'stage': STAGES[best], 'evidence': witnesses}

    def list(self, role=None, group=None):
        return [self.describe(key, group) for key, spec in self.backends.items()
                if role is None or spec['role'] == role]

    def record(self, report):
        backend, group, stage = report['backend'], report['input_group'], report['stage']
        if backend not in self.backends or stage not in STAGES[1:]:
            raise ValueError('unknown backend or qualification stage')
        if group not in self.backends[backend]['groups']:
            raise ValueError('unsupported input group')
        if report['source_ref'] != self.backends[backend]['source_ref']:
            raise ValueError('source version mismatch')
        run_id = report['run_id']
        if not re.fullmatch(r'[a-zA-Z0-9_-]{1,128}', run_id):
            raise ValueError('invalid run id')
        if report['success']:
            if not self._valid(report, backend, group):
                raise ValueError('invalid report, checks or artifact hash')
            previous = STAGES[STAGES.index(stage) - 1]
            if STAGES.index(self.describe(backend, group)['stage']) < STAGES.index(previous):
                raise ValueError('previous qualification stage missing')
        directory = self.evidence_dir / backend
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (run_id + '.json')
        # Exclusive create: acceptance reports cannot overwrite an earlier run.
        with path.open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        return path


@dataclass(frozen=True)
class GroundState:
    armed: bool
    landed: bool
    idle: bool
    received_at: float


def select_backends(registry, destination, localization, planning, group, state, now):
    if (not math.isfinite(now) or not math.isfinite(state.received_at)
            or not 0 <= now - state.received_at <= 1.
            or state.armed is not False or state.landed is not True or state.idle is not True):
        raise ValueError('selection requires fresh, disarmed, landed, idle ground state')
    for role, key in [('localization', localization), ('planning', planning)]:
        item = registry.describe(key, group)
        if item['role'] != role:
            raise ValueError('backend role mismatch')
        if group not in item['groups']:
            raise ValueError('unsupported input group')
        if item['stage'] == 'configured':
            raise ValueError('backend not installed with valid evidence: ' + key)
    result = {'schema': 1, 'localization': localization, 'planning': planning,
              'input_group': group, 'restart_required': True,
              'catalog_sha256': digest(registry.catalog_path)}
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.selection-', dir=destination.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return result
