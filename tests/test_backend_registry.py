"""Catch unsafe selection and qualification inferred from a process or stale report."""
import hashlib
import json
from pathlib import Path

import pytest

from uav_lab_experiments.registry import Registry, GroundState, select_backends


REF = 'a' * 40


@pytest.fixture
def registry(tmp_path):
    catalog = {'schema': 1, 'groups': {'lidar_imu': ['lidar', 'imu'], 'visual': ['camera', 'imu']},
        'backends': [
            {'id': 'lio', 'role': 'localization', 'source_ref': REF, 'groups': ['lidar_imu'],
             'capabilities': {'odometry': True, 'loop_closure': False}},
            {'id': 'astar', 'role': 'planning', 'source_ref': REF, 'groups': ['lidar_imu'],
             'capabilities': {'continuous_trajectory': True}}]}
    path = tmp_path / 'catalog.json'
    path.write_text(json.dumps(catalog))
    return Registry(path, tmp_path / 'evidence')


def report(registry, backend='lio', stage='installed', ref=REF, group='lidar_imu', success=True):
    artifact = registry.evidence_dir.parent / (backend + '-' + stage + '.bin')
    artifact.write_bytes(b'real run artifact')
    return {'schema': 1, 'backend': backend, 'stage': stage, 'source_ref': ref,
            'input_group': group, 'run_id': backend + '-' + stage, 'success': success,
            'checks': {'all_passed': success},
            'artifacts': [{'path': str(artifact), 'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest()}]}


def install_pair(registry):
    for backend in ['lio', 'astar']:
        registry.record(report(registry, backend))


def test_no_report_is_not_installed(registry):
    assert registry.describe('lio')['stage'] == 'configured'
    assert registry.describe('lio')['capabilities']['loop_closure'] is False
    with pytest.raises(ValueError, match='installed'):
        select_backends(registry, registry.evidence_dir.parent / 'selection.json', 'lio', 'astar',
                        'lidar_imu', GroundState(False, True, True, 10.), 10.1)


def test_qualification_requires_prior_stages_and_matching_inputs(registry):
    with pytest.raises(ValueError, match='previous'):
        registry.record(report(registry, stage='replay_passed'))
    registry.record(report(registry))
    with pytest.raises(ValueError, match='input group'):
        registry.record(report(registry, stage='replay_passed', group='visual'))
    with pytest.raises(ValueError, match='source'):
        registry.record(report(registry, stage='replay_passed', ref='b' * 40))
    for stage in ['replay_passed', 'realtime_passed', 'closed_loop_qualified']:
        registry.record(report(registry, stage=stage))
    assert registry.describe('lio', 'lidar_imu')['stage'] == 'closed_loop_qualified'
    assert registry.describe('lio', 'visual')['stage'] == 'configured'


def test_failed_run_retained_does_not_promote(registry):
    registry.record(report(registry, success=False))
    assert registry.describe('lio')['stage'] == 'configured'
    assert len(list(registry.evidence_dir.rglob('*.json'))) == 1


def test_artifact_tampering_revokes_stage(registry):
    evidence = report(registry)
    registry.record(evidence)
    Path(evidence['artifacts'][0]['path']).write_bytes(b'changed after acceptance')
    assert registry.describe('lio')['stage'] == 'configured'


def test_source_patches_and_adapter_fingerprint_revoke_old_stage(registry):
    registry.backends['lio']['implementation_sha256']='a'*64
    evidence=report(registry);evidence['implementation_sha256']='a'*64
    registry.record(evidence)
    assert registry.describe('lio')['stage']=='installed'
    registry.backends['lio']['implementation_sha256']='b'*64
    assert registry.describe('lio')['stage']=='configured'


def test_evidence_immutable_and_rejects_missing_artifact(registry):
    evidence = report(registry)
    registry.record(evidence)
    with pytest.raises(FileExistsError):
        registry.record(evidence)
    evidence = report(registry, stage='replay_passed')
    evidence['artifacts'][0]['sha256'] = '0' * 64
    with pytest.raises(ValueError, match='artifact'):
        registry.record(evidence)


@pytest.mark.parametrize('state', [GroundState(True, True, True, 10.),
                                  GroundState(False, False, True, 10.),
                                  GroundState(False, True, False, 10.),
                                  GroundState(False, True, True, 8.),
                                  GroundState(False, True, True, 11.)])
def test_selection_rejected_without_fresh_ground_idle_state(registry, state):
    install_pair(registry)
    selection = registry.evidence_dir.parent / 'selection.json'
    with pytest.raises(ValueError, match='ground'):
        select_backends(registry, selection, 'lio', 'astar', 'lidar_imu', state, 10.1)
    assert not selection.exists()


def test_selection_checks_roles_and_commits_atomic_configuration(registry):
    install_pair(registry)
    selection = registry.evidence_dir.parent / 'selection.json'
    state = GroundState(False, True, True, 10.)
    with pytest.raises(ValueError, match='role'):
        select_backends(registry, selection, 'astar', 'lio', 'lidar_imu', state, 10.1)
    result = select_backends(registry, selection, 'lio', 'astar', 'lidar_imu', state, 10.1)
    assert json.loads(selection.read_text())['localization'] == 'lio'
    assert result['restart_required'] is True


def test_catalog_rejects_duplicate_ids_and_unknown_groups(registry):
    catalog = json.loads(registry.catalog_path.read_text())
    catalog['backends'].append(catalog['backends'][0])
    registry.catalog_path.write_text(json.dumps(catalog))
    with pytest.raises(ValueError, match='duplicate'):
        Registry(registry.catalog_path, registry.evidence_dir)


def test_platform_catalog_declares_seven_localizers_four_planners_without_fake_ready():
    root = Path(__file__).resolve().parents[1]
    catalog = Registry(root / 'configs/backends.json', root / '.runtime/nonexistent-evidence')
    assert len(catalog.list('localization')) == 7
    assert len(catalog.list('planning')) == 4
    assert all(item['stage'] == 'configured' for item in catalog.list())
