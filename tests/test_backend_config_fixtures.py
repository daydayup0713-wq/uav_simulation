"""Exercise actual config builders on a checkout with no downloaded algorithms."""
import json
import hashlib
from pathlib import Path

import pytest


@pytest.mark.parametrize('backend,profile', [
    ('fast_lio2', 'livox'), ('fast_lio2', 'mechanical'),
    ('fast_livo2', 'livox'), ('fast_livo2_rtk', 'livox-rtk'),
    ('lio_sam', 'mechanical'), ('orb_slam3', 'livox'), ('vins_fusion', 'livox')])
def test_config_builders_work_with_only_pinned_test_templates(backend, profile, tmp_path, monkeypatch):
    import backend_configs
    root = Path(__file__).resolve().parents[1]
    fixture_root = root / 'tests/fixtures/backend-configs'
    monkeypatch.setattr(backend_configs, 'ROOT', fixture_root)
    calibration = json.loads((root / f'configs/sensors-{profile}.json').read_text())
    path = backend_configs.write_config(backend, calibration, tmp_path, 'clean-checkout')
    assert path.is_file()
    assert not (fixture_root / '.deps/backends/install').exists()


def test_test_templates_match_locked_sources_and_retained_hashes():
    root = Path(__file__).resolve().parents[1]
    fixtures = root/'tests/fixtures/backend-configs'
    lock = json.loads((root/'dependencies/backends.lock.json').read_text())
    manifest = json.loads((fixtures/'provenance.json').read_text())
    for repository, data in manifest['repositories'].items():
        for field in ('url', 'ref', 'tree_sha256', 'patches'):
            assert data[field] == lock['repositories'][repository][field]
        for name, digest in data['files'].items():
            assert hashlib.sha256((fixtures/'.deps'/repository/name).read_bytes()).hexdigest() == digest
            installed = root/'.deps'/repository/name
            if installed.is_file():
                assert hashlib.sha256(installed.read_bytes()).hexdigest() == digest
