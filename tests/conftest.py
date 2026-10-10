import sys
import os
import json
import shutil
import pytest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in ('scripts', 'ros2_ws/src/uav_lab_bridge', 'ros2_ws/src/uav_lab_tools',
             'ros2_ws/src/uav_lab_localization', 'ros2_ws/src/uav_lab_navigation',
             'ros2_ws/src/uav_lab_experiments'):
    sys.path.insert(0, str(ROOT / path))

def select_test_domain(runtime, environment):
    excluded = {int(environment[key]) for key in ('ROS_DOMAIN_ID', 'LAB_DOMAIN_ID')
                if environment.get(key)}
    for manifest in runtime.glob('*/manifest.json'):
        if (manifest.parent/'ready').exists():
            excluded.add(int(json.loads(manifest.read_text())['environment']['ROS_DOMAIN_ID']))
    return next(domain for domain in range(90, 101) if domain not in excluded)

@pytest.fixture
def isolated_ros_domain():
    return select_test_domain(ROOT/'.runtime', os.environ)


@pytest.fixture
def pinned_backend_config_root(tmp_path, monkeypatch):
    """Config-only tests use real installed templates, or audited fixture copies.

    This does not mark a backend installed or bypass its production build gate.
    Native tests still read their actual pinned private dependency checkout.
    """
    fixtures = ROOT/'tests/fixtures/backend-configs'
    manifest = json.loads((fixtures/'provenance.json').read_text())
    required = [ROOT/'.deps'/repository/name
        for repository, data in manifest['repositories'].items()
        for name in data['files']]
    if all(path.is_file() for path in required):
        return ROOT
    import backend_configs
    destination = tmp_path/'config-only-checkout'
    shutil.copytree(fixtures/'.deps', destination/'.deps')
    (destination/'configs').symlink_to(ROOT/'configs', target_is_directory=True)
    monkeypatch.setattr(backend_configs, 'ROOT', destination)
    return destination
