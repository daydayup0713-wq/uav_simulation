import sys
import os
import json
import pytest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in ('scripts', 'ros2_ws/src/uav_lab_bridge', 'ros2_ws/src/uav_lab_tools',
             'ros2_ws/src/uav_lab_localization'):
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
