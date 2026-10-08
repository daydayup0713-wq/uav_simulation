import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_six_scenarios_have_repeatable_complex_routes_and_matching_geometry(tmp_path):
    from experiment_scenarios import SCENES, generate_scene
    assert len(SCENES) == 6
    for name in SCENES:
        a = generate_scene(ROOT, tmp_path/name, name)
        b = generate_scene(ROOT, tmp_path/(name+'-repeat'), name)
        assert a['route'] == b['route']
        points = np.array(a['route']['points'])
        assert 20 <= len(points) <= 50
        assert np.isfinite(points).all()
        assert np.ptp(points[:, 0]) > 3
        assert all(0.5 <= z <= 4.5 for z in points[:, 2])
        assert ET.parse(a['world']).getroot().find('world').get('name') == 'lab'
        geometry = json.loads(a['geometry'].read_text())
        assert len(geometry['boxes']) >= 4
        for p in points:
            assert not any(np.all(p > np.array(box[0])-.4) and np.all(p < np.array(box[1])+.4)
                           for box in geometry['boxes'])
        assert a['texture'].read_bytes().startswith(b'\x89PNG')
        assert a['route']['truth_input_allowed'] is False
    with pytest.raises(ValueError, match='scene'):
        generate_scene(ROOT, tmp_path/'bad', '../bad')


@pytest.mark.parametrize('profile,kind', [('livox','livox'),('livox-rtk','livox'),('mechanical','mechanical')])
def test_new_sensor_profiles_preserve_old_and_publish_private_physics_only(tmp_path, profile, kind):
    from sensor_model import prepare_sensors
    result = prepare_sensors(ROOT, tmp_path/profile, sensor_profile=profile, scene='circle-eight')
    c = result['calibration']
    assert c['lidar']['kind'] == kind and c['lidar']['measurement_time'] == 'per_beam'
    assert (c['camera']['width'], c['camera']['height'], c['camera']['hz'], c['imu']['hz']) == (640,480,30,200)
    model = ET.parse(result['model']).getroot()
    assert model.find(".//sensor[@name='lab_lidar']") is None
    physics = model.find(".//plugin[@name='gz::sim::systems::OdometryPublisher']/odom_publish_frequency")
    assert physics is not None
    bridges = json.loads(result['bridge'].read_text())
    assert any(v['ros_topic_name'] == '/_lab/sensor_physics/odometry' for v in bridges)
    assert not any(v['ros_topic_name'] == '/uav001/lidar/points' for v in bridges)
    assert c['imu'].get('attitude_stddev_rad', 0) > 0 if kind == 'mechanical' else 'attitude_stddev_rad' not in c['imu']
    assert ('gnss' in c) == (profile == 'livox-rtk')
