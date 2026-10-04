import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from slam_config import prepare_slam


ROOT = Path(__file__).resolve().parents[1]


def test_cpu_config_preserves_physical_sensor_contract(tmp_path):
    prepare_slam(ROOT, tmp_path)
    sensors = json.loads((ROOT / 'configs/sensors.json').read_text())
    cfg = json.loads((tmp_path / 'config_sensors.json').read_text())['sensors']
    # A point at the IMU origin is below the lidar origin by the actual mounting height.
    assert np.allclose(cfg['T_lidar_imu'][:3], [0, 0, -0.16])
    assert np.allclose(Rotation.from_quat(cfg['T_lidar_imu'][3:]).as_matrix(), np.eye(3))
    assert cfg['global_shutter_lidar'] is True
    assert cfg['autoconf_perpoint_times'] is False
    ros = json.loads((tmp_path / 'config_ros.json').read_text())['glim_ros']
    assert ros['imu_topic'] == '/uav001/imu/data'
    assert ros['points_topic'] == '/uav001/lidar/points'
    assert ros['acc_scale'] == 1.0
    assert ros['base_frame_id'] == ros['imu_frame_id'] == 'lio_base_link'
    assert ros['publish_imu2lidar'] is False
    cfg = json.loads((tmp_path / 'config.json').read_text())['global']
    assert cfg['config_odometry'] == 'config_odometry_cpu.json'
    assert cfg['config_global_mapping'] == 'config_global_mapping_pose_graph.json'


def test_rejects_unimplemented_scan_timing(tmp_path):
    sensors = json.loads((ROOT / 'configs/sensors.json').read_text())
    sensors['lidar']['scan_timing'] = 'rolling'
    with pytest.raises(ValueError, match='synchronous'):
        prepare_slam(ROOT, tmp_path, sensors)
