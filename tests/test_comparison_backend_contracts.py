import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from backend_configs import simulation_parameters
from uav_lab_experiments.backend_contract import input_topics


def rig(name):
    return json.loads(Path('configs/sensors-' + name + '.json').read_text())


def test_visual_backends_only_receive_calibrated_camera_and_imu():
    for backend in ('orb_slam3', 'vins_fusion'):
        assert set(input_topics(backend)) == {'/clock', '/uav001/imu/data',
            '/uav001/camera/image_raw', '/uav001/camera/camera_info'}
        parameters, camera = simulation_parameters(backend, rig('livox'), 'test')
        assert parameters['metric_scale_source'] == 'inertial'
        expected = Rotation.from_euler('xyz', rig('livox')['camera']['optical_rpy']).as_matrix()
        assert np.array(parameters['imu_T_camera']).reshape(4, 4)[:3, :3] == pytest.approx(expected)
        assert camera['cam_fx'] == pytest.approx(320.)


def test_lio_sam_requires_measured_ring_scans_and_attitude_imu():
    with pytest.raises(ValueError, match='mechanical'):
        simulation_parameters('lio_sam', rig('livox'), 'test')
    calibration = rig('mechanical')
    parameters, _ = simulation_parameters('lio_sam', calibration, 'test')
    assert parameters['sensor'] == 'ouster'
    assert parameters['pointCloudTopic'] == '/uav001/backends/lio_sam/points'
    assert parameters['N_SCAN'] == 16 and parameters['Horizon_SCAN'] == 400
    assert parameters['savePCD'] is False
    assert isinstance(parameters['lidarMaxRange'],float)
    assert parameters['gpsTopic'].startswith('/uav001/backends/lio_sam/')
    calibration['imu'].pop('attitude_stddev_rad')
    with pytest.raises(ValueError, match='attitude'):
        simulation_parameters('lio_sam', calibration, 'test')


def test_unknown_backend_does_not_receive_fallback_sensor_contract():
    with pytest.raises(ValueError, match='unknown'):
        input_topics('not_an_algorithm')


def test_rejected_monocular_scale_cannot_be_claimed_metric():
    from uav_lab_experiments.visual_contract import metric_tracking_ready
    assert not metric_tracking_ready('orb_slam3', 2, False)
    assert metric_tracking_ready('orb_slam3', 2, True)
    assert not metric_tracking_ready('orb_slam3', 3, True)
    assert not metric_tracking_ready('vins_fusion', 0, False)
    assert metric_tracking_ready('vins_fusion', 1, True)


def test_visual_quality_requires_camera_instead_of_an_unconsumed_lidar():
    from uav_lab_experiments.localization_quality import LocalizationQuality
    quality = LocalizationQuality(warmup_samples=1, source_limits={'imu': .25, 'camera': .25})
    assert quality.observe_source('imu', 10., 1.)
    assert quality.observe_pose(10., [0., 0., 0.], [0., 0., 0., 1.], 1.)
    assert not quality.check(10.1, 1.1)['ready']
    assert quality.observe_source('camera', 10., 1.)
    assert quality.check(10.1, 1.1)['ready']
    assert quality.check(10.3, 1.3)['state'] == 'FAILED'


def test_lio_sam_processes_share_private_graph_and_separate_global_output():
    from backend_launch import core_commands
    commands = core_commands('lio_sam', '/private/lio_sam_mapOptimization', '/config')
    assert len(commands) == 4
    for command in commands:
        assert '__ns:=/uav001/backends/lio_sam' in command
        assert 'lio_sam/mapping/odometry_incremental:=/uav001/backends/lio_sam/raw_odometry' in command
        assert 'lio_sam/mapping/odometry:=/uav001/backends/lio_sam/global_odometry' in command


def test_generated_visual_configs_are_readable_by_actual_opencv(tmp_path):
    import cv2
    from backend_configs import write_config
    for backend in ('orb_slam3', 'vins_fusion'):
        directory = tmp_path / backend
        write_config(backend, rig('livox'), directory, 'test')
        settings = cv2.FileStorage(str(directory/'algorithm.yaml'), cv2.FILE_STORAGE_READ)
        assert settings.isOpened()
        transform = settings.getNode('IMU.T_b_c1' if backend=='orb_slam3' else 'body_T_cam0').mat()
        assert transform.shape == (4, 4)
        assert np.linalg.det(transform[:3,:3]) == pytest.approx(1.)
        settings.release()


def test_glim_timed_profile_keeps_real_beam_times_without_changing_old_default(tmp_path):
    from uav_lab_localization.configuration import prepare_slam
    root=Path('.').resolve()
    timed=prepare_slam(root,tmp_path/'timed',rig('livox'),use_timed_scans=True)
    parameters=json.loads((timed/'config_sensors.json').read_text())['sensors']
    assert parameters['global_shutter_lidar'] is False
    assert parameters['perpoint_relative_time'] is True
    old=prepare_slam(root,tmp_path/'old')
    assert json.loads((old/'config_sensors.json').read_text())['sensors']['global_shutter_lidar'] is True


def test_public_visual_output_paths_are_absolute_for_opencv(tmp_path, monkeypatch):
    import cv2
    from backend_configs import public_ntu_parameters,write_visual_settings
    monkeypatch.chdir(tmp_path)
    directory=Path('.local/config');directory.mkdir(parents=True)
    parameters,camera,_=public_ntu_parameters('vins_fusion','test')
    write_visual_settings('vins_fusion',parameters,camera,directory)
    settings=cv2.FileStorage(str(directory/'algorithm.yaml'),cv2.FILE_STORAGE_READ)
    assert settings.isOpened()
    assert Path(settings.getNode('output_path').string()).is_absolute()


def test_lio_parameter_yaml_has_no_aliases_rejected_by_ros_parser(tmp_path):
    from backend_configs import write_config
    path=write_config('lio_sam',rig('mechanical'),tmp_path,'test')
    assert '&id' not in path.read_text() and '*id' not in path.read_text()


def test_vins_pose_graph_uses_real_keyframes_and_private_global_output():
    from backend_launch import core_commands
    commands = core_commands('vins_fusion', '/private/lib/vins_fusion/vins_fusion_node', '/config')
    assert len(commands) == 2
    assert '/private/lib/loop_fusion/loop_fusion_node' in commands[1]
    assert '/vins_estimator/keyframe_pose:=/uav001/backends/vins_fusion/keyframe_pose' in commands[1]
    assert '/vins_estimator/keyframe_point:=/uav001/backends/vins_fusion/keyframe_point' in commands[1]
    assert 'odometry_rect:=/uav001/backends/vins_fusion/global_odometry' in commands[1]
    assert '/tf:=/uav001/backends/vins_fusion/tf' in commands[1]


def test_glim_map_dump_is_owned_by_the_experiment(tmp_path):
    import yaml
    from backend_configs import write_config
    generated=yaml.safe_load(write_config('glim',rig('livox'),tmp_path/'glim-run','test').read_text())['/**']['ros__parameters']
    assert Path(generated['dump_path']).is_relative_to(tmp_path)


def test_vins_graph_path_matches_upstream_concatenation_contract(tmp_path):
    import cv2
    from backend_configs import write_config
    write_config('vins_fusion',rig('livox'),tmp_path,'test')
    settings=cv2.FileStorage(str(tmp_path/'algorithm.yaml'),cv2.FILE_STORAGE_READ)
    directory=settings.getNode('pose_graph_save_path').string()
    assert Path(directory+'pose_graph.txt').parent == tmp_path/'pose_graph'
    assert Path(directory+'pose_graph.txt') == tmp_path/'pose_graph'/'pose_graph.txt'


def test_glim_comparison_accepts_original_synchronous_calibration(tmp_path):
    from backend_configs import write_config
    from uav_lab_experiments.backend_contract import consumed_input_group
    calibration=json.loads(Path('configs/sensors.json').read_text())
    write_config('glim',calibration,tmp_path,'baseline')
    parameters=json.loads((tmp_path/'glim/config_sensors.json').read_text())['sensors']
    assert parameters['global_shutter_lidar'] is True
    assert consumed_input_group('glim',calibration)=='synchronous_lidar_imu'
