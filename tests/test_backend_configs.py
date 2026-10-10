import json
from pathlib import Path
import numpy as np
import pytest

pytestmark = pytest.mark.usefixtures('pinned_backend_config_root')


def test_livo_camera_and_lidar_extrinsics_come_from_declared_rig():
    from backend_configs import simulation_parameters
    calibration=json.loads(Path('configs/sensors-livox.json').read_text())
    parameters,camera=simulation_parameters('fast_livo2',calibration,'test')
    assert parameters['common']['imu_topic']=='/uav001/imu/data'
    assert parameters['time_offset']['img_time_offset']==0
    assert camera['cam_fx']==pytest.approx(320)
    assert camera['cam_width']==640 and camera['cam_height']==480
    assert np.array(parameters['extrin_calib']['Rcl']).reshape(3,3)@np.array([1,0,0])==pytest.approx([0,0,1])
    assert parameters['extrin_calib']['Pcl']==pytest.approx([0,-.1,-.2])


def test_new_backends_reject_synchronous_lidar_and_missing_rtk_contract():
    from backend_configs import simulation_parameters
    calibration=json.loads(Path('configs/sensors.json').read_text())
    with pytest.raises(ValueError,match='timed'):simulation_parameters('fast_livo2',calibration,'test')
    calibration=json.loads(Path('configs/sensors-livox.json').read_text())
    with pytest.raises(ValueError,match='GNSS'):simulation_parameters('fast_livo2_rtk',calibration,'test')


def test_fastlio_local_map_cannot_delete_map_while_stationary():
    from backend_configs import simulation_parameters
    calibration=json.loads(Path('configs/sensors-livox.json').read_text())
    parameters,_=simulation_parameters('fast_lio2',calibration,'test')
    # Upstream moves its cube whenever either face is <=1.5*DET_RANGE away.
    # Both faces must be outside that region on initialization.
    assert parameters['cube_side_length']/2>1.5*parameters['mapping']['det_range']


def test_fastlio_mechanical_uses_real_ring_and_seconds_contract():
    from backend_configs import simulation_parameters
    calibration=json.loads(Path('configs/sensors-mechanical.json').read_text())
    parameters,_=simulation_parameters('fast_lio2',calibration,'test')
    assert parameters['preprocess']['lidar_type']==2
    assert parameters['preprocess']['scan_line']==16
    assert parameters['preprocess']['timestamp_unit']==0
    assert parameters['common']['lid_topic'].endswith('/points')


def test_public_ntu_clip_declares_end_of_scan_timing_for_each_backend():
    from backend_configs import public_ntu_parameters
    params,camera,calibration=public_ntu_parameters('fast_lio2','fixed-clip')
    assert params['preprocess']['lidar_type']==3
    assert params['preprocess']['scan_line']==16
    assert params['common']['time_offset_lidar_to_imu']==pytest.approx(-.1)
    assert calibration['pose_time_offset_s']['fast_lio2']==pytest.approx(-.1)
    assert calibration['scan_end_offset_s']==0
    assert params['mapping']['extrinsic_T']==pytest.approx([-.05,0,.055])
    assert not params['pcd_save']['pcd_save_en']
    params,camera,calibration=public_ntu_parameters('fast_livo2','fixed-clip')
    assert params['time_offset']['lidar_time_offset']==pytest.approx(-.1)
    assert params['common']['img_topic']=='/uav001/camera/image_raw'
    assert camera['cam_width']==752 and camera['cam_height']==480
