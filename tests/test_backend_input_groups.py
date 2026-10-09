import pytest

def test_comparison_groups_follow_consumed_inputs_instead_of_rig_inventory():
    from uav_lab_experiments.backend_contract import consumed_input_group
    livox={'lidar':{'kind':'livox'}}
    assert consumed_input_group('glim',livox)=='livox_inertial'
    assert consumed_input_group('fast_lio2',livox)=='livox_inertial'
    assert consumed_input_group('fast_livo2',livox)=='livox_visual_inertial'
    assert consumed_input_group('fast_livo2_rtk',livox)=='livox_visual_inertial_rtk'
    for backend in ('vins_fusion','orb_slam3'):
        assert consumed_input_group(backend,livox)=='monocular_inertial'
    assert consumed_input_group('lio_sam',{'lidar':{'kind':'mechanical'}})=='mechanical_inertial'
    with pytest.raises(ValueError):consumed_input_group('invented',livox)


def test_mechanical_six_axis_and_attitude_inputs_are_not_ranked_together():
    from uav_lab_experiments.backend_contract import consumed_input_group
    mechanical={'lidar':{'kind':'mechanical'}}
    assert consumed_input_group('lio_sam',mechanical)=='mechanical_inertial'
    assert consumed_input_group('fast_lio2',mechanical)=='mechanical_lidar_imu'
    assert consumed_input_group('glim',mechanical)=='mechanical_lidar_imu'
    assert consumed_input_group('fast_livo2',mechanical)=='mechanical_visual_imu'
    legacy_ntu={'reference':'NTU_VIRAL upstream parameters','imu':{'xyz':[0,0,0],'rpy':[0,0,0]}}
    assert consumed_input_group('fast_lio2',legacy_ntu)=='mechanical_lidar_imu'
    assert consumed_input_group('fast_livo2',legacy_ntu)=='mechanical_visual_imu'
