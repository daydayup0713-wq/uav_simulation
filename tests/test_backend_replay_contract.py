import numpy as np
import pytest
from uav_lab_experiments.backend_contract import normalized_pose, input_topics, trajectory_report


def test_imu_pose_to_body_and_clock_validation():
    row = normalized_pose(1., [1., 2., 3.], [0., 0., 0., 1.],
                          {'xyz': [.1, .2, .3], 'rpy': [0., 0., 0.]})
    assert row == pytest.approx([1., .9, 1.8, 2.7, 0., 0., 0., 1.])
    for stamp in (0., float('nan')):
        with pytest.raises(ValueError):normalized_pose(stamp, [1.,2.,3.], [0.,0.,0.,1.], {'xyz':[0,0,0],'rpy':[0,0,0]})
    with pytest.raises(ValueError):normalized_pose(1., [1.,2.,3.], [0.,0.,0.,0.], {'xyz':[0,0,0],'rpy':[0,0,0]})


def test_algorithm_topics_never_contain_truth_or_px4():
    assert '/uav001/gnss/fix' in input_topics('fast_livo2_rtk')
    assert '/uav001/gnss/fix' not in input_topics('fast_livo2')
    assert all('ground_truth' not in x and '/fmu' not in x for x in input_topics('fast_livo2_rtk'))


def test_missing_pose_is_failure_and_position_only_reference_has_no_attitude_claim():
    truth=np.array([[t,t,0,0,0,0,0,1] for t in np.arange(1.,4.,.1)])
    failed=trajectory_report([],truth,[1.,2.,3.])
    assert not failed['passed'] and failed['reason']=='no valid trajectory'
    report=trajectory_report(truth,truth,truth[:,0],attitude_valid=False)
    assert report['passed'] and report['metrics']['attitude_rmse_deg'] is None
    assert report['metrics']['rpe_rotation_rmse_deg'] is None
    assert report['metrics']['rpe_translation_rmse_m'] is None
    assert report['metrics']['rpe_translation_world_rmse_m']==pytest.approx(0.)


def test_absent_evaluation_reference_is_a_reported_failure():
    estimate=np.array([[t,t,0,0,0,0,0,1] for t in np.arange(1.,4.,.1)])
    result=trajectory_report(estimate,[],estimate[:,0])
    assert not result['passed'] and result['metrics'] is None
    assert result['reason']=='no valid independent reference'
