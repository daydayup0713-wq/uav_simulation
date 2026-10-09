import numpy as np
import pytest
from uav_lab_experiments.rtk_posterior import normalize_posterior


def test_posterior_antenna_lever_arm_and_global_correction_are_separate_from_local_pose():
    # The global body is translated (10,20,30) from the local frame, with yaw90.
    q=np.array([0.,0.,np.sqrt(.5),np.sqrt(.5)])
    local=np.array([[t,1+t,2,3,*q] for t in [1.,2.,3.]])
    antenna=local.copy();antenna[:,1:4]+=np.array([10.,20.2,30.])
    before=local.copy()
    result=normalize_posterior(local,antenna,{'imu':{'rpy':[0.,0.,0.]},'gnss':{'xyz':[.2,0.,0.]}})
    assert result['body_trajectory'][:,1:4]==pytest.approx(local[:,1:4]+[10.,20.,30.])
    assert result['map_to_local'][:3,3]==pytest.approx([10.,20.,30.])
    assert result['map_to_local'][:3,:3]==pytest.approx(np.eye(3))
    assert np.array_equal(local,before)
    assert result['source_time_s']==3.


def test_posterior_requires_a_matching_local_pose_in_source_time():
    rows=np.array([[t,t,0,0,0,0,0,1] for t in [1.,2.,3.]])
    shifted=rows.copy();shifted[:,0]+=10
    with pytest.raises(ValueError,match='association'):
        normalize_posterior(rows,shifted,{'imu':{'rpy':[0.,0.,0.]},'gnss':{'xyz':[0.,0.,.1]}})
