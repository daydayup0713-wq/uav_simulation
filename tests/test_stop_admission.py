"""Operator braking must stay in source-current observed collision space."""
import numpy as np
import pytest
from uav_lab_bridge.controller import FlightController
from uav_lab_bridge.continuous_trajectory import State, Trajectory
from uav_lab_navigation.occupancy import CollisionMap


def moving_controller():
    controller=FlightController()
    controller.update(10.,position=(0.,0.,2.),yaw=0.,valid=True,armed=True,offboard=True,landed=False)
    controller.trajectory=Trajectory.generate([[0.,0.,2.],[2.,0.,2.]])
    controller.reference=State(np.array([0.,0.,2.]),np.array([.5,0.,0.]),np.zeros(3))
    controller.state='MOVING';controller.sim_time=10.
    return controller


def test_hold_refuses_a_stop_curve_in_blocked_or_unknown_space():
    controller=moving_controller()
    free=np.ones((50,50,50),dtype=bool)
    free[27:,:,:]=False
    collision=CollisionMap(.1,np.array([-2.5,-2.5,0.]),free,1,10.)
    assert not Trajectory.stop(controller.reference).collision_free(collision)
    def admit(trajectory,now,sim):
        if not trajectory.collision_free(collision):raise ValueError('stop curve blocked or unknown')
    controller.stop_admission=admit
    with pytest.raises(ValueError,match='blocked or unknown'):controller.hold(10.)
    assert controller.state=='FAILSAFE'
    assert controller.trajectory is None


def snapshot(**changes):
    value={'lower':[-2.5,-2.5,0.], 'resolution':.1,'shape':[50,50,50],
        'free':bytes(np.ones(125000,dtype=np.uint8)), 'envelope':[.65,.65,.55],
        'source_stamp':10.,'frame':'odom','version':1}
    value.update(changes);return value


def test_stop_admission_requires_fresh_actual_inflated_free_map():
    from uav_lab_bridge.stop_admission import StopAdmission
    gate=StopAdmission([.65,.65,.55]);trajectory=Trajectory.stop(moving_controller().reference)
    with pytest.raises(ValueError,match='map unavailable'):gate.admit(trajectory,10.,10.)
    gate.observe(snapshot(),10.)
    assert gate.admit(trajectory,10.1,10.1) is True
    with pytest.raises(ValueError,match='expired'):gate.admit(trajectory,11.,10.1)
    with pytest.raises(ValueError,match='expired'):gate.admit(trajectory,10.1,11.01)
    unknown=np.ones(125000,dtype=np.uint8).reshape((50,50,50));unknown[27:,:,:]=0
    gate.observe(snapshot(free=unknown.tobytes(),source_stamp=10.2,version=2),10.2)
    with pytest.raises(ValueError,match='blocked or unknown'):gate.admit(trajectory,10.2,10.2)


@pytest.mark.parametrize('changes',[
    {'frame':'map'}, {'shape':[500,500,500]}, {'shape':[50,50,50.5]},
    {'free':bytes([2])*125000}, {'envelope':[.1,.1,.1]},
    {'source_stamp':float('nan')}, {'resolution':0.}, {'free':b'1'},
])
def test_stop_map_rejects_malformed_uninflated_or_unbounded_input(changes):
    from uav_lab_bridge.stop_admission import StopAdmission
    gate=StopAdmission([.65,.65,.55])
    with pytest.raises(ValueError):gate.observe(snapshot(**changes),10.)


def test_stop_map_cannot_replace_current_snapshot_with_older_source():
    from uav_lab_bridge.stop_admission import StopAdmission
    gate=StopAdmission([.65,.65,.55]);gate.observe(snapshot(version=2),10.)
    assert gate.observe(snapshot(version=1,source_stamp=9.),10.1) is False
    assert gate.admit(Trajectory.stop(moving_controller().reference),10.1,10.1)
