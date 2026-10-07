import importlib.util
import numpy as np
import pytest


def implementation():
    assert importlib.util.find_spec('uav_lab_bridge.trajectory'), 'bounded acceleration trajectory missing'
    from uav_lab_bridge.trajectory import MotionProfile
    return MotionProfile


@pytest.mark.parametrize('length',[.05,3.])
def test_profile_bounds_speed_acceleration_and_does_not_overshoot(length):
    profile=implementation()((0,0,0),(length,0,0),.5,.5)
    times=np.linspace(0,profile.duration,1001)
    samples=[profile.sample(t) for t in times]
    positions=np.asarray([x[0] for x in samples]); velocities=np.asarray([x[1] for x in samples])
    assert positions[:,0].min()>=0 and positions[:,0].max()<=length+1e-12
    assert np.linalg.norm(velocities,axis=1).max()<=.5+1e-9
    assert np.linalg.norm(np.diff(velocities,axis=0),axis=1).max()/np.diff(times)[0]<=.5+1e-6
    assert np.allclose(positions[-1],(length,0,0))
    assert np.allclose(velocities[[0,-1]],0)


def test_zero_distance_and_invalid_profile_rejected():
    cls=implementation(); profile=cls((1,2,3),(1,2,3),.5,.5)
    assert profile.duration==0 and np.allclose(profile.sample(0)[0],(1,2,3))
    with pytest.raises(ValueError): cls((0,0,0),(np.inf,0,0),.5,.5)
    with pytest.raises(ValueError): cls((0,0,0),(1,0,0),0,.5)


def test_navigation_controller_uses_profile_while_default_behavior_stays_linear():
    from uav_lab_bridge.controller import FlightController
    assert 'acceleration' in __import__('inspect').signature(FlightController).parameters, 'profile integration missing'
    c=FlightController(speed=.5,acceleration=.5);c.state='HOLDING'
    def fresh(t): c.update(t,position=(0.,0.,2.),yaw=0.,armed=True,offboard=True,landed=False,valid=True)
    fresh(0); c.fly('GOTO',0,target=(3,0,2))
    fresh(.1);c.tick(.1,.1)
    assert c.setpoint[0]==pytest.approx(.0025)
    default=FlightController(speed=.5);default.state='HOLDING'
    default.update(0,position=(0.,0.,2.),yaw=0.,armed=True,offboard=True,landed=False,valid=True)
    default.fly('GOTO',0,target=(3,0,2));default.tick(.1,.1)
    assert default.setpoint[0]==pytest.approx(.05)
