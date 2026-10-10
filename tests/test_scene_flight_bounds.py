import pytest
from uav_lab_bridge.controller import FlightController
from uav_lab_bridge.continuous_trajectory import Trajectory


def airborne(bounds=None):
    controller=FlightController(flight_bounds=bounds) if bounds else FlightController()
    controller.update(10.,position=(0.,0.,2.),yaw=0.,valid=True,armed=True,offboard=True,landed=False)
    controller.state='HOLDING';controller.setpoint=(0.,0.,2.)
    return controller


def test_archived_scene_bounds_permit_long_routes_and_reject_outside_targets_and_curves():
    bounds=([-2.,-4.,.2],[62.,4.,5.])
    controller=airborne(bounds)
    controller.follow_trajectory(Trajectory.generate([[0.,0.,2.],[60.,0.,2.]]),'long',10.,0.)
    controller=airborne(bounds)
    with pytest.raises(ValueError,match='bounds'):
        controller.follow_trajectory(Trajectory.generate([[0.,0.,2.],[63.,0.,2.]]),'outside',10.,0.)
    with pytest.raises(ValueError,match='bounds'):
        controller.fly('GOTO',10.,target=(63.,0.,2.))


def test_legacy_bounds_and_invalid_or_unbounded_scene_configuration():
    with pytest.raises(ValueError,match='bounds'):
        airborne().fly('GOTO',10.,target=(11.,0.,2.))
    for bounds in [([0.,0.,0.],[0.,2.,5.]),([-2.,-2.,.2],[10000.,2.,5.]),([-2.,-2.,float('nan')],[2.,2.,5.])]:
        with pytest.raises(ValueError,match='bounds'):airborne(bounds)


def test_constrained_hold_cannot_command_a_curve_outside_archived_flight_bounds():
    import numpy as np
    from uav_lab_bridge.continuous_trajectory import State
    controller=airborne(([-2.,-2.,.2],[1.,2.,5.]))
    controller.trajectory=Trajectory.generate([[0.,0.,2.],[.99,0.,2.]])
    controller.reference=State(np.array([.9,0.,2.]),np.array([.5,0.,0.]),np.zeros(3))
    controller.state='MOVING'
    with pytest.raises(ValueError,match='bounds'):controller.hold(10.)
    assert controller.state=='FAILSAFE'
