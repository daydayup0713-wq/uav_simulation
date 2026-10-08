"""Catch early success, unsafe replacement and discontinuous operator stops."""
import numpy as np
import pytest
from uav_lab_bridge.controller import FlightController
from uav_lab_bridge.continuous_trajectory import Trajectory, State


def hover(now=10.):
    controller = FlightController(tolerance=.15)
    controller.update(now, position=(0., 0., 2.), yaw=0., valid=True, armed=True,
                      offboard=True, landed=False)
    controller.state = 'HOLDING'
    controller.setpoint = controller.target = (0., 0., 2.)
    controller.streaming = True
    return controller


def advance(controller, wall, sim):
    controller.update(wall, position=controller.setpoint, yaw=controller.yaw, valid=True,
                      armed=True, offboard=True, landed=False)
    controller.tick(wall, .02, sim_time=sim)


def test_timed_execution_does_not_finish_at_midpoint_and_finishes_after_final_settle():
    controller = hover()
    trajectory = Trajectory.generate([(0, 0, 2), (1, 0, 2), (1, 1, 2)])
    token = controller.follow_trajectory(trajectory, 'route-1', 10., 0.)
    for sim in np.arange(.02, trajectory.duration + .02, .02):
        advance(controller, 10. + sim, sim)
        if sim < trajectory.duration:
            assert token not in controller.results
    assert np.linalg.norm(controller.reference.velocity) < 1e-8
    for sim in np.arange(trajectory.duration + .02, trajectory.duration + 2.2, .02):
        advance(controller, 10. + sim, sim)
    assert controller.results[token][0] is True


def test_duplicate_motion_rejected_and_future_replan_joins_the_expected_state():
    controller = hover()
    first = Trajectory.generate([(0, 0, 2), (2, 0, 2)])
    token = controller.follow_trajectory(first, 'first', 10., 0.)
    with pytest.raises(ValueError, match='busy'):
        controller.follow_trajectory(first, 'duplicate', 10., 0.)
    advance(controller, 11., 1.)
    join = first.sample(1.2)
    replacement = Trajectory.generate([join.position, (2., 1., 2.)], initial=join)
    next_token = controller.follow_trajectory(replacement, 'second', 11., 1.2, replaces='first')
    assert controller.active == token
    advance(controller, 11.2, 1.2)
    assert controller.active == next_token
    assert controller.results[token][0] is False
    for attribute in ('position', 'velocity', 'acceleration'):
        assert np.allclose(getattr(controller.reference, attribute), getattr(join, attribute), atol=1e-7)


def test_replan_with_wrong_initial_derivatives_or_unknown_owner_rejected():
    controller = hover()
    route = Trajectory.generate([(0, 0, 2), (2, 0, 2)])
    controller.follow_trajectory(route, 'first', 10., 0.)
    with pytest.raises(ValueError, match='owner'):
        controller.follow_trajectory(route, 'next', 10., .2, replaces='unknown')
    with pytest.raises(ValueError, match='continuous'):
        controller.follow_trajectory(route, 'next', 10., .2, replaces='first')


def test_hold_cancels_pending_replan_and_follows_a_constrained_stop():
    controller = hover()
    trajectory = Trajectory.generate([(0, 0, 2), (2, 0, 2)])
    token = controller.follow_trajectory(trajectory, 'first', 10., 0.)
    advance(controller, 11., 1.)
    join = trajectory.sample(1.2)
    replacement = Trajectory.generate([join.position, (2, 1, 2)], initial=join)
    pending = controller.follow_trajectory(replacement, 'second', 11., 1.2, replaces='first')
    reference = controller.reference
    controller.hold(11.)
    assert controller.results[token][0] is False and controller.results[pending][0] is False
    assert controller.state == 'HOLDING'
    assert np.allclose(controller.trajectory.sample(0).velocity, reference.velocity)
    assert np.allclose(controller.trajectory.sample(0).acceleration, reference.acceleration)
    assert controller.trajectory.derivative_maxima()['speed'] <= .5 + 1e-7
    with pytest.raises(ValueError, match='stop'):
        controller.fly('GOTO', 11., target=(1., 0., 2.))


def test_land_overrides_trajectory_and_stale_telemetry_latches_failure():
    controller = hover()
    route = Trajectory.generate([(0, 0, 2), (2, 0, 2)])
    token = controller.follow_trajectory(route, 'first', 10., 0.)
    landing = controller.fly('LAND', 10.1)
    assert not controller.results[token][0] and controller.active == landing
    assert controller.trajectory is None
    assert not controller.cancel(landing, 10.2)
    controller = hover()
    controller.follow_trajectory(route, 'first', 10., 0.)
    controller.tick(12., .02, sim_time=.02)
    assert controller.state == 'FAILSAFE' and not controller.streaming


def test_scheduled_trajectory_reserves_motion_until_it_starts():
    controller = hover()
    route = Trajectory.generate([(0, 0, 2), (2, 0, 2)])
    controller.follow_trajectory(route, 'first', 10., .2)
    with pytest.raises(ValueError, match='busy'):
        controller.fly('GOTO', 10., target=(1, 0, 2))
