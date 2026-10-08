"""Catch midpoint stops, p/v/a discontinuities and limits missed by sampling."""
import math
import numpy as np
import pytest

from uav_lab_bridge.continuous_trajectory import Trajectory, Limits, Segment, State
from uav_lab_navigation.occupancy import CollisionMap


def test_three_waypoint_turn_flies_through_and_has_c2_joins():
    trajectory = Trajectory.generate([(0., 0., 2.), (2., 0., 2.), (2., 2., 2.)])
    first, second = trajectory.segments
    left, right = first.sample(first.duration), second.sample(0.)
    assert np.linalg.norm(left.velocity) > .05
    for attribute in ['position', 'velocity', 'acceleration']:
        assert np.allclose(getattr(left, attribute), getattr(right, attribute), atol=1e-9)
    assert np.allclose(trajectory.sample(0).velocity, [0, 0, 0], atol=1e-9)
    assert np.allclose(trajectory.sample(trajectory.duration).acceleration, [0, 0, 0], atol=1e-9)
    extrema = trajectory.derivative_maxima()
    assert extrema['speed'] <= .5 + 1e-8
    assert extrema['acceleration'] <= .5 + 1e-8
    assert extrema['jerk'] <= 1. + 1e-8


def test_replan_preserves_initial_position_velocity_acceleration():
    state = State(np.array([0., 0., 2.]), np.array([.2, .05, 0.]), np.array([.05, 0., 0.]))
    trajectory = Trajectory.generate([state.position, (2., 1., 2.), (3., 2., 2.)], initial=state)
    start = trajectory.sample(0.)
    for attribute in ['position', 'velocity', 'acceleration']:
        assert np.allclose(getattr(start, attribute), getattr(state, attribute), atol=1e-9)
    assert trajectory.derivative_maxima()['speed'] <= .5 + 1e-8


def test_explicit_stop_has_a_dwell_but_default_knots_do_not():
    trajectory = Trajectory.generate([(0., 0., 2.), (1., 0., 2.), (1., 1., 2.)], dwell={1: 3.})
    pause = trajectory.segments[1]
    assert pause.duration == 3.
    sample = pause.sample(1.5)
    assert np.allclose(sample.position, [1, 0, 2])
    assert np.linalg.norm(sample.velocity) < 1e-10
    assert np.linalg.norm(sample.acceleration) < 1e-10


def test_analytic_speed_extremum_between_coarse_samples():
    # x(u) = u - u²/2: speed decreases linearly, maximum 1 at u=0.
    segment = Segment(1., np.array([[0., 1., -.5, 0, 0, 0], [0]*6, [2, 0, 0, 0, 0, 0]]))
    assert segment.maximum_derivative(1) == pytest.approx(1.)
    # x(u)=u²/2-u³/3; v=u-u² maximized at u=.5, not endpoints.
    segment = Segment(1., np.array([[0, 0, .5, -1/3, 0, 0], [0]*6, [2, 0, 0, 0, 0, 0]]))
    assert segment.maximum_derivative(1) == pytest.approx(.25)


@pytest.mark.parametrize('points', [[], [(0, 0, 2)], [(0, 0, 2), (math.nan, 1, 2)],
                                   [(0, 0, 2), (0, 0, 2)], [(0, 2), (1, 2)]])
def test_invalid_or_stationary_routes_rejected(points):
    with pytest.raises(ValueError):
        Trajectory.generate(points)


def test_nonfinite_sampling_and_impossible_initial_limits_rejected():
    trajectory = Trajectory.generate([(0, 0, 2), (1, 0, 2)])
    with pytest.raises(ValueError):
        trajectory.sample(math.nan)
    with pytest.raises(ValueError, match='initial'):
        Trajectory.generate([(0, 0, 2), (1, 0, 2)], initial=State(np.array([0., 0., 2.]),
            np.array([1., 0., 0.]), np.zeros(3)))
    with pytest.raises(ValueError):
        Trajectory.generate([(0, 0, 2), (1, 0, 2)], limits=Limits(speed=0.))


def test_curve_crossing_obstacle_is_rejected_even_when_endpoints_clear():
    free = np.ones((40, 40, 40), dtype=bool)
    free[15:20, :, :] = False
    collision = CollisionMap(.1, np.array([-1., -1., -1.]), free, 1)
    trajectory = Trajectory.generate([(0, 0, 2), (2, 0, 2)])
    assert not trajectory.collision_free(collision)
    free[:] = True
    assert trajectory.collision_free(collision)
    free[20, 10, 30] = False
    assert not trajectory.collision_free(collision)


def test_unknown_bounds_and_curve_overshoot_fail_closed():
    free = np.ones((30, 30, 40), dtype=bool)
    collision = CollisionMap(.1, np.array([0., 0., 0.]), free, 1)
    trajectory = Trajectory.generate([(1, .05, 2), (2, .05, 2), (2, 2, 2)])
    # Minimum-jerk corner rounds outside y>=0 before reaching the middle point.
    assert not trajectory.collision_free(collision)


def test_serialized_segments_reject_noncontinuous_or_out_of_limit_curve():
    trajectory = Trajectory.generate([(0, 0, 2), (1, 0, 2), (1, 1, 2)])
    payload = trajectory.to_dict()
    recovered = Trajectory.from_dict(payload)
    assert np.allclose(recovered.sample(2).position, trajectory.sample(2).position)
    payload['segments'][1]['coefficients'][0][0] += .1
    with pytest.raises(ValueError, match='continuous'):
        Trajectory.from_dict(payload)


def test_constrained_stop_keeps_initial_state_and_ends_at_rest_within_limits():
    initial = State(np.array([1., 2., 2.]), np.array([.4, 0., 0.]), np.zeros(3))
    trajectory = Trajectory.stop(initial)
    assert np.allclose(trajectory.sample(0).velocity, [.4, 0, 0])
    assert np.allclose(trajectory.sample(0).position, [1, 2, 2])
    assert np.linalg.norm(trajectory.sample(trajectory.duration).velocity) < 1e-9
    assert trajectory.derivative_maxima()['speed'] <= .5 + 1e-9
    assert trajectory.derivative_maxima()['acceleration'] <= .5 + 1e-9
    assert trajectory.derivative_maxima()['jerk'] <= 1. + 1e-9
    stationary = Trajectory.stop(State(np.array([0., 0., 2.])))
    assert np.allclose(stationary.sample(1).position, [0, 0, 2])
