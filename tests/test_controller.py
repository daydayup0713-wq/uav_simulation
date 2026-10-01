import math
import pytest
from uav_lab_bridge.controller import FlightController

def refresh(c, now, **kwargs):
    c.update(now, position=(0, 0, .05), yaw=0, valid=True,
             armed=False, offboard=False, landed=True, **kwargs)

def start():
    c = FlightController()
    refresh(c, 0)
    token = c.arm(0)
    for t in (.5, 1, 1.5):
        refresh(c, t)
        c.tick(t, .05)
        assert not c.drain_commands()
    refresh(c, 2.01)
    c.tick(2.01, .05)
    assert c.drain_commands() == [(176, (1., 6.))]
    return c, token

def armed():
    c, token = start()
    c.ack(176, 0)
    c.update(2.1, offboard=True)
    c.tick(2.1, .05)
    assert c.drain_commands() == [(400, (1.,))]
    c.ack(400, 0)
    c.update(2.2, armed=True)
    c.tick(2.2, .05)
    assert c.results[token][0]
    return c

def test_mode_ack_alone_does_not_arm():
    c, token = start()
    c.ack(176, 0)
    c.tick(2.1, .05)
    assert not c.drain_commands()
    assert token not in c.results

def test_arm_ack_alone_does_not_report_success():
    c, token = start()
    c.ack(176, 0)
    c.update(2.1, offboard=True)
    c.tick(2.1, .05)
    c.ack(400, 0)
    c.tick(2.2, .05)
    assert token not in c.results

@pytest.mark.parametrize('ack', [False, True])
def test_command_timeout_with_fresh_telemetry_never_arms(ack):
    c, token = start()
    if ack:
        c.ack(176, 0)
    refresh(c, 7.1)
    c.tick(7.1, .05)
    assert c.state == 'FAILSAFE'
    assert not c.streaming and not c.drain_commands()
    assert c.results[token] == (False, 'command ACK/state confirmation timeout')

def test_disarm_in_air_and_unarmed_takeoff_are_rejected():
    c = FlightController()
    refresh(c, 0)
    with pytest.raises(ValueError):
        c.fly('TAKEOFF', 0, height=2)
    c = armed()
    c.update(2.3, landed=False)
    with pytest.raises(ValueError):
        c.disarm(2.3)

def test_delayed_takeoff_uses_explicit_arm_ground_anchor():
    c = armed()
    # PX4 land detector can clear after motors spin, while still near ground.
    c.update(2.3, position=(.02, .01, .08), landed=False)
    c.fly('TAKEOFF', 2.3, height=2)
    assert c.target == (0., 0., 2.05)

def test_takeoff_from_uncommanded_airborne_location_is_rejected():
    c = armed()
    c.update(2.3, position=(0., 0., 1.), landed=False)
    with pytest.raises(ValueError, match='ground'):
        c.fly('TAKEOFF', 2.3, height=2)

def test_target_speed_and_continuous_tolerance():
    c = armed()
    token = c.fly('GOTO', 2.2, target=(3, 4, 2), yaw=0)
    c.tick(2.25, .05)
    assert math.dist((0, 0, .05), c.setpoint) <= .0500001
    for t in (2.3, 2.8, 3.3, 3.8):
        c.update(t, position=(3, 4, 2), yaw=0, valid=True, armed=True, offboard=True, landed=False)
        c.tick(t, .05)
    assert token not in c.results
    c.update(3.9, position=(4, 4, 2))
    c.tick(3.9, .05)
    for t in (4, 4.5, 5, 5.5, 6.01):
        c.update(t, position=(3, 4, 2), yaw=0, valid=True, armed=True, offboard=True, landed=False)
        c.tick(t, .05)
    for index in range(1,121):
        now = 6.01+index*.05
        c.update(now, position=(3,4,2), yaw=0, valid=True, armed=True, offboard=True, landed=False)
        c.tick(now, .05)
    assert c.results[token][0]

def test_cancel_old_goal_cannot_cancel_landing():
    c = armed()
    motion = c.fly('TAKEOFF', 2.2, height=2)
    landing = c.fly('LAND', 2.3)
    assert not c.results[motion][0]
    assert not c.cancel(motion, 2.3)
    assert not c.cancel(landing, 2.3)
    assert c.state == 'LANDING'
    c.ack(21, 0)
    c.update(2.4, armed=False, offboard=False, landed=True, landing_mode=True)
    c.tick(2.4, .05)
    assert c.results[landing][0]

def test_cancel_motion_holds_measured_pose_and_finishes_old_goal():
    c = armed()
    token = c.fly('GOTO', 2.2, target=(3., 4., 2.))
    c.update(2.3, position=(.1, .2, .3), landed=False)
    assert c.cancel(token, 2.3)
    assert c.state == 'HOLDING'
    assert c.setpoint == c.target == (.1, .2, .3)
    assert not c.results[token][0] and c.active is None

def test_stale_telemetry_latches_and_cannot_resume():
    c = armed()
    token = c.fly('TAKEOFF', 2.2, height=2)
    c.tick(4, .05)
    assert c.state == 'FAILSAFE'
    assert not c.streaming
    assert not c.results[token][0]
    refresh(c, 4.1)
    c.tick(4.1, .05)
    assert c.state == 'FAILSAFE'

def test_rejected_command_fails_without_arm():
    c, token = start()
    c.ack(176, 2)
    c.tick(2.1, .05)
    assert not c.results[token][0]
    assert not c.drain_commands()

@pytest.mark.parametrize('target', [(float('nan'), 0, 2), (0, 0, -2), (50, 0, 2)])
def test_invalid_or_outside_lab_targets_are_rejected(target):
    c = armed()
    with pytest.raises(ValueError):
        c.fly('GOTO', 2.2, target=target)
