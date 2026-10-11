"""Real ROS serialization and bridge trajectory ownership/feedforward checks."""
import math
import time
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.action import GoalResponse
from uav_lab_bridge.node import Bridge
from uav_lab_bridge.continuous_trajectory import Trajectory
from uav_lab_bridge.trajectory_interface import trajectory_goal
from uav_lab_interfaces.action import ExecuteTrajectory


def test_trajectory_action_rejects_raw_navigation_and_stale_epoch_and_publishes_feedforward(monkeypatch, tmp_path, isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init(domain_id=isolated_ros_domain)
    node = Bridge()
    try:
        now = time.monotonic()
        node.navigation_enabled = True
        node.navigation.observe(True, 0., 'odom', now)
        node.policy.update(now, position=(0., 0., 2.), yaw=0., valid=True, armed=True,
                           offboard=True, landed=False)
        node.policy.state = 'HOLDING'
        node.policy.setpoint = (0., 0., 2.)
        node.policy.streaming = True
        route = Trajectory.generate([(0, 0, 2), (1, 0, 2), (1, 1, 2)])
        request = trajectory_goal(route, 'native-route')
        assert node.trajectory_goal_callback(request) == GoalResponse.REJECT
        request.navigation = True
        request.navigation_epoch = 1
        assert node.trajectory_goal_callback(request) == GoalResponse.REJECT
        request.navigation_epoch = 0
        assert node.trajectory_goal_callback(request) == GoalResponse.ACCEPT
        token = node.policy.follow_trajectory(route, 'native-route', now, 0.)
        node.policy.tick(now, .02, sim_time=1.)
        message = node.setpoint_message(1_000_000)
        assert message.position == pytest.approx([node.policy.setpoint[1], node.policy.setpoint[0], -2.])
        assert message.velocity == pytest.approx([node.policy.reference.velocity[1], node.policy.reference.velocity[0], -node.policy.reference.velocity[2]])
        assert np.isfinite(message.acceleration).all()
        assert math.isfinite(message.yawspeed)
        request.segments[0].coefficients[0] += .5
        assert node.trajectory_goal_callback(request) == GoalResponse.REJECT
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_trajectory_action_serialization_rejects_limits_forged_by_client(monkeypatch, tmp_path, isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init(domain_id=isolated_ros_domain)
    node = Bridge()
    try:
        now = time.monotonic()
        node.policy.update(now, position=(0., 0., 2.), yaw=0., valid=True, armed=True, offboard=True, landed=False)
        node.policy.state = 'HOLDING'
        node.policy.setpoint = (0., 0., 2.)
        request = trajectory_goal(Trajectory.generate([(0, 0, 2), (1, 0, 2)]), 'fast')
        for segment in request.segments:
            segment.duration *= .1
        assert node.trajectory_goal_callback(request) == GoalResponse.REJECT
    finally:
        node.destroy_node()
        rclpy.shutdown()
