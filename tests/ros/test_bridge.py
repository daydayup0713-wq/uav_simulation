"""Real ROS message serialization checks; run via scripts/env.sh pytest."""
import time
import pytest
pytest.importorskip('uav_lab_interfaces.action')
import rclpy
from uav_lab_bridge.node import Bridge
from px4_msgs.msg import VehicleLocalPosition

@pytest.fixture(autouse=True)
def isolated_ros_contract_domain(monkeypatch, tmp_path):
    # Message serialization tests create fake /fmu publishers. Never allow
    # them onto the live lab domain or into its evidence directory.
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))

def initialize_test_context():
    rclpy.init(domain_id=181, args=['--ros-args', '-r', '__ns:=/bridge_contract_test'])

def test_observation_serializes_real_ros_messages_before_and_after_telemetry():
    initialize_test_context()
    bridge = Bridge()
    try:
        bridge.observe(time.monotonic())
        now = time.monotonic()
        bridge.policy.update(now, position=(1., 2., 3.), yaw=0., valid=True,
                             armed=False, offboard=False, landed=True)
        bridge.observe(now)
        assert len(bridge.path.poses) == 1
        assert bridge.path.poses[0].pose.position.x == 1.
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

def test_estimator_coordinate_reset_latches_failsafe():
    initialize_test_context()
    bridge = Bridge()
    try:
        message = VehicleLocalPosition()
        message.timestamp = 1_000_000
        message.xy_valid, message.z_valid = True, True
        bridge.position_callback(message)
        bridge.policy.streaming = True
        message.timestamp += 10_000
        message.xy_reset_counter = 1
        bridge.position_callback(message)
        assert bridge.policy.state == 'FAILSAFE'
        assert 'reset' in bridge.policy.reason
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

def test_estimator_initialization_reset_before_control_is_allowed():
    initialize_test_context()
    bridge = Bridge()
    try:
        message = VehicleLocalPosition()
        message.timestamp = 1_000_000
        bridge.position_callback(message)
        message.timestamp += 10_000
        message.xy_reset_counter = 1
        bridge.position_callback(message)
        assert bridge.policy.state != 'FAILSAFE'
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

def test_small_inflight_heading_alignment_keeps_target_but_large_reset_fails():
    initialize_test_context()
    bridge = Bridge()
    try:
        message = VehicleLocalPosition()
        message.timestamp = 1_000_000
        bridge.position_callback(message)
        bridge.policy.streaming = True
        bridge.policy.target = (1., 2., 3.)
        message.timestamp += 10_000
        message.heading_reset_counter = 1
        message.delta_heading = .003
        bridge.position_callback(message)
        assert bridge.policy.state != 'FAILSAFE'
        assert bridge.policy.target == (1., 2., 3.)
        message.timestamp += 10_000
        message.heading_reset_counter = 2
        message.delta_heading = .5
        bridge.position_callback(message)
        assert bridge.policy.state == 'FAILSAFE'
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
