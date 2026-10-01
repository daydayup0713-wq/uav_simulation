"""Real ROS message serialization checks; run via scripts/env.sh pytest."""
import time
import pytest
pytest.importorskip('uav_lab_interfaces.action')
import rclpy
from uav_lab_bridge.node import Bridge

def test_observation_serializes_real_ros_messages_before_and_after_telemetry():
    rclpy.init()
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
