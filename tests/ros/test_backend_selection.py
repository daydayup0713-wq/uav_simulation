"""The bridge lock, not a CLI pre-check, owns ground configuration changes."""
import os
import time
import pytest

pytest.importorskip('rclpy')
from uav_lab_bridge.node import Bridge
from uav_lab_interfaces.srv import SelectBackends


def test_bridge_selection_rejects_armed_and_stale_state(isolated_ros_domain, monkeypatch, tmp_path):
    import rclpy
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init(domain_id=isolated_ros_domain)
    node = Bridge()
    request = SelectBackends.Request()
    request.localization, request.planning, request.input_group = 'glim', 'astar', 'synchronous_lidar_imu'
    try:
        now = time.monotonic()
        node.policy.update(now, position=(0., 0., 0.), yaw=0., valid=True, armed=True, landed=True)
        result = node.select_backend_callback(request, SelectBackends.Response())
        assert result.success is False
        assert 'ground' in result.reason
        node.policy.update(now - 5, armed=False)
        result = node.select_backend_callback(request, SelectBackends.Response())
        assert result.success is False
        assert 'ground' in result.reason
    finally:
        node.destroy_node()
        rclpy.shutdown()
