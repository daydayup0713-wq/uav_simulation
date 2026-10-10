import json
from pathlib import Path
import os
import pytest
rclpy=pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry

ROOT=Path(__file__).resolve().parents[2]


def test_rtk_pose_callback_publishes_typed_diagnostics(tmp_path, isolated_ros_domain, monkeypatch):
    from sensor_model import prepare_sensors
    from uav_lab_experiments.timed_sensor_node import TimedSensors
    config=prepare_sensors(ROOT,tmp_path,sensor_profile='livox-rtk')['calibration_path']
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(args=['--ros-args','-p','calibration_file:='+str(config)],domain_id=isolated_ros_domain)
    node=TimedSensors()
    try:
        msg=Odometry();msg.header.stamp.sec=1;msg.header.frame_id='sim_world'
        msg.child_frame_id='truth_base_link';msg.pose.pose.orientation.w=1.
        node.pose(msg)
        assert node.gnss_next>1.
        assert not node.failure
        assert len(node.poses.samples)==1
    finally:
        node.destroy_node();rclpy.shutdown()
