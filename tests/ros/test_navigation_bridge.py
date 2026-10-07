import time
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.action import GoalResponse
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from std_srvs.srv import Trigger
from uav_lab_interfaces.action import ExecuteFlight
from uav_lab_bridge.node import Bridge


def ready_bridge(monkeypatch,domain):
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(domain_id=domain);bridge=Bridge();bridge.navigation_enabled=True
    now=time.monotonic();bridge.policy.update(now,position=(0.,0.,2.),yaw=0.,valid=True,armed=True,offboard=True,landed=False)
    bridge.policy.state='HOLDING';bridge.policy.streaming=True;bridge.policy.setpoint=(0.,0.,2.)
    return bridge


def test_navigation_guard_rejects_raw_motion_and_recovery_does_not_resume(monkeypatch,isolated_ros_domain):
    bridge=ready_bridge(monkeypatch,isolated_ros_domain)
    try:
        msg=DiagnosticArray();msg.status=[DiagnosticStatus(name='uav001/navigation',level=DiagnosticStatus.OK,values=[KeyValue(key=k,value=v) for k,v in {'ready':'True','source_stamp':'0','frame':'odom'}.items()])]
        bridge.navigation_callback(msg);bridge.pump()
        assert bridge.policy.streaming
        goal=ExecuteFlight.Goal();goal.operation=1;goal.target.header.frame_id='odom';goal.target.pose.orientation.w=1.
        assert bridge.goal(goal)==GoalResponse.REJECT
        goal.navigation=True
        assert bridge.goal(goal)==GoalResponse.ACCEPT
        bridge.navigation.received=time.monotonic()-2;bridge.pump()
        assert bridge.policy.state=='FAILSAFE' and not bridge.policy.streaming
        bridge.navigation_callback(msg);bridge.pump()
        assert bridge.policy.state=='FAILSAFE' and not bridge.policy.streaming
    finally:bridge.destroy_node();rclpy.shutdown()


def test_hold_epoch_invalidates_queued_navigation_leg(monkeypatch,isolated_ros_domain):
    bridge=ready_bridge(monkeypatch,isolated_ros_domain)
    try:
        bridge.navigation.observe(True,0.,'odom',time.monotonic())
        response=bridge.service_callback('hold')(Trigger.Request(),Trigger.Response())
        assert response.success and bridge.operator_generation==1
        goal=ExecuteFlight.Goal();goal.operation=1;goal.navigation=True;goal.navigation_epoch=0
        goal.target.header.frame_id='odom';goal.target.pose.orientation.w=1.
        assert bridge.goal(goal)==GoalResponse.REJECT
        goal.navigation_epoch=1
        assert bridge.goal(goal)==GoalResponse.ACCEPT
    finally:bridge.destroy_node();rclpy.shutdown()
