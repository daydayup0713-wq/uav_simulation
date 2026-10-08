"""Actual ROS Action transport: the new mode sends one C2 curve, not stop legs."""
import time
import threading
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from uav_lab_interfaces.action import Navigate, ExecuteTrajectory
from uav_lab_interfaces.msg import TrajectoryReference
from uav_lab_navigation.node import NavigationNode
from uav_lab_navigation.cli import pose
from uav_lab_navigation.planner import Plan
from uav_lab_bridge.trajectory_interface import trajectory_from_goal


@pytest.mark.parametrize('mode', ['success', 'source_loss', 'operator_hold', 'cancel', 'reject'])
def test_continuous_navigation_sends_one_collision_checked_curve(monkeypatch, tmp_path, isolated_ros_domain, mode):
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init(domain_id=isolated_ros_domain)
    nav = NavigationNode()
    nav.continuous_enabled = True
    nav.grid.score[:] = -4
    nav.grid.seen[:] = True
    nav.current = pose([0, 0, 2]).pose
    nav.current_at = time.monotonic()
    nav.flight_status = {'phase': 'HOLDING', 'armed': 'True', 'offboard': 'True', 'operator_generation': '0'}
    nav.flight_at = time.monotonic()
    healthy = [True]
    monkeypatch.setattr(nav, 'report', lambda **kw: {'ready': healthy[0],
                        'state': 'READY' if healthy[0] else 'FAILED', 'reason': 'controlled source'})
    monkeypatch.setattr(nav, 'make_plan', lambda goal, start_override=None:
        (nav.grid.snapshot(nav.envelope), Plan(True, 'PLANNED', [[0., 0., 2.], [0., 1., 2.], [1., 1., 2.]])))
    reference = TrajectoryReference(position=[0., 0., 2.], velocity=[0., 0., 0.], acceleration=[0., 0., 0.])
    reference.header.frame_id = 'odom'
    nav.on_reference(reference)
    adapter = rclpy.create_node('continuous_flight_double')
    client_node = rclpy.create_node('continuous_navigation_test')
    requests = []
    def execute(handle):
        requests.append(handle.request)
        trajectory = trajectory_from_goal(handle.request)
        midpoint = trajectory.segments[0].sample(trajectory.segments[0].duration)
        assert np.linalg.norm(midpoint.velocity) > .05
        if mode != 'success':
            if mode == 'source_loss':
                healthy[0] = False
            if mode == 'operator_hold':
                nav.flight_status['operator_generation'] = '1'
            deadline = time.monotonic() + 2
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(.01)
            if handle.is_cancel_requested:
                handle.canceled()
                return ExecuteTrajectory.Result(success=False, reason='cancel confirmed')
        handle.succeed()
        return ExecuteTrajectory.Result(success=True, reason='transport confirmed')
    server = ActionServer(adapter, ExecuteTrajectory, '/uav001/execute_trajectory',
                          execute_callback=execute, callback_group=ReentrantCallbackGroup(),
                          goal_callback=lambda r: GoalResponse.REJECT if mode == 'reject' else GoalResponse.ACCEPT,
                          cancel_callback=lambda h: CancelResponse.ACCEPT)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (nav, adapter, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, Navigate, '/uav001/navigation/navigate')
    def wait(future):
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert future.done()
        return future.result()
    try:
        assert client.wait_for_server(timeout_sec=5)
        handle = wait(client.send_goal_async(Navigate.Goal(goal=pose([1, 1, 2]))))
        assert handle.accepted
        if mode == 'cancel':
            wait(handle.cancel_goal_async())
        result = wait(handle.get_result_async())
        if mode == 'success':
            assert result.result.success
        else:
            assert not result.result.success
        assert len(requests) <= 1
        if requests:
            assert requests[0].navigation and requests[0].navigation_epoch == 0
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(3)
        client.destroy()
        server.destroy()
        for node in (client_node, adapter, nav):
            node.destroy_node()
        rclpy.shutdown()
