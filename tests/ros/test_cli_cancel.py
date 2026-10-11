"""Exercise real SIGINT/DDS cancellation without publishing flight inputs."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import pytest

pytest.importorskip('uav_lab_interfaces.action')
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from uav_lab_interfaces.action import ExecuteFlight

@pytest.mark.parametrize('operation,reason,should_cancel,response_delay,accepted', [
    ('goto', 'cancellation requested', True, 0., True),
    ('goto', 'cancellation requested', True, .8, True),
    ('land', 'landing continues', False, 0., True),
    ('land', 'landing continues', False, .8, True),
    ('land', 'landing acceptance unconfirmed', False, .8, False)])
def test_cli_sigint_preserves_motion_and_landing_semantics(monkeypatch, tmp_path, isolated_ros_domain, operation, reason, should_cancel, response_delay, accepted):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    domain = isolated_ros_domain
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('cancel_test_server')
    started, canceled, stop = threading.Event(), threading.Event(), threading.Event()
    def accept(goal):
        if goal.operation == 255:
            return GoalResponse.REJECT
        started.set()
        time.sleep(response_delay)
        return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT
    def execute(goal):
        while not goal.is_cancel_requested and not stop.is_set():
            time.sleep(.01)
        goal.canceled() if goal.is_cancel_requested else goal.abort()
        result = ExecuteFlight.Result()
        result.reason = 'test canceled'
        return result
    def cancel(goal):
        canceled.set()
        return CancelResponse.ACCEPT
    server = ActionServer(node, ExecuteFlight, '/uav001/execute_flight', execute_callback=execute,
                          goal_callback=accept, cancel_callback=cancel, callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    root = Path(__file__).resolve().parents[2]
    executable = root/'ros2_ws/install/uav_lab_tools/lib/uav_lab_tools/labctl'
    environment = {**os.environ, 'ROS_DOMAIN_ID': str(domain)}
    arguments = [operation, '--x', '3', '--y', '0', '--z', '2'] if operation == 'goto' else [operation]
    client = subprocess.Popen([str(executable), *arguments],
                              env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert started.wait(5), 'client did not start goal'
        time.sleep(.2)
        client.send_signal(signal.SIGINT)
        output, error = client.communicate(timeout=5)
        assert client.returncode == 1, error
        assert canceled.wait(1) == should_cancel, output+error
        response = json.loads(output.splitlines()[-1])
        assert reason in response['reason'], output+error
    finally:
        if client.poll() is None:
            client.kill()
            client.wait(timeout=3)
        stop.set()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()
