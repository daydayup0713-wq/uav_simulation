"""CLI experiment execution through the sole flight adapter Action."""
import json
import time
import uuid
import numpy as np


def execute_route(path, timeout=600.):
    import rclpy
    from rclpy.action import ActionClient
    from uav_lab_interfaces.action import ExecuteTrajectory
    from uav_lab_interfaces.msg import TrajectoryReference
    from uav_lab_bridge.continuous_trajectory import Trajectory, State
    from uav_lab_bridge.trajectory_interface import trajectory_goal
    data = json.loads(path.read_text())
    rclpy.init()
    node = rclpy.create_node('trajectory_operator')
    reference = {'value': None, 'received': 0.}
    def observe(message):
        reference.update(value=message, received=time.monotonic())
    subscription = node.create_subscription(TrajectoryReference, '/uav001/trajectory/reference', observe, 10)
    client = ActionClient(node, ExecuteTrajectory, '/uav001/execute_trajectory')
    handle, submitted = None, None
    def wait(future, budget):
        rclpy.spin_until_future_complete(node, future, timeout_sec=budget)
        if not future.done() or future.exception():
            raise RuntimeError('trajectory confirmation timeout')
        return future.result()
    try:
        deadline = time.monotonic() + 5
        while reference['value'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        message = reference['value']
        if message is None or time.monotonic() - reference['received'] > .5:
            raise RuntimeError('fresh trajectory reference unavailable; start lab, arm and take off first')
        if message.trajectory_id and (np.linalg.norm(message.velocity) > 1e-6 or np.linalg.norm(message.acceleration) > 1e-6):
            raise RuntimeError('route requires stationary hover; current trajectory/stop still active')
        initial = State(np.array(message.position), yaw=message.yaw)
        points = [initial.position.tolist()] + data['controls']
        headings = data.get('yaws')
        if headings is not None:
            headings = [initial.yaw] + headings
        trajectory = Trajectory.generate(points, initial=initial, dwell={int(k): v for k, v in data.get('dwell', {}).items()}, yaws=headings)
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('trajectory adapter unavailable')
        submitted = client.send_goal_async(trajectory_goal(trajectory, uuid.uuid4().hex))
        handle = wait(submitted, 5)
        if not handle.accepted:
            raise RuntimeError('trajectory rejected; check ownership, ground state, gates and initial reference')
        result = wait(handle.get_result_async(), max(timeout, trajectory.duration * 3 + 20)).result
        if not result.success:
            raise RuntimeError(result.reason)
        return {'success': True, 'reason': result.reason, 'duration_s': trajectory.duration,
                'reference_maxima': trajectory.derivative_maxima()}
    except (RuntimeError, KeyboardInterrupt) as error:
        if submitted is not None:
            try:
                handle = handle or wait(submitted, 3)
                if handle.accepted:
                    response = wait(handle.cancel_goal_async(), 3)
                    if not response.goals_canceling:
                        raise RuntimeError('cancellation unconfirmed')
            except (RuntimeError, KeyboardInterrupt) as stop_error:
                raise RuntimeError(str(error) + '; trajectory cancellation unconfirmed: ' + str(stop_error)) from error
        raise RuntimeError(str(error) or 'CLI interrupted; constrained hold requested') from error
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
