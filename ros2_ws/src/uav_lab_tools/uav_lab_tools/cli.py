import json
import math
import os
from pathlib import Path
import time
from .arguments import parser

class Operator:
    def __init__(self, node, timeout):
        from rclpy.action import ActionClient
        from uav_lab_interfaces.action import ExecuteFlight
        from diagnostic_msgs.msg import DiagnosticArray
        self.node, self.timeout = node, timeout
        self.action_type = ExecuteFlight
        self.action = ActionClient(node, ExecuteFlight, '/uav001/execute_flight')
        self.latest = None
        self.latest_at = 0.
        self.samples = []
        self.sampling = False
        self.goal_handle = None
        self.subscription = node.create_subscription(DiagnosticArray, '/uav001/diagnostics', self.on_status, 10)

    def on_status(self, msg):
        if msg.status:
            status = msg.status[0]
            self.latest = {'phase': status.message, **{v.key: v.value for v in status.values}}
            self.latest_at = time.monotonic()

    def spin(self, duration=.1):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=duration)

    def wait(self, future, timeout=None):
        deadline = time.monotonic()+(timeout or self.timeout)
        while not future.done() and time.monotonic() < deadline:
            self.spin()
        if not future.done():
            raise RuntimeError('operation timeout')
        if future.exception():
            raise RuntimeError(str(future.exception()))
        return future.result()

    def status(self):
        deadline = time.monotonic()+min(self.timeout, 10)
        while (self.latest is None or time.monotonic()-self.latest_at > 1) and time.monotonic() < deadline:
            self.spin()
        if self.latest is None or time.monotonic()-self.latest_at > 1:
            raise RuntimeError('bridge diagnostic stream unavailable on /uav001/diagnostics; '
                               'start ./scripts/start_lab.sh --rviz (or --headless) in another terminal '
                               'and wait for LAB READY; if already started, check its output and use '
                               'the same LAB_DOMAIN_ID in both terminals')
        return self.latest

    def service(self, name):
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, '/uav001/'+name)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError('service unavailable: '+name)
            response = self.wait(client.call_async(Trigger.Request()))
            if not response.success:
                raise RuntimeError(response.message)
            return {'success': True, 'reason': response.message}
        finally:
            self.node.destroy_client(client)

    def flight(self, operation, height=2., target=(0., 0., 2.), yaw=0.):
        if not self.action.wait_for_server(timeout_sec=5):
            raise RuntimeError('flight action unavailable')
        goal = self.action_type.Goal()
        goal.operation = {'TAKEOFF': 0, 'GOTO': 1, 'LAND': 2}[operation]
        goal.height_m = float(height)
        goal.target.header.frame_id = 'odom'
        goal.target.pose.position.x, goal.target.pose.position.y, goal.target.pose.position.z = map(float, target)
        goal.target.pose.orientation.z = math.sin(yaw/2)
        goal.target.pose.orientation.w = math.cos(yaw/2)
        def feedback(message):
            if self.sampling:
                pose = message.feedback.current_pose.pose.position
                self.samples.append((time.monotonic(), (pose.x, pose.y, pose.z)))
        submitted = self.action.send_goal_async(goal, feedback_callback=feedback)
        handle = None
        try:
            handle = self.wait(submitted)
            if not handle.accepted:
                raise RuntimeError('flight goal rejected')
            self.goal_handle = handle
            response = self.wait(handle.get_result_async())
        except (RuntimeError, KeyboardInterrupt) as interrupted:
            prefix = 'CLI interrupted' if isinstance(interrupted, KeyboardInterrupt) else str(interrupted)
            if operation == 'LAND':
                if isinstance(interrupted, KeyboardInterrupt):
                    try:
                        if handle is None:
                            handle = self.wait(submitted, 3)
                        if not handle.accepted:
                            raise RuntimeError('flight goal was rejected')
                    except (RuntimeError, KeyboardInterrupt) as error:
                        raise RuntimeError(prefix+'; landing acceptance unconfirmed: '+str(error)) from interrupted
                    raise RuntimeError(prefix+'; landing continues') from interrupted
                raise
            # The server may have accepted the goal before the response reaches
            # us. Keep its future/context alive and include that window in cleanup.
            deadline = time.monotonic()+3
            try:
                if handle is None:
                    handle = self.wait(submitted, max(.001, deadline-time.monotonic()))
                if handle.accepted:
                    canceled = self.wait(handle.cancel_goal_async(), max(.001, deadline-time.monotonic()))
                    if not canceled.goals_canceling:
                        raise RuntimeError('server did not confirm cancellation')
                elif isinstance(interrupted, KeyboardInterrupt):
                    raise RuntimeError('flight goal was rejected')
            except (RuntimeError, KeyboardInterrupt) as error:
                raise RuntimeError(prefix+'; motion cancellation unconfirmed: '+str(error)) from interrupted
            if isinstance(interrupted, KeyboardInterrupt):
                raise RuntimeError(prefix+'; motion cancellation requested') from interrupted
            raise
        finally:
            self.goal_handle = None
        if not response.result.success:
            raise RuntimeError(response.result.reason)
        return {'success': True, 'reason': response.result.reason}

    def hover(self, seconds, target):
        from nav_msgs.msg import Odometry
        samples = []
        subscription = self.node.create_subscription(Odometry, '/uav001/odometry',
            lambda msg: samples.append((time.monotonic(), math.dist((msg.pose.pose.position.x,
                 msg.pose.pose.position.y, msg.pose.pose.position.z), target))), 10)
        start = time.monotonic()
        try:
            while time.monotonic()-start < seconds:
                self.spin()
            if len(samples) < seconds*3 or any(error > .3 for _, error in samples):
                raise RuntimeError('hover tolerance/frequency acceptance failed')
            return {'seconds': seconds, 'samples': len(samples), 'maximum_error_m': max(e for _, e in samples)}
        finally:
            self.node.destroy_subscription(subscription)

    def demo(self, runs, hover):
        import ast
        if runs < 1:
            raise RuntimeError('--runs must be positive')
        reports = []
        # /odom is tied to the fixed PX4 local origin, so return to the same start.
        for run in range(runs):
            status = self.status()
            if status.get('fresh') != 'True' or status.get('armed') != 'False':
                raise RuntimeError('demo requires fresh disarmed vehicle')
            origin = ast.literal_eval(status['position'])
            height = origin[2]+2
            started = time.monotonic()
            self.service('arm')
            try:
                self.flight('TAKEOFF', height=2)
                report = self.hover(hover, (origin[0], origin[1], height))
                for dx, dy in ((3, 0), (3, 3), (0, 3), (0, 0)):
                    self.flight('GOTO', target=(origin[0]+dx, origin[1]+dy, height))
                self.flight('LAND')
            except BaseException:
                try:
                    self.flight('LAND')
                except RuntimeError:
                    pass
                raise
            report.update(run=run+1, elapsed_s=time.monotonic()-started, landed=True, disarmed=True)
            reports.append(report)
            print(json.dumps(report), flush=True)
            deadline = time.monotonic()+3
            while time.monotonic()<deadline:
                self.spin()
        directory = Path(os.environ.get('LAB_RUN_DIR', os.environ.get('LAB_ROOT', '.')))
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'acceptance.json').write_text(json.dumps({'passed': True, 'runs': reports}, indent=2)+'\n')
        return {'success': True, 'runs': len(reports)}

def main(args=None):
    options = parser().parse_args(args)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    # Keep the context alive while the flight client sends SIGINT cancellation.
    # The standard rclpy handler otherwise shuts DDS down before that exchange.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('lab_operator')
    operator = Operator(node, options.timeout)
    try:
        if options.command == 'status':
            result = operator.status()
            if result.get('fresh') != 'True' or result['phase'] == 'FAILSAFE':
                raise RuntimeError(json.dumps(result))
        elif options.command in ('arm', 'disarm', 'hold'):
            result = operator.service(options.command)
        elif options.command == 'demo':
            result = operator.demo(options.runs, options.hover)
        else:
            result = operator.flight(options.command.upper(), height=getattr(options, 'height', 2),
                                     target=(getattr(options, 'x', 0), getattr(options, 'y', 0), getattr(options, 'z', 2)),
                                     yaw=math.radians(getattr(options, 'yaw', 0)))
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except KeyboardInterrupt:
        # No motion has been submitted when interruption reaches this handler.
        print(json.dumps({'success': False, 'reason': 'CLI interrupted'}), flush=True)
        return 1
    except (RuntimeError, ValueError) as exc:
        print(json.dumps({'success': False, 'reason': str(exc)}, ensure_ascii=False), flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
