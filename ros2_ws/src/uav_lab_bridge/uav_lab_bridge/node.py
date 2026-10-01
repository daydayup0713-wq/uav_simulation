"""PX4 boundary adapter. The only lab component writing /fmu/in topics."""
import json
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import (VehicleLocalPosition, VehicleStatus, VehicleAttitude,
                          VehicleLandDetected, VehicleCommandAck, VehicleCommand,
                          OffboardControlMode, TrajectorySetpoint)
from uav_lab_interfaces.action import ExecuteFlight
from .controller import FlightController
from .coordinate import ned_to_enu, enu_to_ned, px4_to_ros_quaternion, yaw_to_ned
from .clock import Px4Clock

class Bridge(Node):
    def __init__(self):
        super().__init__('flight_bridge', namespace='uav001')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.lock = threading.RLock()
        self.policy = FlightController()
        self.px4_clock = Px4Clock()
        self.orientation = (0., 0., 0., 1.)
        self.velocity = (0., 0., 0.)
        self.previous_sim = None
        self.reset_counters = None
        self.last_observe = 0.
        self.last_state = None
        self.path = NavPath()
        self.path.header.frame_id = 'odom'
        self.trace = None
        run_dir = os.environ.get('LAB_RUN_DIR')
        if run_dir:
            self.trace = (Path(run_dir) / 'events.jsonl').open('a', buffering=1)
        group = ReentrantCallbackGroup()
        self.group = group
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        for name, msg, callback in [
                ('vehicle_local_position', VehicleLocalPosition, self.position_callback),
                ('vehicle_status', VehicleStatus, self.status_callback),
                ('vehicle_attitude', VehicleAttitude, self.attitude_callback),
                ('vehicle_land_detected', VehicleLandDetected, self.land_callback),
                ('vehicle_command_ack', VehicleCommandAck, self.ack_callback)]:
            version = getattr(msg, 'MESSAGE_VERSION', 0)
            topic = '/fmu/out/' + name + (f'_v{version}' if version else '')
            self.create_subscription(msg, topic, callback, qos_profile_sensor_data, callback_group=group)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.path_pub = self.create_publisher(NavPath, 'path', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.tf = TransformBroadcaster(self)
        for name in ('arm', 'disarm', 'hold'):
            self.create_service(Trigger, name, self.service_callback(name), callback_group=group)
        self.server = ActionServer(self, ExecuteFlight, 'execute_flight',
                                   execute_callback=self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=group)
        self.timer = self.create_timer(.05, self.pump, callback_group=group,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def record(self, event, **values):
        if self.trace:
            self.trace.write(json.dumps({'wall_monotonic': time.monotonic(),
                                         'sim_ns': self.get_clock().now().nanoseconds,
                                         'event': event, **values}, allow_nan=False)+'\n')

    def position_callback(self, msg):
        with self.lock:
            counters = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
            if (self.reset_counters is not None and counters != self.reset_counters
                    and (self.policy.streaming or self.policy.active is not None)):
                # PX4 re-aligns magnetic heading above 1.5m. This updates the
                # attitude estimate, not the ENU position frame. Bound it by
                # the same yaw tolerance used by target acceptance.
                heading_only = counters[:2] == self.reset_counters[:2]
                self.record('estimator_reset', previous=self.reset_counters, current=counters,
                            delta_heading=float(msg.delta_heading))
                if not (heading_only and math.isfinite(msg.delta_heading) and abs(msg.delta_heading) <= .15):
                    self.policy.fail('estimator coordinate reset; restart lab')
            self.reset_counters = counters
            try:
                self.px4_clock.observe(msg.timestamp, self.get_clock().now().nanoseconds)
            except RuntimeError as exc:
                self.policy.fail(str(exc))
            self.policy.update(time.monotonic(), position=ned_to_enu((msg.x, msg.y, msg.z)), valid=bool(msg.xy_valid and msg.z_valid))
            self.velocity = ned_to_enu((msg.vx, msg.vy, msg.vz))

    def status_callback(self, msg):
        with self.lock:
            self.policy.update(time.monotonic(), armed=msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                               offboard=msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                               preflight=bool(msg.pre_flight_checks_pass),
                               landing_mode=msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

    def attitude_callback(self, msg):
        with self.lock:
            try:
                self.orientation = px4_to_ros_quaternion(msg.q)
                x, y, z, w = self.orientation
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                self.policy.update(time.monotonic(), yaw=yaw)
            except ValueError:
                self.policy.fail('invalid attitude')

    def land_callback(self, msg):
        with self.lock:
            self.policy.update(time.monotonic(), landed=bool(msg.landed))

    def ack_callback(self, msg):
        if msg.target_system not in (0, 255) or msg.target_component not in (0, 1):
            return
        with self.lock:
            self.record('ack', command=msg.command, result=msg.result)
            self.policy.ack(msg.command, msg.result)

    def pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.policy.t.position
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = self.orientation
        return pose

    def pump(self):
        with self.lock:
            wall = time.monotonic()
            sim = self.get_clock().now().nanoseconds
            dt = 0 if self.previous_sim is None else (sim-self.previous_sim)/1e9
            if dt < 0:
                self.policy.fail('simulation clock regression')
            self.previous_sim = sim
            self.policy.tick(wall, dt)
            if self.policy.streaming:
                for topic in ('/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint', '/fmu/in/vehicle_command'):
                    if self.count_publishers(topic) > 1:
                        self.policy.fail('another PX4 writer detected: '+topic)
                        break
            try:
                stamp = self.px4_clock.timestamp(sim)
            except RuntimeError:
                stamp = 0
            if self.policy.streaming and stamp:
                mode = OffboardControlMode()
                mode.timestamp = stamp
                mode.position = True
                self.control_pub.publish(mode)
                if self.policy.t.offboard and self.policy.setpoint is not None:
                    target = TrajectorySetpoint()
                    target.timestamp = stamp
                    target.position = list(enu_to_ned(self.policy.setpoint))
                    target.velocity = [math.nan]*3
                    target.acceleration = [math.nan]*3
                    target.jerk = [math.nan]*3
                    target.yaw = yaw_to_ned(self.policy.yaw)
                    target.yawspeed = math.nan
                    self.setpoint_pub.publish(target)
            for command, params in self.policy.drain_commands():
                msg = VehicleCommand()
                msg.timestamp = stamp
                msg.command = command
                for index, value in enumerate(params, 1):
                    setattr(msg, f'param{index}', float(value))
                msg.target_system, msg.target_component = 1, 1
                msg.source_system, msg.source_component = 255, 1
                msg.from_external = True
                self.command_pub.publish(msg)
                self.record('command', command=command, params=params)
            if self.last_state != self.policy.state:
                self.last_state = self.policy.state
                self.get_logger().info(f'{self.policy.state}: {self.policy.reason}')
                self.record('state', phase=self.policy.state, reason=self.policy.reason)
            if wall-self.last_observe >= .2:
                self.last_observe = wall
                self.observe(wall)

    def observe(self, wall):
        pose = self.pose()
        if self.policy.fresh(wall):
            odom = Odometry()
            odom.header = pose.header
            odom.child_frame_id = 'base_link'
            odom.pose.pose = pose.pose
            # Odometry twist is expressed in child_frame_id (FLU), not world ENU.
            from .coordinate import multiply
            x, y, z, w = self.orientation
            _, vx, vy, vz = multiply(multiply((w, -x, -y, -z), (0., *self.velocity)), (w, x, y, z))
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = vx, vy, vz
            self.odom_pub.publish(odom)
            transform = TransformStamped()
            transform.header, transform.child_frame_id = pose.header, 'base_link'
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.policy.t.position
            transform.transform.rotation = pose.pose.orientation
            self.tf.sendTransform(transform)
            self.path.header.stamp = pose.header.stamp
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-3000:]
            self.path_pub.publish(self.path)
            self.record('telemetry', position=self.policy.t.position, yaw=self.policy.t.yaw,
                        armed=self.policy.t.armed, offboard=self.policy.t.offboard, landed=self.policy.t.landed)
        diag = DiagnosticArray()
        diag.header.stamp = pose.header.stamp
        status = DiagnosticStatus()
        status.name = 'uav001/flight'
        status.hardware_id = 'PX4-SITL-1'
        status.level = DiagnosticStatus.OK if self.policy.fresh(wall) and self.policy.state != 'FAILSAFE' else DiagnosticStatus.ERROR
        status.message = self.policy.state
        status.values = [KeyValue(key=k, value=str(v)) for k, v in {
            'reason': self.policy.reason, 'armed': self.policy.t.armed, 'offboard': self.policy.t.offboard,
            'landed': self.policy.t.landed, 'position': self.policy.t.position,
            'preflight': self.policy.t.preflight,
            'fresh': self.policy.fresh(wall)}.items()]
        diag.status = [status]
        self.diag_pub.publish(diag)

    def service_callback(self, operation):
        def callback(request, response):
            try:
                with self.lock:
                    token = getattr(self.policy, operation)(time.monotonic())
                if operation == 'hold':
                    response.success, response.message = True, 'hold commanded'
                    return response
                deadline = time.monotonic()+15
                while rclpy.ok() and time.monotonic() < deadline:
                    with self.lock:
                        result = self.policy.results.get(token)
                    if result:
                        response.success, response.message = result
                        return response
                    time.sleep(.05)
                response.success, response.message = False, 'operation timed out'
            except ValueError as exc:
                response.success, response.message = False, str(exc)
            return response
        return callback

    def goal(self, goal):
        with self.lock:
            try:
                self.policy.require_ready(time.monotonic())
                if not self.policy.t.armed or goal.operation not in (0, 1, 2):
                    return GoalResponse.REJECT
                if goal.operation != 2 and (self.policy.active is not None or not self.policy.t.offboard):
                    return GoalResponse.REJECT
                if self.policy.state == 'LANDING':
                    return GoalResponse.REJECT
                if goal.operation == 0 and (not math.isfinite(goal.height_m) or not 0 < goal.height_m <= 5):
                    return GoalResponse.REJECT
                if goal.operation == 1:
                    if goal.target.header.frame_id != 'odom':
                        return GoalResponse.REJECT
                    p, q = goal.target.pose.position, goal.target.pose.orientation
                    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                        return GoalResponse.REJECT
                    if abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>1e-3:
                        return GoalResponse.REJECT
                return GoalResponse.ACCEPT
            except ValueError:
                return GoalResponse.REJECT

    def cancel(self, handle):
        with self.lock:
            return CancelResponse.ACCEPT if self.policy.state == 'MOVING' else CancelResponse.REJECT

    def execute(self, handle):
        result = ExecuteFlight.Result()
        request = handle.request
        try:
            p, q = request.target.pose.position, request.target.pose.orientation
            with self.lock:
                token = self.policy.fly(('TAKEOFF', 'GOTO', 'LAND')[request.operation], time.monotonic(),
                                        target=(p.x, p.y, p.z), yaw=2*math.atan2(q.z, q.w) if request.operation == 1 else None,
                                        height=request.height_m)
            while rclpy.ok():
                with self.lock:
                    if handle.is_cancel_requested:
                        canceled = self.policy.cancel(token, time.monotonic())
                        if canceled:
                            handle.canceled()
                            result.success, result.reason = False, 'canceled; hold commanded'
                            return result
                    outcome = self.policy.results.get(token)
                    feedback = ExecuteFlight.Feedback()
                    feedback.phase = self.policy.state
                    feedback.current_pose = self.pose()
                if outcome:
                    result.success, result.reason = outcome
                    handle.succeed() if result.success else handle.abort()
                    return result
                handle.publish_feedback(feedback)
                time.sleep(.1)
            result.success, result.reason = False, 'bridge shutting down'
        except (ValueError, RuntimeError) as exc:
            result.success, result.reason = False, str(exc)
        handle.abort()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node.trace:
            node.trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
