"""PX4 boundary adapter. The only lab component writing /fmu/in topics."""
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
import hashlib
import numpy as np

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
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
                          OffboardControlMode, TrajectorySetpoint, VehicleOdometry, EstimatorStatusFlags)
from uav_lab_interfaces.action import ExecuteFlight, ExecuteTrajectory
from uav_lab_interfaces.msg import TrajectoryReference,CollisionSnapshot
from uav_lab_interfaces.srv import SelectBackends
from .controller import FlightController
from .coordinate import ned_to_enu, enu_to_ned, px4_to_ros_quaternion, yaw_to_ned
from .clock import Px4Clock
from .external_odometry import convert_external, ExternalOdometryGate
from .navigation_gate import NavigationGate

class Bridge(Node):
    def __init__(self):
        super().__init__('flight_bridge', namespace='uav001')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.lock = threading.RLock()
        self.declare_parameter('navigation_required', False)
        self.navigation_enabled = bool(self.get_parameter('navigation_required').value)
        self.navigation = NavigationGate()
        self.operator_generation = 0
        self.declare_parameter('flight_lower',[-10.,-10.,.2])
        self.declare_parameter('flight_upper',[10.,10.,5.])
        bounds=(self.get_parameter('flight_lower').value,self.get_parameter('flight_upper').value)
        self.policy = (FlightController(speed=.5, tolerance=.15, acceleration=.5,flight_bounds=bounds)
                       if self.navigation_enabled else FlightController(flight_bounds=bounds))
        self.declare_parameter('external_odometry', False)
        self.external_enabled = bool(self.get_parameter('external_odometry').value)
        self.external = ExternalOdometryGate()
        self.external_sent = None
        self.fusion_flags, self.fusion_received = {}, 0.
        self.fusion_was_ready = False
        self.nav_state = None
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
        self.run_dir=Path(run_dir) if run_dir else None
        if run_dir:
            self.trace = (Path(run_dir) / 'events.jsonl').open('a', buffering=1)
        group = ReentrantCallbackGroup()
        self.group = group
        if self.navigation_enabled:
            from .stop_admission import StopAdmission
            root=Path(os.environ.get('LAB_ROOT',Path.cwd()))
            candidate=self.run_dir/'configuration/navigation.json' if self.run_dir else None
            config_path=candidate if candidate and candidate.exists() else root/'configs/navigation.json'
            config=json.loads(config_path.read_text())
            self.stop_map=StopAdmission(np.asarray(config['body_halfsize_m'])+config['clearance_m'])
            self.policy.stop_admission=self.admit_stop
            self.create_subscription(DiagnosticArray, 'navigation/diagnostics', self.navigation_callback, 10, callback_group=group)
            from rclpy.qos import QoSProfile,ReliabilityPolicy
            self.create_subscription(CollisionSnapshot,'navigation/collision_snapshot',self.collision_callback,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT),callback_group=group)
        # The executor must not start a newer sample while an earlier telemetry
        # callback waits for CPU/the policy lock. Actions and control remain concurrent.
        self.telemetry_group = MutuallyExclusiveCallbackGroup()
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        if self.external_enabled:
            self.external_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
            self.create_subscription(Odometry, '/uav001/localization/odometry', self.external_callback, 20,
                                     callback_group=self.telemetry_group)
            self.create_subscription(DiagnosticArray, '/uav001/localization/diagnostics', self.quality_callback, 10,
                                     callback_group=self.telemetry_group)
            version = getattr(EstimatorStatusFlags, 'MESSAGE_VERSION', 0)
            self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags'+(f'_v{version}' if version else ''),
                                     self.fusion_callback, qos_profile_sensor_data, callback_group=self.telemetry_group)
        for name, msg, callback in [
                ('vehicle_local_position', VehicleLocalPosition, self.position_callback),
                ('vehicle_status', VehicleStatus, self.status_callback),
                ('vehicle_attitude', VehicleAttitude, self.attitude_callback),
                ('vehicle_land_detected', VehicleLandDetected, self.land_callback),
                ('vehicle_command_ack', VehicleCommandAck, self.ack_callback)]:
            version = getattr(msg, 'MESSAGE_VERSION', 0)
            topic = '/fmu/out/' + name + (f'_v{version}' if version else '')
            self.create_subscription(msg, topic, callback, qos_profile_sensor_data, callback_group=self.telemetry_group)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.path_pub = self.create_publisher(NavPath, 'path', 10)
        self.reference_pub = self.create_publisher(TrajectoryReference, 'trajectory/reference', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.tf = TransformBroadcaster(self)
        for name in ('arm', 'disarm', 'hold'):
            self.create_service(Trigger, name, self.service_callback(name), callback_group=group)
        self.create_service(SelectBackends, 'experiments/select_backends', self.select_backend_callback,
                            callback_group=group)
        self.server = ActionServer(self, ExecuteFlight, 'execute_flight',
                                   execute_callback=self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=group)
        self.trajectory_server = ActionServer(self, ExecuteTrajectory, 'execute_trajectory',
                                    execute_callback=self.execute_trajectory,
                                    goal_callback=self.trajectory_goal_callback,
                                    cancel_callback=lambda handle: CancelResponse.ACCEPT,
                                    callback_group=group)
        self.timer = self.create_timer(.02, self.pump, callback_group=self.telemetry_group,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.heartbeat_timer = self.create_timer(.05, self.heartbeat, callback_group=group,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def select_backend_callback(self, request, response):
        from uav_lab_experiments.registry import Registry, GroundState, select_backends
        try:
            with self.lock:
                now = time.monotonic()
                # Both the selection and arm service hold the same policy lock.
                if not self.policy.fresh(now):
                    raise ValueError('selection requires fresh ground telemetry')
                root = Path(os.environ['LAB_ROOT'])
                registry = Registry(root / 'configs/backends.json', root / '.runtime/backend-evidence')
                select_backends(registry, root / '.runtime/next-experiment.json',
                                request.localization, request.planning, request.input_group,
                                GroundState(self.policy.t.armed, self.policy.t.landed,
                                            self.policy.active is None and not self.policy.streaming,
                                            self.policy.t.status_at), now)
                self.record('backend_selection', localization=request.localization,
                            planning=request.planning, input_group=request.input_group)
                response.success, response.reason = True, 'saved for next run; restart lab required'
        except (ValueError, KeyError, OSError) as exc:
            response.success, response.reason = False, str(exc)
        return response

    def record(self, event, **values):
        if self.trace:
            self.trace.write(json.dumps({'wall_monotonic': time.monotonic(),
                                         'sim_ns': self.get_clock().now().nanoseconds,
                                         'event': event, **values}, allow_nan=False)+'\n')

    def external_callback(self, msg):
        with self.lock:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            v, w = msg.twist.twist.linear, msg.twist.twist.angular
            self.external.observe_sample({'source_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
                'position': [p.x,p.y,p.z], 'quaternion': [q.x,q.y,q.z,q.w],
                'body_velocity': [v.x,v.y,v.z], 'body_angular_velocity': [w.x,w.y,w.z],
                'pose_covariance': list(msg.pose.covariance), 'twist_covariance': list(msg.twist.covariance)}, time.monotonic())

    def collision_callback(self, msg):
        with self.lock:
            try:
                self.stop_map.observe({'lower':list(msg.lower),'resolution':msg.resolution,'shape':list(msg.shape),
                    'free':msg.free,'envelope':list(msg.envelope),'version':msg.version,
                    'source_stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                    'frame':msg.header.frame_id},time.monotonic())
            except (ValueError,TypeError,OverflowError) as error:
                self.stop_map.snapshot=None
                self.record('stop_map_rejected',reason=str(error))

    def admit_stop(self, trajectory, now, sim):
        self.stop_map.admit(trajectory,now,sim)
        snapshot=self.stop_map.snapshot;artifact={}
        if self.run_dir:
            try:
                directory=self.run_dir/'stop-admission';directory.mkdir(exist_ok=True)
                path=directory/(uuid.uuid4().hex+'.npz')
                with path.open('xb') as stream:
                    np.savez_compressed(stream,free=snapshot.free,lower=snapshot.lower,resolution=snapshot.resolution,
                        source_stamp=snapshot.source_stamp,envelope=snapshot.envelope,map_version=snapshot.version)
                artifact={'map_artifact':str(path.relative_to(self.run_dir)),
                    'map_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
            except OSError as error:raise ValueError('stop map archive failed: '+str(error)) from error
        self.record('stop_admission',trajectory=trajectory.to_dict(),source_stamp=snapshot.source_stamp,
            map_version=snapshot.version,**artifact)
        return True

    def navigation_callback(self, msg):
        with self.lock:
            for status in msg.status:
                if status.name == 'uav001/navigation':
                    values = {v.key: v.value for v in status.values}
                    try:source = float(values['source_stamp'])
                    except (ValueError, KeyError):source = math.nan
                    self.navigation.observe(status.level == DiagnosticStatus.OK and values.get('ready') == 'True', source, values.get('frame'), time.monotonic())

    def navigation_ready(self, now):
        return not self.navigation_enabled or self.navigation.ready(now, self.get_clock().now().nanoseconds/1e9)

    def quality_callback(self, msg):
        with self.lock:
            for status in msg.status:
                if status.name == 'uav001/localization':
                    values = {v.key: v.value for v in status.values}
                    self.external.observe_quality(status.level == DiagnosticStatus.OK and values.get('ready') == 'True', time.monotonic())

    def fusion_callback(self, msg):
        with self.lock:
            self.fusion_flags = {k: bool(getattr(msg,k)) for k in
                ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_gnss_pos','cs_gnss_vel','cs_gps_hgt','cs_gnss_yaw')}
            self.fusion_received = time.monotonic()
            self.record('fusion', **self.fusion_flags)

    def external_ready(self, now):
        return (not self.external_enabled or (self.external.ready(now) and now-self.fusion_received < 1.5 and
                all(self.fusion_flags.get(k) is True for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw')) and
                all(self.fusion_flags.get(k) is False for k in ('cs_gnss_pos','cs_gnss_vel','cs_gps_hgt','cs_gnss_yaw'))))


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
                self.record('clock_rejected', previous_px4_us=self.px4_clock.px4_us,
                            previous_ros_ns=self.px4_clock.ros_ns, received_px4_us=int(msg.timestamp),
                            observed_ros_ns=self.get_clock().now().nanoseconds)
                self.policy.fail(str(exc))
            self.policy.update(time.monotonic(), position=ned_to_enu((msg.x, msg.y, msg.z)), valid=bool(msg.xy_valid and msg.z_valid))
            self.velocity = ned_to_enu((msg.vx, msg.vy, msg.vz))

    def status_callback(self, msg):
        with self.lock:
            self.nav_state = msg.nav_state
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
            if self.navigation_enabled and self.policy.streaming and self.policy.state != 'LANDING' and not self.navigation_ready(wall):
                self.policy.fail(self.navigation.failed or 'required navigation map unavailable')
            if self.external_enabled:
                if self.external_ready(wall):
                    self.fusion_was_ready = True
                elif self.fusion_was_ready:
                    self.external.failed = self.external.failed or 'required external EKF fusion lost/stale or GNSS fusion enabled; restart required'
                if self.external.ready(wall) and self.external.sample['source_ns'] != self.external_sent and self.external.sample['source_ns'] <= sim:
                    try:
                        values = convert_external(self.external.sample, self.px4_clock, sim)
                        msg = VehicleOdometry()
                        for key, value in values.items(): setattr(msg, key, value)
                        if self.count_publishers('/fmu/in/vehicle_visual_odometry') != 1:
                            raise ValueError('another external odometry writer detected')
                        self.external_pub.publish(msg)
                        self.external_sent = self.external.sample['source_ns']
                        self.record('external_odometry', source_ns=self.external_sent,
                                    position=values['position'], sample_us=values['timestamp_sample'])
                    except (ValueError, RuntimeError) as error:
                        # Wait for the first clock anchor before fusion starts.
                        if self.px4_clock.px4_us is not None:
                            self.external.failed = self.external.failed or str(error)
                if self.external.failed:
                    self.policy.fail(self.external.failed)
            self.policy.tick(wall, dt, sim_time=sim/1e9)
            if self.policy.fresh(wall):
                self.reference_pub.publish(self.reference_message())
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
                if self.policy.t.offboard and self.policy.setpoint is not None:
                    self.setpoint_pub.publish(self.setpoint_message(stamp))
                    if self.policy.trajectory is not None:
                        self.record('trajectory_sample', trajectory_id=self.policy.trajectory_id,
                                    elapsed=sim/1e9-self.policy.trajectory_started,
                                    position=self.policy.setpoint, velocity=self.policy.reference.velocity.tolist(),
                                    acceleration=self.policy.reference.acceleration.tolist(),
                                    jerk=self.policy.reference.jerk.tolist(), yaw=self.policy.yaw)
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
            'external_ready': self.external_ready(wall), 'fusion': self.fusion_flags, 'nav_state': self.nav_state,
            'fresh': self.policy.fresh(wall), 'navigation_ready': self.navigation_ready(wall),
            'operator_generation': self.operator_generation}.items()]
        diag.status = [status]
        self.diag_pub.publish(diag)

    def heartbeat(self):
        with self.lock:
            if not self.policy.streaming:
                return
            try:
                stamp = self.px4_clock.timestamp(self.get_clock().now().nanoseconds)
            except RuntimeError:
                return
            mode = OffboardControlMode(timestamp=stamp, position=True)
            self.control_pub.publish(mode)

    def setpoint_message(self, stamp):
        target = TrajectorySetpoint(timestamp=stamp)
        target.position = list(enu_to_ned(self.policy.setpoint))
        reference = self.policy.reference
        target.velocity = list(enu_to_ned(reference.velocity)) if reference is not None else [math.nan]*3
        target.acceleration = list(enu_to_ned(reference.acceleration)) if reference is not None else [math.nan]*3
        target.jerk = [math.nan]*3
        target.yaw = yaw_to_ned(self.policy.yaw)
        target.yawspeed = -reference.yaw_rate if reference is not None else math.nan
        return target

    def reference_message(self):
        msg = TrajectoryReference()
        msg.header.frame_id = 'odom'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(round(self.policy.sim_time * 1e9)), 10**9)
        msg.trajectory_id = self.policy.trajectory_id
        reference = self.policy.reference
        if reference is not None:
            msg.elapsed = max(0., self.policy.sim_time - self.policy.trajectory_started)
            msg.position, msg.velocity, msg.acceleration = [list(map(float, v)) for v in
                (reference.position, reference.velocity, reference.acceleration)]
            msg.yaw, msg.yaw_rate = reference.yaw, reference.yaw_rate
        else:
            msg.position = list(map(float, self.policy.setpoint or self.policy.t.position))
            msg.yaw = float(self.policy.yaw)
        return msg

    def trajectory_guard(self, request):
        now = time.monotonic()
        self.policy.require_ready(now)
        if not self.external_ready(now) or not self.navigation_ready(now):
            raise ValueError('validated localization and navigation required')
        if self.navigation_enabled and (not request.navigation or request.navigation_epoch != self.operator_generation):
            raise ValueError('stale navigation owner/epoch')
        if not self.policy.t.armed or not self.policy.t.offboard or self.policy.state not in ('MOVING', 'HOLDING'):
            raise ValueError('trajectory requires armed Offboard vehicle')

    def trajectory_goal_callback(self, request):
        from .trajectory_interface import trajectory_from_goal
        try:
            trajectory = trajectory_from_goal(request)
            with self.lock:
                self.trajectory_guard(request)
                if request.replaces_id:
                    if request.replaces_id != self.policy.trajectory_id or self.policy.trajectory_pending is not None:
                        return GoalResponse.REJECT
                elif self.policy.active is not None:
                    return GoalResponse.REJECT
                start = trajectory.sample(0.)
                if not request.replaces_id and math.dist(start.position, self.policy.setpoint or self.policy.t.position) > 1e-6:
                    return GoalResponse.REJECT
            return GoalResponse.ACCEPT
        except (ValueError, TypeError, KeyError):
            return GoalResponse.REJECT

    def execute_trajectory(self, handle):
        from .trajectory_interface import trajectory_from_goal
        result = ExecuteTrajectory.Result()
        token = None
        try:
            trajectory = trajectory_from_goal(handle.request)
            with self.lock:
                self.trajectory_guard(handle.request)
                stamp = handle.request.header.stamp
                start = stamp.sec + stamp.nanosec / 1e9
                if start == 0:
                    start = self.policy.sim_time
                token = self.policy.follow_trajectory(trajectory, handle.request.trajectory_id,
                        time.monotonic(), start, replaces=handle.request.replaces_id)
                self.record('trajectory_accepted', token=token, trajectory_id=handle.request.trajectory_id,
                            replaces_id=handle.request.replaces_id, starts_at=start, trajectory=trajectory.to_dict())
            while rclpy.ok():
                with self.lock:
                    if handle.is_cancel_requested and self.policy.cancel(token, time.monotonic()):
                        handle.canceled()
                        return ExecuteTrajectory.Result(success=False, reason='canceled; constrained hold commanded')
                    outcome = self.policy.results.get(token)
                    feedback = ExecuteTrajectory.Feedback(phase=self.policy.state, current_pose=self.pose(),
                                                          reference=self.reference_message())
                if outcome:
                    result.success, result.reason = outcome
                    handle.succeed() if result.success else handle.abort()
                    return result
                handle.publish_feedback(feedback)
                time.sleep(.05)
            result.reason = 'bridge shutting down'
        except (ValueError, RuntimeError) as exc:
            result.reason = str(exc)
        handle.abort()
        return result

    def service_callback(self, operation):
        def callback(request, response):
            try:
                with self.lock:
                    if operation == 'arm' and not self.external_ready(time.monotonic()):
                        raise ValueError('validated external localization and no GNSS fusion required')
                    if operation == 'arm' and not self.navigation_ready(time.monotonic()):
                        raise ValueError('fresh observed navigation map required')
                    token = getattr(self.policy, operation)(time.monotonic())
                    if operation == 'hold':self.operator_generation += 1
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
                if not self.external_ready(time.monotonic()):
                    return GoalResponse.REJECT
                if goal.operation != 2 and not self.navigation_ready(time.monotonic()):
                    return GoalResponse.REJECT
                if self.navigation_enabled and goal.operation == 1 and (not goal.navigation or goal.navigation_epoch != self.operator_generation):
                    return GoalResponse.REJECT
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
                if self.navigation_enabled and request.operation == 1 and (not request.navigation or request.navigation_epoch != self.operator_generation):
                    raise ValueError('navigation interrupted by operator; stale leg rejected')
                if request.operation != 2 and not self.navigation_ready(time.monotonic()):
                    raise ValueError('required navigation map unavailable')
                token = self.policy.fly(('TAKEOFF', 'GOTO', 'LAND')[request.operation], time.monotonic(),
                                        target=(p.x, p.y, p.z), yaw=2*math.atan2(q.z, q.w) if request.operation == 1 else None,
                                        height=request.height_m)
                if request.operation == 2:self.operator_generation += 1
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

    def destroy_node(self):
        self.server.destroy()
        self.trajectory_server.destroy()
        return super().destroy_node()

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
