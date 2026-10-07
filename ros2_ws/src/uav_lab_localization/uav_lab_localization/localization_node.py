"""Normalize GLIM's world-frame twist; expose only validated continuous LIO."""
import copy
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu, PointCloud2, PointField
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .benchmark import stamp, cloud_xyz
from .quality import LocalizationQuality, normalize_pose
from .maps import load_map, relocalize, map_change_allowed
from .frames import pose_matrix, control_frame_alignment
from .ingress import validated_scan, validate_imu, RejectedScan


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization', namespace='uav001')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.quality = LocalizationQuality()
        self.latest = None
        self.latest_scan = None
        self.loaded_map = None
        self.map_to_odom = None
        self.lio_to_control = None
        self.live_map_to_odom = None
        self.flight_status, self.flight_received = {}, 0.
        root = Path(os.environ.get('LAB_ROOT', Path.cwd()))
        default_calibration = str(Path(os.environ['LAB_RUN_DIR'])/'configuration/calibration.json') if os.environ.get('LAB_RUN_DIR') else str(root/'configs/sensors.json')
        self.declare_parameter('calibration_file', default_calibration)
        self.calibration = json.loads(Path(self.get_parameter('calibration_file').value).read_text())
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        from uav_lab_interfaces.srv import LoadMap, Relocalize
        self.map_group = MutuallyExclusiveCallbackGroup()
        self.create_service(LoadMap, 'localization/load_map', self.load_map_service, callback_group=self.map_group)
        self.create_service(Relocalize, 'localization/relocalize', self.relocalize_service, callback_group=self.map_group)
        self.create_subscription(DiagnosticArray, '/uav001/diagnostics', self.flight, 10)
        self.create_subscription(Odometry, '/uav001/odometry', self.control_pose, 10)
        self.create_subscription(TFMessage, '/uav001/localization/raw_tf', self.transforms, 20)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.control_alignment_pub = self.create_publisher(TransformStamped, 'localization/control_alignment', map_qos)
        self.map_pub = self.create_publisher(PointCloud2, 'localization/map', map_qos)
        self.create_subscription(PointCloud2, '/glim_ros/map', self.map_cloud, map_qos)
        extrinsic = TransformStamped()
        extrinsic.header.frame_id = 'lio_base_link'; extrinsic.child_frame_id = 'lio_lidar'
        extrinsic.transform.translation.x, extrinsic.transform.translation.y, extrinsic.transform.translation.z = map(float, self.calibration['lidar']['xyz'])
        q = Rotation.from_euler('xyz', self.calibration['lidar']['rpy']).as_quat()
        extrinsic.transform.rotation.x, extrinsic.transform.rotation.y, extrinsic.transform.rotation.z, extrinsic.transform.rotation.w = q
        self.static_tf.sendTransform(extrinsic)
        self.path = NavPath()
        self.path.header.frame_id = 'lio_odom'
        self.pub = self.create_publisher(Odometry, 'localization/odometry', 20)
        self.path_pub = self.create_publisher(NavPath, 'localization/path', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'localization/diagnostics', 10)
        self.imu_input = self.create_publisher(Imu, 'localization/input/imu', qos_profile_sensor_data)
        self.points_input = self.create_publisher(PointCloud2, 'localization/input/points', qos_profile_sensor_data)
        self.rejected_scans = 0
        self.create_subscription(Imu, '/uav001/imu/data', self.imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/uav001/lidar/points', self.points, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/glim_ros/odom_corrected', self.odometry, 20)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.trace = None
        if os.environ.get('LAB_RUN_DIR'):
            self.trace = (Path(os.environ['LAB_RUN_DIR'])/'localization.jsonl').open('a', buffering=1)

    def imu(self, msg):
        try:
            validate_imu(msg)
            if self.quality.observe_source('imu', stamp(msg)/1e9, time.monotonic()):
                self.imu_input.publish(msg)
        except ValueError as error:
            self.quality.fail(str(error))

    def points(self, msg):
        try:
            points = validated_scan(msg)
            if self.quality.observe_source('points', stamp(msg)/1e9, time.monotonic()):
                self.latest_scan = (stamp(msg)/1e9, points)
                self.points_input.publish(msg)
        except RejectedScan as error:
            self.rejected_scans += 1
            if self.trace:
                self.trace.write(json.dumps({'event':'scan_rejected', 'stamp':stamp(msg)/1e9, 'reason':str(error)})+'\n')
        except ValueError as error:
            self.quality.fail(str(error))

    def odometry(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        if msg.header.frame_id != 'lio_odom' or msg.child_frame_id != 'lio_base_link':
            self.quality.fail('unexpected LIO coordinate frame')
            return
        try:
            values = normalize_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], [v.x, v.y, v.z])
            if not self.quality.observe_pose(stamp(msg)/1e9, values['position'], values['quaternion'], time.monotonic()):
                return
        except ValueError as error:
            self.quality.fail(str(error))
            return
        output = copy.deepcopy(msg)
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = values['body_velocity']
        # GLIM v1.1 does not publish marginal covariance. These are declared
        # observation-noise floors for EKF fusion, not a claimed precision estimate.
        output.pose.covariance = np.diag([.01, .01, .01, .01, .01, .01]).ravel().tolist()
        output.twist.covariance = np.diag([.04, .04, .04, .01, .01, .01]).ravel().tolist()
        self.latest = output
        report = self.quality.check(self.get_clock().now().nanoseconds/1e9, time.monotonic())
        if report['ready']:
            self.pub.publish(output)
            pose = PoseStamped(header=output.header, pose=output.pose.pose)
            self.path.header = output.header
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-6000:]
            self.path_pub.publish(self.path)
        if self.trace:
            self.trace.write(json.dumps({'stamp': stamp(msg)/1e9, 'position': values['position'].tolist(),
                                         'quaternion': values['quaternion'].tolist(), **report})+'\n')

    def flight(self, msg):
        for status in msg.status:
            if status.name == 'uav001/flight':
                self.flight_status = {v.key: v.value for v in status.values}
                self.flight_received = time.monotonic()

    def transforms(self, msg):
        for item in msg.transforms:
            if item.header.frame_id == 'lio_odom' and item.child_frame_id == 'lio_base_link':
                self.tf.sendTransform(item)
            elif item.header.frame_id == 'lio_map' and item.child_frame_id == 'lio_odom' and self.loaded_map is None:
                item = copy.deepcopy(item); item.header.frame_id = 'map'
                t = item.transform
                self.live_map_to_odom = np.eye(4)
                self.live_map_to_odom[:3, :3] = Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
                self.live_map_to_odom[:3, 3] = [t.translation.x,t.translation.y,t.translation.z]
                self.tf.sendTransform(item)

    def control_pose(self, msg):
        # This is a frame adapter after the estimator, not an algorithm input.
        # Freeze once on the ground; EKF/LIO drift never moves control targets.
        if self.lio_to_control is not None or self.latest is None or msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link': return
        if not map_change_allowed(self.flight_status, self.flight_received, time.monotonic()): return
        if not self.quality.check(self.get_clock().now().nanoseconds/1e9, time.monotonic())['ready']: return
        if abs(stamp(msg)-stamp(self.latest))/1e9 > .15: return
        self.lio_to_control = control_frame_alignment(pose_matrix(self.latest.pose.pose), pose_matrix(msg.pose.pose))
        if self.trace: self.trace.write(json.dumps({'event':'control_frame_aligned','lio_to_control':self.lio_to_control.tolist()})+'\n')

    def map_cloud(self, msg):
        if self.loaded_map is None:
            output = copy.deepcopy(msg); output.header.frame_id = 'map'; self.map_pub.publish(output)

    def publish_loaded_map(self):
        values = np.asarray(self.loaded_map['points'], dtype='<f4')
        msg = PointCloud2(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(values)
        msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1) for i,name in enumerate('xyz')]
        msg.point_step, msg.row_step = 12, len(values)*12
        msg.is_dense = True; msg.data = values.tobytes()
        self.map_pub.publish(msg)

    def load_map_service(self, request, response):
        try:
            if not map_change_allowed(self.flight_status, self.flight_received, time.monotonic()):
                raise ValueError('map changes require fresh landed disarmed vehicle')
            loaded = load_map(request.directory, self.calibration)
            if not map_change_allowed(self.flight_status, self.flight_received, time.monotonic()):
                raise ValueError('vehicle state changed during map load')
            self.loaded_map, self.map_to_odom = loaded, None
            self.publish_loaded_map()
            response.success, response.reason = True, 'map loaded; coarse-prior relocalization required'
        except (ValueError, OSError) as error:
            response.success, response.reason = False, str(error)
        return response

    def relocalize_service(self, request, response):
        try:
            if not map_change_allowed(self.flight_status, self.flight_received, time.monotonic()):
                raise ValueError('relocalization requires fresh landed disarmed vehicle')
            if self.loaded_map is None or self.latest_scan is None or self.latest is None:
                raise ValueError('loaded map and live LIO/scan required')
            quality = self.quality.check(self.get_clock().now().nanoseconds/1e9, time.monotonic())
            if not quality['ready'] or abs(self.latest_scan[0]-stamp(self.latest)/1e9) > .15:
                raise ValueError('fresh synchronized scan and localization required')
            scan = self.latest_scan
            odometry = self.latest
            loaded_map = self.loaded_map
            if request.initial_body_pose.header.frame_id != 'map':
                raise ValueError('initial pose must use map frame')
            extrinsic = np.eye(4)
            extrinsic[:3, :3] = Rotation.from_euler('xyz', self.calibration['lidar']['rpy']).as_matrix()
            extrinsic[:3, 3] = self.calibration['lidar']['xyz']
            result = relocalize(loaded_map, scan[1],
                                pose_matrix(request.initial_body_pose.pose) @ extrinsic,
                                pose_matrix(odometry.pose.pose) @ extrinsic)
            if not map_change_allowed(self.flight_status, self.flight_received, time.monotonic()) or not self.quality.check(self.get_clock().now().nanoseconds/1e9, time.monotonic())['ready']:
                raise ValueError('vehicle/localization state changed during relocalization')
            self.map_to_odom = result['map_to_odom']
            response.map_to_odom = self.map_transform()
            response.success, response.reason = True, 'geometric relocalization accepted; continuous odom preserved'
            response.inlier_fraction, response.rmse_m = result['inlier_fraction'], result['rmse_m']
            self.tf.sendTransform(response.map_to_odom)
            if self.trace:
                self.trace.write(json.dumps({'event': 'relocalized', 'map_to_odom': self.map_to_odom.tolist(),
                                             'rmse_m': result['rmse_m'], 'inlier_fraction': result['inlier_fraction']})+'\n')
        except (ValueError, OSError) as error:
            response.success, response.reason = False, str(error)
        return response

    def map_transform(self, matrix=None, child='lio_odom'):
        matrix = self.map_to_odom if matrix is None else matrix
        transform = TransformStamped()
        transform.header.frame_id = 'map'; transform.child_frame_id = child
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = matrix[:3, 3]
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = q
        return transform

    def tick(self):
        if self.lio_to_control is not None:
            alignment = self.map_transform(self.lio_to_control, 'odom')
            alignment.header.frame_id = 'lio_odom'
            self.control_alignment_pub.publish(alignment)
        report = self.quality.check(self.get_clock().now().nanoseconds/1e9, time.monotonic())
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='uav001/localization', hardware_id='GLIM-CPU',
                                  level=DiagnosticStatus.OK if report['ready'] else DiagnosticStatus.ERROR if report['reason'] else DiagnosticStatus.WARN,
                                  message=report['state'])
        status.values = [KeyValue(key=k, value=str(v)) for k, v in report.items()]
        status.values.append(KeyValue(key='rejected_scans', value=str(self.rejected_scans)))
        status.values.append(KeyValue(key='covariance_source', value='configured EKF observation-noise floors; no marginal covariance'))
        array.status = [status]
        self.diag_pub.publish(array)
        if self.map_to_odom is not None:
            self.tf.sendTransform(self.map_transform())
        matrix = self.map_to_odom if self.loaded_map is not None else self.live_map_to_odom
        if matrix is not None and self.lio_to_control is not None:
            self.tf.sendTransform(self.map_transform(matrix @ self.lio_to_control, 'odom'))

    def destroy_node(self):
        if self.trace:
            self.trace.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
