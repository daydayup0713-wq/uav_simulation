"""Continuous local pose normalization; global corrections never enter this node."""
import copy,json,os,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2,Imu,Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,TransformStamped
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from .backend_contract import normalized_pose
from .algorithm_inputs import livox_records
from .localization_quality import LocalizationQuality
from .localization_frames import pose_matrix,control_frame_alignment


def source_seconds(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9


class BackendNormalizer(Node):
    def __init__(self):
        super().__init__('backend_normalizer',namespace='uav001')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.backend=self.declare_parameter('backend','fast_livo2').value
        if self.backend not in ('glim','fast_lio2','fast_livo2','fast_livo2_rtk','lio_sam','orb_slam3','vins_fusion'):raise ValueError('unknown backend')
        self.active=self.declare_parameter('active',False).value
        self.calibration=json.loads(Path(self.declare_parameter('calibration','').value).read_text())
        self.prefix='/uav001/backends/'+self.backend
        frame_prefix='lio' if self.active else self.backend
        self.world,self.body=frame_prefix+'_odom',frame_prefix+'_base_link'
        self.visual_only=self.backend in ('orb_slam3','vins_fusion')
        self.uses_camera=self.backend in ('fast_livo2','fast_livo2_rtk','orb_slam3','vins_fusion')
        self.pose_sensor=self.calibration['lidar'] if self.backend=='lio_sam' else self.calibration['imu']
        self.quality=LocalizationQuality(source_limits={'imu':.25,'camera':.25} if self.visual_only else None)
        self.previous=self.latest=None;self.camera=None
        self.flight_state={};self.flight_received=0.;self.alignment=None
        self.create_subscription(Odometry,self.prefix+'/raw_odometry',self.pose,qos_profile_sensor_data)
        self.create_subscription(String,self.prefix+'/core_state',self.core_state,10)
        self.create_subscription(Imu,'/uav001/imu/data',self.imu,qos_profile_sensor_data)
        if not self.visual_only:self.create_subscription(PointCloud2,'/uav001/lidar/points',self.points,qos_profile_sensor_data)
        if self.uses_camera:self.create_subscription(Image,'/uav001/camera/image_raw',self.image,qos_profile_sensor_data)
        self.pub=self.create_publisher(Odometry,self.prefix+'/odometry',20)
        self.diag=self.create_publisher(DiagnosticArray,self.prefix+'/diagnostics',10)
        self.registered=self.create_publisher(PointCloud2,self.prefix+'/registered',qos_profile_sensor_data)
        self.create_subscription(PointCloud2,self.prefix+'/registered_points',self.cloud,qos_profile_sensor_data)
        self.path=NavPath();self.path_pub=self.create_publisher(NavPath,self.prefix+'/path_normalized',10)
        self.active_pub=self.active_diag=self.alignment_pub=None
        if self.active:
            self.active_pub=self.create_publisher(Odometry,'/uav001/localization/odometry',20)
            self.active_diag=self.create_publisher(DiagnosticArray,'/uav001/localization/diagnostics',10)
            self.alignment_pub=self.create_publisher(TransformStamped,'/uav001/localization/control_alignment',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(DiagnosticArray,'/uav001/diagnostics',self.flight,10)
            self.create_subscription(Odometry,'/uav001/odometry',self.control_pose,10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        if self.active:
            identity=TransformStamped();identity.header.frame_id='map';identity.child_frame_id=self.world
            identity.transform.rotation.w=1.;self.static.sendTransform(identity)
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def imu(self,msg):
        vectors=[getattr(getattr(msg,name),axis) for name in ('linear_acceleration','angular_velocity') for axis in 'xyz']
        if msg.header.frame_id!=self.calibration['imu']['frame'] or not np.isfinite(vectors).all():self.quality.fail('invalid inertial source');return
        self.quality.observe_source('imu',source_seconds(msg),time.monotonic())

    def core_state(self,msg):
        try:
            state=json.loads(msg.data)
            if not isinstance(state,dict):raise ValueError('core state must be an object')
            if 'continuity_failed' in state and not isinstance(state['continuity_failed'],bool):
                raise ValueError('continuity status must be boolean')
            if state.get('continuity_failed'):
                self.quality.fail('local continuity failed: '+str(state.get('reason','unknown')))
        except (ValueError,TypeError) as error:self.quality.fail('invalid backend core state: '+str(error))

    def points(self,msg):
        if msg.header.frame_id!=self.calibration['lidar']['frame']:self.quality.fail('invalid point cloud source frame');return
        try:
            lidar=self.calibration['lidar'];mechanical=lidar.get('kind')=='mechanical'
            if self.backend=='glim' and lidar.get('measurement_time')!='per_beam':
                from uav_lab_localization.ingress import validated_scan
                validated_scan(msg)
                measured=source_seconds(msg)
            else:
                start,records=livox_records(msg,lidar['vertical_samples'] if mechanical else 4,'ring' if mechanical else 'line')
                if not len(records):raise ValueError('point cloud has no finite measured returns')
                measured=(start+int(records['offset_ns'].max()))/1e9
        except ValueError as error:self.quality.fail(str(error));return
        self.quality.observe_source('points',measured,time.monotonic())

    def image(self,msg):
        stamp=source_seconds(msg)
        if (msg.header.frame_id!=self.calibration['camera']['optical_frame'] or stamp<=0 or
            self.camera and stamp<=self.camera[0]):self.quality.fail('invalid camera source time/frame');return
        self.camera=(stamp,time.monotonic())
        if self.visual_only:self.quality.observe_source('camera',stamp,self.camera[1])

    def report(self):
        clock=self.get_clock().now().nanoseconds/1e9;now=time.monotonic()
        if self.uses_camera and self.quality.was_ready:
            if self.camera is None or now-self.camera[1]>1.5 or not -.15<=clock-self.camera[0]<=.25:self.quality.fail('visual source stale or simulation paused')
        result=self.quality.check(clock,now)
        result['clock_s']=clock
        for kind,source in self.quality.sources.items():
            result[kind+'_age_s']=clock-source[0]
            result[kind+'_arrival_age_s']=now-source[1]
        result['pose_arrival_age_s']=None if self.quality.pose is None else now-self.quality.pose[1]
        if self.uses_camera and self.camera is None and result['state']!='FAILED':result.update(ready=False,state='INITIALIZING')
        return result

    def pose(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        try:
            row=normalized_pose(source_seconds(msg),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],self.pose_sensor)
            expected=(self.backend+'_odom',('imu_link',)) if self.visual_only or self.backend=='glim' else ('lio_sam_odom',('odom_mapping',)) if self.backend=='lio_sam' else ('camera_init',('body','aft_mapped'))
            if msg.header.frame_id!=expected[0] or msg.child_frame_id not in expected[1]:raise ValueError('unexpected estimator pose frame')
            if not self.quality.observe_pose(row[0],row[1:4],row[4:],time.monotonic()):return
        except ValueError as error:self.quality.fail(str(error));return
        output=Odometry();output.header=copy.deepcopy(msg.header);output.header.frame_id=self.world;output.child_frame_id=self.body
        output.pose.pose.position.x,output.pose.pose.position.y,output.pose.pose.position.z=row[1:4]
        output.pose.pose.orientation.x,output.pose.pose.orientation.y,output.pose.pose.orientation.z,output.pose.pose.orientation.w=row[4:]
        if self.previous is not None:
            dt=row[0]-self.previous[0]
            world_velocity=(np.array(row[1:4])-np.array(self.previous[1:4]))/dt
            rotation=Rotation.from_quat(row[4:]);velocity=rotation.inv().apply(world_velocity)
            angular=(Rotation.from_quat(self.previous[4:]).inv()*rotation).as_rotvec()/dt
            output.twist.twist.linear.x,output.twist.twist.linear.y,output.twist.twist.linear.z=velocity.tolist()
            output.twist.twist.angular.x,output.twist.twist.angular.y,output.twist.twist.angular.z=angular.tolist()
        self.previous=row
        # These are explicit fusion-noise floors, not posterior accuracy claims.
        output.pose.covariance=np.diag([.01]*6).ravel().tolist()
        output.twist.covariance=np.diag([.04]*3+[.01]*3).ravel().tolist()
        self.latest=output
        if not self.report()['ready']:return
        self.pub.publish(output)
        if self.active_pub:self.active_pub.publish(output)
        self.path.header=output.header;self.path.poses.append(PoseStamped(header=output.header,pose=output.pose.pose))
        self.path.poses=self.path.poses[-6000:];self.path_pub.publish(self.path)
        tf=TransformStamped(header=output.header,child_frame_id=self.body)
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=row[1:4]
        tf.transform.rotation=output.pose.pose.orientation;self.tf.sendTransform(tf)

    def cloud(self,msg):
        expected=self.backend+'_odom' if self.visual_only or self.backend=='glim' else 'lio_sam_odom' if self.backend=='lio_sam' else 'camera_init'
        if msg.header.frame_id!=expected or source_seconds(msg)<=0:return
        output=copy.deepcopy(msg);output.header.frame_id=self.world;self.registered.publish(output)

    def flight(self,msg):
        for status in msg.status:
            if status.name=='uav001/flight':self.flight_state={v.key:v.value for v in status.values};self.flight_received=time.monotonic()

    def control_pose(self,msg):
        if self.alignment is not None or self.latest is None or msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        if not (0<=time.monotonic()-self.flight_received<=1. and self.flight_state.get('armed')=='False' and self.flight_state.get('landed')=='True'):return
        if not self.report()['ready'] or abs(source_seconds(msg)-source_seconds(self.latest))>.15:return
        self.alignment=control_frame_alignment(pose_matrix(self.latest.pose.pose),pose_matrix(msg.pose.pose))

    def tick(self):
        report=self.report()
        status=DiagnosticStatus(name='uav001/localization' if self.active else self.prefix,hardware_id=self.backend,
            level=DiagnosticStatus.OK if report['ready'] else DiagnosticStatus.ERROR if report['reason'] else DiagnosticStatus.WARN,message=report['state'])
        status.values=[KeyValue(key=k,value=str(v)) for k,v in report.items()]
        status.values+=[KeyValue(key='velocity_source',value='finite differences of estimator poses; body frame'),KeyValue(key='covariance_source',value='configured observation-noise floors; no accuracy claim'),
                        KeyValue(key='global_correction',value='separate; not applied to continuous odometry')]
        message=DiagnosticArray(status=[status]);message.header.stamp=self.get_clock().now().to_msg();self.diag.publish(message)
        if self.active_diag:self.active_diag.publish(message)
        if self.alignment is not None:
            tf=TransformStamped();tf.header.stamp=message.header.stamp;tf.header.frame_id=self.world;tf.child_frame_id='odom'
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=self.alignment[:3,3].tolist()
            q=Rotation.from_matrix(self.alignment[:3,:3]).as_quat()
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=q.tolist()
            self.alignment_pub.publish(tf)
            # Local map frame is the continuous estimator frame. No SLAM/global
            # layer is advertised for a frontend that only estimates odometry.
            tf.header.frame_id='map';self.tf.sendTransform(tf)


def main():
    rclpy.init();node=None
    try:node=BackendNormalizer();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
