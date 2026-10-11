"""Standard ROS subscriptions -> bounded display frames; no PX4 publisher."""
import json
from collections import deque
import os
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as NavPath,Odometry
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import Buffer,TransformListener,TransformException,ExtrapolationException
from tf2_msgs.msg import TFMessage
from .observatory import LatestFrames,Accumulation,read_cloud,pack_points
from .observation_server import ObservationServer


class Observatory(Node):
    def __init__(self):
        super().__init__('web_observatory',namespace='uav001')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.evaluation=bool(self.declare_parameter('evaluation',False).value)
        port=int(self.declare_parameter('port',8765).value)
        self.store=LatestFrames(evaluation=self.evaluation);self.pending={};self.errors={}
        self.accumulation=Accumulation();self.telemetry={};self.last_pose=None;self.global_cloud=None
        run=Path(os.environ['LAB_RUN_DIR']) if os.environ.get('LAB_RUN_DIR') else None
        manifest=json.loads((run/'manifest.json').read_text()) if run and (run/'manifest.json').exists() else {}
        self.backend=self.declare_parameter('backend',(manifest.get('backend_selection') or {}).get('localization','')).value
        calibration_path=self.declare_parameter('calibration','').value
        self.calibration=json.loads(Path(calibration_path).read_text()) if calibration_path else manifest.get('calibration')
        self.backend_local=deque(maxlen=1000);self.backend_global=None
        self.tf_buffer=Buffer(node=self);self.tf=TransformListener(self.tf_buffer,self)
        self.backend_tf=Buffer(node=self)
        qos_map=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        registered_topic=('/uav001/backends/glim/registered_points' if self.backend=='glim' else
            '/uav001/backends/'+self.backend+'/registered' if self.backend else '/glim_ros/aligned_points_corrected')
        for layer,topic,qos in [('raw','/uav001/lidar/points',qos_profile_sensor_data),
                ('registered',registered_topic,qos_profile_sensor_data),
                ('global','/uav001/backends/'+self.backend+'/global_map' if self.backend else '/uav001/localization/map',qos_map),('voxels','/uav001/navigation/occupied',qos_map)]:
            self.create_subscription(PointCloud2,topic,lambda msg,key=layer:self.pending.update({key:msg}),qos)
        for layer,topic in [('planned','/uav001/navigation/path'),('actual','/uav001/path')]:
            self.create_subscription(NavPath,topic,lambda msg,key=layer:self.path(key,msg),10)
        self.create_subscription(Odometry,'/uav001/odometry',self.pose,10)
        if self.backend:
            if self.calibration is None:raise ValueError('selected backend display requires archived calibration')
            self.create_subscription(Odometry,'/uav001/backends/'+self.backend+'/odometry',self.backend_local_pose,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/uav001/backends/'+self.backend+'/global_odometry',self.backend_global_pose,qos_profile_sensor_data)
            if self.backend=='glim':
                self.create_subscription(TFMessage,'/uav001/backends/glim/tf',self.backend_transforms,qos_profile_sensor_data)
        for topic in ('/uav001/diagnostics','/uav001/localization/diagnostics','/uav001/navigation/diagnostics','/uav001/sensors/timed_diagnostics'):
            self.create_subscription(DiagnosticArray,topic,self.diagnostics,10)
        if self.evaluation:
            self.create_subscription(Odometry,'/uav001/ground_truth/odometry',self.truth,qos_profile_sensor_data)
        root=Path(os.environ.get('LAB_ROOT',Path.cwd()))
        from .registry import Registry
        registry=Registry(root/'configs/backends.json',root/'.runtime/backend-evidence')
        self.store.put('registry',json.dumps({'kind':'registry','backends':registry.list()}))
        run=Path(os.environ['LAB_RUN_DIR']) if os.environ.get('LAB_RUN_DIR') else None
        self.run=run.name if run else None
        self.server=ObservationServer(self.store,port);self.server.start()
        self.ready_file=run/'web-observatory-ready' if run else None
        if self.ready_file:self.ready_file.write_text('observation socket ready\n')
        self.create_timer(.2,self.display,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def backend_local_pose(self,msg):
        if msg.header.frame_id!='lio_odom':return
        self.backend_local.append(msg)

    def backend_global_pose(self,msg):
        # Pinned GLIM odom_corrected remains local fixed-window odometry.
        # Its global optimization is communicated by the private map->odom TF.
        expected={'lio_sam':'lio_sam_odom','vins_fusion':'vins_fusion_map'}.get(self.backend)
        if expected and msg.header.frame_id==expected:self.backend_global=msg

    def backend_transforms(self,msg):
        for tf in msg.transforms:
            if tf.header.frame_id=='glim_map' and tf.child_frame_id=='glim_odom':
                self.backend_tf.set_transform(tf,'selected GLIM core')

    @staticmethod
    def transform_matrix(tf):
        p,q=tf.transform.translation,tf.transform.rotation
        matrix=np.eye(4);matrix[:3,3]=[p.x,p.y,p.z]
        matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        return matrix

    def backend_map_transform(self,source,stamp):
        if self.backend=='glim':
            tf=self.backend_tf.lookup_transform('glim_odom','glim_map',Time())
            measured=Time.from_msg(tf.header.stamp).nanoseconds/1e9
            now=self.get_clock().now().nanoseconds/1e9
            if measured<=0 or now and not -.15<=now-measured<=.25:
                raise ExtrapolationException('actual GLIM global correction is stale')
            # Both local world frames have identical coordinates; normalized
            # odometry changes IMU->body pose, never the world origin/axes.
            aligned,_=self.display_transform('lio_odom',tf.header.stamp)
            return aligned@self.transform_matrix(tf),abs(Time.from_msg(stamp).nanoseconds/1e9-measured)
        from .localization_frames import pose_matrix
        from .backend_contract import normalized_pose
        global_pose=self.backend_global
        if global_pose is None or global_pose.header.frame_id!=source or not self.backend_local:
            raise ValueError('global map has no matching actual local/global pose correction')
        seconds=lambda msg:msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        measured=seconds(global_pose)
        local=min(self.backend_local,key=lambda p:abs(seconds(p)-measured))
        now=self.get_clock().now().nanoseconds/1e9
        if measured<=0 or abs(seconds(local)-measured)>.15 or now and not -.15<=now-measured<=.25:
            raise ValueError('actual global/local display correction is stale or unmatched')
        sensor=self.calibration['lidar'] if self.backend=='lio_sam' else self.calibration['imu']
        p,q=global_pose.pose.pose.position,global_pose.pose.pose.orientation
        row=normalized_pose(measured,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],sensor)
        matrix=np.eye(4);matrix[:3,3]=row[1:4];matrix[:3,:3]=Rotation.from_quat(row[4:]).as_matrix()
        correction=pose_matrix(local.pose.pose)@np.linalg.inv(matrix)
        aligned,_=self.display_transform('lio_odom',local.header.stamp)
        return aligned@correction,abs(Time.from_msg(stamp).nanoseconds/1e9-measured)

    def display_transform(self,source,stamp,*,global_map=False):
        if self.backend and source in ('glim_map','lio_sam_odom','vins_fusion_map'):
            if not global_map:raise ValueError('optimized global frame may only enter the global display layer')
            return self.backend_map_transform(source,stamp)
        # GLIM's lio_map topic frame has an explicit platform alias, map.
        source='map' if source=='lio_map' else source
        if source=='odom':return np.eye(4),0.
        delta=0.
        if global_map:
            # Optimized map geometry is expressed in the current map frame;
            # retaining its measurement stamp must not select an old correction.
            tf=self.tf_buffer.lookup_transform('odom',source,Time())
            delta=abs(Time.from_msg(stamp).nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
            now=self.get_clock().now().nanoseconds
            if now and abs(now-Time.from_msg(tf.header.stamp).nanoseconds)>150000000:
                raise ExtrapolationException('current global map alignment is stale')
        else:
            try:tf=self.tf_buffer.lookup_transform('odom',source,Time.from_msg(stamp))
            except ExtrapolationException:
                tf=self.tf_buffer.lookup_transform('odom',source,Time())
                delta=abs(Time.from_msg(stamp).nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
                if delta>.15:raise ExtrapolationException('display TF time difference exceeds 150ms')
        return self.transform_matrix(tf),delta

    def path(self,layer,msg):
        points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]
        points=points[::max(1,int(np.ceil(len(points)/6000)))]
        if msg.header.frame_id!='odom':return
        self.store.put(layer,json.dumps({'kind':'path','layer':layer,'frame':'odom','points':points},allow_nan=False))

    def pose(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if msg.header.frame_id=='odom':
            self.last_pose={'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],
                            'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,'received':time.monotonic()}

    def diagnostics(self,msg):
        for s in msg.status[:16]:
            if len(self.telemetry)>=32 and s.name[:100] not in self.telemetry:
                self.telemetry.pop(next(iter(self.telemetry)))
            self.telemetry[s.name[:100]]={'level':int.from_bytes(s.level,'little'),'message':s.message[:200],
                'values':{v.key[:100]:v.value[:200] for v in s.values[:40]},'received':time.monotonic()}

    def truth(self,msg):
        p=msg.pose.pose.position
        self.store.put('truth',json.dumps({'kind':'truth','evaluation_only':True,'frame':msg.header.frame_id,
                                         'position':[p.x,p.y,p.z]},allow_nan=False))

    def display(self):
        pending,self.pending=self.pending,{}
        if 'global' in pending:self.global_cloud=pending['global']
        if self.global_cloud is not None:pending['global']=self.global_cloud
        for layer,msg in pending.items():
            try:
                source=msg.header.frame_id
                # Upstream aligned_points uses local T_world_sensor while
                # labeling it map_frame_id; the exact private topic distinguishes
                # it from the genuinely globally optimized map layer.
                local_glim=layer=='registered' and self.backend=='glim' and source=='glim_map'
                transform,delta=self.display_transform('lio_odom' if local_glim else source,msg.header.stamp,global_map=layer=='global')
                points,metadata=read_cloud(msg,transform=transform,frame='odom')
                metadata['display_transform_time_difference_s']=delta
                if local_glim:metadata['display_alignment']='pinned GLIM local aligned_points topic; source header preserved as glim_map'
                if layer=='global':metadata['display_alignment']='current map correction; original source stamp preserved'
                if layer=='registered':
                    source_count=len(self.accumulation.points)+len(points)
                    points=self.accumulation.add('odom',points)
                    metadata.update(source_count=source_count,display_decimated=True,display_scope='bounded accumulation window')
                self.store.put(layer,pack_points(points,{**metadata,'layer':layer}))
                self.errors.pop(layer,None)
            except (ValueError,TransformException) as error:self.errors[layer]=str(error)[:200]
        now=time.monotonic()
        telemetry={key:{**value,'age_s':now-value['received']} for key,value in self.telemetry.items()}
        pose={**self.last_pose,'age_s':now-self.last_pose['received']} if self.last_pose else None
        self.store.put('telemetry',json.dumps({'kind':'telemetry','run_id':self.run,'simulation_time':self.get_clock().now().nanoseconds/1e9,
            'vehicle':pose,'diagnostics':telemetry,'layer_errors':self.errors,'evaluation':self.evaluation,
            'display_hz':5,'point_budget':100000},allow_nan=False))

    def destroy_node(self):
        self.server.close()
        if self.ready_file:self.ready_file.unlink(missing_ok=True)
        super().destroy_node()


def main():
    rclpy.init();node=None
    try:node=Observatory();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
