"""Standard ROS subscriptions -> bounded display frames; no PX4 publisher."""
import json
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
        self.tf_buffer=Buffer(node=self);self.tf=TransformListener(self.tf_buffer,self)
        qos_map=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for layer,topic,qos in [('raw','/uav001/lidar/points',qos_profile_sensor_data),
                ('registered','/glim_ros/aligned_points_corrected',qos_profile_sensor_data),
                ('global','/uav001/localization/map',qos_map),('voxels','/uav001/navigation/occupied',qos_map)]:
            self.create_subscription(PointCloud2,topic,lambda msg,key=layer:self.pending.update({key:msg}),qos)
        for layer,topic in [('planned','/uav001/navigation/path'),('actual','/uav001/path')]:
            self.create_subscription(NavPath,topic,lambda msg,key=layer:self.path(key,msg),10)
        self.create_subscription(Odometry,'/uav001/odometry',self.pose,10)
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

    def display_transform(self,source,stamp,*,global_map=False):
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
        p,q=tf.transform.translation,tf.transform.rotation
        matrix=np.eye(4);matrix[:3,3]=[p.x,p.y,p.z]
        matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        return matrix,delta

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
                transform,delta=self.display_transform(msg.header.frame_id,msg.header.stamp,global_map=layer=='global')
                points,metadata=read_cloud(msg,transform=transform,frame='odom')
                metadata['display_transform_time_difference_s']=delta
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
