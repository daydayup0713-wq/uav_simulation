"""Private physics -> noisy sensor measurements. Never a control component."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .timed_sensors import PoseBuffer, measure_rays, scan_pattern, point_records, rtk_measurement


def stamp(message, seconds):
    ns = int(round(seconds*1e9))
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)


class TimedSensors(Node):
    def __init__(self):
        super().__init__('timed_sensor_simulator', namespace='_lab')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        config_path=Path(self.declare_parameter('calibration_file','').value)
        self.config = json.loads(config_path.read_text())
        self.boxes = np.asarray(json.loads((config_path.parent/self.config['geometry_file']).read_text())['boxes'])
        lidar = self.config['lidar']
        self.extrinsic = np.eye(4)
        self.extrinsic[:3,:3] = Rotation.from_euler('xyz',lidar['rpy']).as_matrix()
        self.extrinsic[:3,3] = lidar['xyz']
        self.poses, self.rng = PoseBuffer(), np.random.default_rng(self.config['seed'])
        self.worker, self.future = ThreadPoolExecutor(max_workers=1), None
        self.last_scan, self.truth_next, self.gnss_next = None, 0., 0.
        self.cloud_pub = self.create_publisher(PointCloud2,'/uav001/lidar/points',qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(Odometry,'/uav001/ground_truth/odometry',qos_profile_sensor_data)
        self.gnss_pub = self.create_publisher(NavSatFix,'/uav001/gnss/fix',qos_profile_sensor_data) if 'gnss' in self.config else None
        self.diagnostics = self.create_publisher(DiagnosticArray,'/uav001/sensors/timed_diagnostics',10)
        self.create_subscription(Odometry,'/_lab/sensor_physics/odometry',self.pose,
                                 QoSProfile(depth=256,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.005,self.acquire)
        run = os.environ.get('LAB_RUN_DIR')
        self.trace = (Path(run)/'timed-sensors.jsonl').open('a',buffering=1) if run else None
        self.failure, self.dropped = '', 0

    def pose(self, msg):
        expected=self.config['sensor_physics']
        if msg.header.frame_id!=expected['frame'] or msg.child_frame_id!=expected['child_frame']:
            self.failure='private sensor physics frame mismatch';return
        t = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        try:
            self.poses.add(t,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        except ValueError as error:
            self.failure = str(error); return
        if t+1e-7 >= self.truth_next:
            self.truth_pub.publish(msg)
            self.truth_next = (np.floor((t+1e-7)*self.config['truth']['hz'])+1)/self.config['truth']['hz']
        if self.gnss_pub and t+1e-7 >= self.gnss_next:
            c = self.config['gnss']
            position = np.array([p.x,p.y,p.z])+Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(c['xyz'])
            measurement = rtk_measurement(c,t,position,self.rng)
            fix = NavSatFix(); stamp(fix,t); fix.header.frame_id=c['frame']
            fix.status.status,fix.status.service=measurement['status'],1
            fix.latitude,fix.longitude,fix.altitude=map(float,measurement['lla'])
            fix.position_covariance=measurement['covariance'];fix.position_covariance_type=2
            self.gnss_pub.publish(fix)
            diagnostic=DiagnosticArray(); diagnostic.header=fix.header
            diagnostic.status=[DiagnosticStatus(name='uav001/gnss',level=DiagnosticStatus.WARN if measurement['mode']!='fixed' else DiagnosticStatus.OK,
                                               message=measurement['mode'],values=[KeyValue(key='source_time',value=str(t))])]
            self.diagnostics.publish(diagnostic)
            self.gnss_next=(np.floor((t+1e-7)*c['hz'])+1)/c['hz']

    def measure(self, poses, start):
        begun=time.monotonic(); c=self.config['lidar']
        times,directions,channels=scan_pattern(c,start)
        # Separate deterministic streams make scheduling skips independent of
        # subsequent scan noise and avoid sharing RNG state with GNSS callbacks.
        rng=np.random.default_rng([self.config['seed'],int(round(start*c['hz']))])
        xyz=measure_rays(poses,times,directions,self.boxes,self.extrinsic,c['min_m'],c['max_m'],c['noise_stddev_m'],rng)
        records=point_records(xyz,times-start,channels,c['kind'])
        return start,records,time.monotonic()-begun

    def acquire(self):
        if self.future and self.future.done():
            try:
                start,records,compute=self.future.result()
                msg=PointCloud2(); stamp(msg,start);msg.header.frame_id=self.config['lidar']['frame']
                msg.height,msg.width,msg.point_step=1,len(records),records.dtype.itemsize
                msg.row_step=msg.width*msg.point_step;msg.is_dense=False
                msg.fields=[PointField(name=name,offset=records.dtype.fields[name][1],datatype=4 if name in ('ring','line') else 7,count=1)
                            for name in records.dtype.names if name!='padding']
                msg.data=records.tobytes();self.cloud_pub.publish(msg)
                if self.trace:
                    self.trace.write(json.dumps({'start_s':start,'end_s':start+float(records['time'].max()),
                        'compute_s':compute,'points':len(records),'finite_points':int(np.isfinite(records['x']).sum()),
                        'dropped_scans':self.dropped,'measurement':'per-beam ray intersection'})+'\n')
            except (ValueError,RuntimeError) as error:
                self.failure=str(error)
            self.future=None
        if self.failure or self.future or len(self.poses.samples)<2:
            return
        hz=self.config['lidar']['hz']; latest=self.poses.samples[-1][0]
        start=(np.floor((latest+1e-7)*hz)-1)/hz
        if start < self.poses.samples[0][0] or (self.last_scan is not None and start<=self.last_scan+1e-7):
            return
        if self.last_scan is not None:
            self.dropped+=max(0,int(round((start-self.last_scan)*hz))-1)
        self.last_scan=start
        poses=PoseBuffer();poses.samples.extend(self.poses.samples)
        self.future=self.worker.submit(self.measure,poses,start)

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        if self.trace:self.trace.close()
        super().destroy_node()


def main():
    rclpy.init();node=TimedSensors()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
