#!/usr/bin/python3
"""CPU-only analytical observation fixture for CI, distinct from rendered sensor acceptance."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/uav_lab_tools'))
from uav_lab_tools.datasets import file_hash


def state(t):
    u=np.clip((t-5)/30,0,1)
    theta=2*np.pi*(3*u*u-2*u*u*u)
    xyz=np.array([1.5*(1-np.cos(theta)),1.5*np.sin(theta),.5*(1-np.cos(theta))])
    r=Rotation.from_euler('xyz',[.07*np.sin(theta),.05*np.sin(2*theta),.1*np.sin(theta)])
    return xyz,r


def generate(destination):
    destination=Path(destination)
    if destination.exists(): raise ValueError('fixture output already exists')
    destination.mkdir(parents=True)
    calibration=json.loads((ROOT/'configs/sensors.json').read_text())
    (destination/'configuration').mkdir()
    (destination/'configuration/sensors.json').write_text(json.dumps(calibration,indent=2)+'\n')
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(destination/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    topics={'/uav001/imu/data':'sensor_msgs/msg/Imu','/uav001/lidar/points':'sensor_msgs/msg/PointCloud2',
            '/uav001/ground_truth/odometry':'nav_msgs/msg/Odometry','/clock':'rosgraph_msgs/msg/Clock'}
    for name,kind in topics.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
    rng=np.random.default_rng(1234)
    wall=rng.uniform([-4,-4,-.25],[6,6,4],(2400,3))
    for axis,limit,index in [(0,-4,0),(0,6,1),(1,-4,2),(1,6,3),(2,-.25,4),(2,4,5)]:
        wall[400*index:400*(index+1),axis]=limit
    def header(msg,t,frame):
        ns=int(round((100+t)*1e9));msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
        msg.header.frame_id=frame;return ns
    for i in range(8001):
        t=i/200; p,r=state(t); dt=.001
        before,rb=state(t-dt);after,ra=state(t+dt)
        acc=r.inv().apply((after-2*p+before)/(dt*dt)+[0,0,9.80665])
        omega=(rb.inv()*ra).as_rotvec()/(2*dt)
        imu=Imu();ns=header(imu,t,'imu_link');imu.orientation_covariance[0]=-1.
        clock=Clock();clock.clock.sec=ns//10**9;clock.clock.nanosec=ns%10**9
        writer.write('/clock',serialize_message(clock),ns)
        imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,acc)
        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,omega)
        writer.write('/uav001/imu/data',serialize_message(imu),ns)
        if i%20==0:
            cloud=PointCloud2();header(cloud,t,'lidar_link')
            xyz=r.inv().apply(wall-p)-calibration['lidar']['xyz']+rng.normal(0,.003,wall.shape)
            values=np.column_stack([xyz,np.ones(len(xyz))]).astype('<f4')
            cloud.height=1;cloud.width=len(values);cloud.point_step=16;cloud.row_step=16*len(values);cloud.is_dense=True
            cloud.fields=[PointField(name=name,offset=j*4,datatype=7,count=1) for j,name in enumerate(['x','y','z','intensity'])]
            cloud.data=values.tobytes();writer.write('/uav001/lidar/points',serialize_message(cloud),ns)
        if i%8==0:
            truth=Odometry();header(truth,t,'sim_world');truth.child_frame_id='truth_base_link'
            truth.pose.pose.position.x,truth.pose.pose.position.y,truth.pose.pose.position.z=map(float,p)
            q=r.as_quat();truth.pose.pose.orientation.x,truth.pose.pose.orientation.y,truth.pose.pose.orientation.z,truth.pose.pose.orientation.w=q
            writer.write('/uav001/ground_truth/odometry',serialize_message(truth),ns)
    del writer
    metadata={'schema_version':1,'complete':True,'kind':'analytical_lio_fixture',
              'purpose':'CI backend regression; not ray rendering, real sensor fidelity or flight acceptance',
              'calibration':calibration,
              'bag_sha256':{p.name:file_hash(p) for p in (destination/'bag').iterdir() if p.is_file()},
              'configuration_sha256':{'sensors.json':file_hash(destination/'configuration/sensors.json')}}
    (destination/'dataset.json').write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('destination')
    generate(parser.parse_args().destination)
