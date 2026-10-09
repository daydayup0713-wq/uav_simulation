import json,time
from pathlib import Path
import pytest
import rclpy
from sensor_msgs.msg import Imu,PointCloud2,Image
from nav_msgs.msg import Odometry
from uav_lab_experiments.backend_node import BackendNormalizer
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField


def measured_cloud(header):
    fields=[PointField(name=name,offset=offset,datatype=datatype,count=1) for name,offset,datatype in
            [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('time',16,7),('line',20,4)]]
    return point_cloud2.create_cloud(header,fields,[(1.,0.,0.,10.,0.,0),(2.,0.,0.,20.,.099,1)])


def test_normalizer_preserves_source_pose_and_latches_visual_failure():
    calibration=Path('configs/sensors-livox.json').resolve()
    rclpy.init(args=['--ros-args','-p','backend:=fast_livo2','-p','calibration:='+str(calibration)])
    node=BackendNormalizer()
    try:
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=11.,clock_type=rclpy.clock.ClockType.ROS_TIME))
        for i in range(25):
            ns=10_000_000_000+i*40_000_000
            imu=Imu();imu.header.frame_id='imu_link';imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(ns,10**9)
            node.imu(imu)
            cloud=measured_cloud(Header(stamp=imu.header.stamp,frame_id='lidar_link'));node.points(cloud)
            image=Image();image.header=imu.header;image.header.frame_id='camera_optical_frame';node.image(image)
            odom=Odometry();odom.header=imu.header;odom.header.frame_id='camera_init';odom.child_frame_id='aft_mapped'
            odom.pose.pose.orientation.w=1.;odom.pose.pose.position.x=.01*i;node.pose(odom)
        assert node.latest.header.stamp.nanosec==960000000
        assert node.latest.header.frame_id=='fast_livo2_odom'
        assert node.latest.twist.twist.linear.x==pytest.approx(.25)
        assert node.report()['ready']
        node.camera=(10.96,time.monotonic()-2.)
        assert node.report()['state']=='FAILED'
        node.camera=(10.96,time.monotonic())
        assert not node.report()['ready']
        assert not node.active and node.active_pub is None
    finally:
        node.destroy_node();rclpy.shutdown()


def test_watchdog_uses_latest_measured_beam_without_relabeling_scan_header():
    calibration=Path('configs/sensors-livox.json').resolve()
    rclpy.init(args=['--ros-args','-p','calibration:='+str(calibration)])
    node=BackendNormalizer()
    try:
        node.quality.warmup_samples=1
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.42,clock_type=rclpy.clock.ClockType.ROS_TIME))
        header=Header(frame_id='lidar_link');header.stamp.sec=10
        cloud=measured_cloud(header);node.points(cloud)
        imu=Imu();imu.header.frame_id='imu_link';imu.header.stamp.sec=10;imu.header.stamp.nanosec=420000000;node.imu(imu)
        image=Image();image.header=imu.header;image.header.frame_id='camera_optical_frame';node.image(image)
        odom=Odometry();odom.header=imu.header;odom.header.frame_id='camera_init';odom.child_frame_id='aft_mapped'
        odom.pose.pose.orientation.w=1.;node.pose(odom)
        assert node.report()['ready']
        assert node.report()['points_age_s']==pytest.approx(.321)
        assert cloud.header.stamp.sec==10 and cloud.header.stamp.nanosec==0
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.6,clock_type=rclpy.clock.ClockType.ROS_TIME))
        assert node.report()['state']=='FAILED'
    finally:
        node.destroy_node();rclpy.shutdown()
