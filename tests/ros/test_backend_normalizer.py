import json,time
from pathlib import Path
import pytest
rclpy=pytest.importorskip('rclpy')
from sensor_msgs.msg import Imu,PointCloud2,Image
from nav_msgs.msg import Odometry
from uav_lab_experiments.backend_node import BackendNormalizer
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from std_msgs.msg import String


def measured_cloud(header):
    fields=[PointField(name=name,offset=offset,datatype=datatype,count=1) for name,offset,datatype in
            [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('time',16,7),('line',20,4)]]
    return point_cloud2.create_cloud(header,fields,[(1.,0.,0.,10.,0.,0),(2.,0.,0.,20.,.099,1)])


def test_glim_original_synchronous_cloud_needs_no_invented_beam_time():
    calibration=Path('configs/sensors.json').resolve()
    rclpy.init(args=['--ros-args','-p','backend:=glim','-p','calibration:='+str(calibration)])
    node=BackendNormalizer()
    try:
        header=Header(frame_id='lidar_link');header.stamp.sec=10
        cloud=point_cloud2.create_cloud_xyz32(header,[(1.+x*.2,y*.2,1.) for x in range(32) for y in range(32)])
        node.points(cloud)
        assert node.quality.sources['points'][0]==10.
        assert node.quality.reason==''
    finally:
        node.destroy_node();rclpy.shutdown()


def test_orb_core_continuity_failure_before_warmup_is_explicit_and_latched():
    calibration=Path('configs/sensors-livox.json').resolve()
    rclpy.init(args=['--ros-args','-p','backend:=orb_slam3','-p','calibration:='+str(calibration)])
    node=BackendNormalizer()
    try:
        node.core_state(String(data='{"input_ready":true,"inertial_initialized":false}'))
        assert node.report()['state']=='INITIALIZING'
        node.core_state(String(data='{"continuity_failed":true,"reason":"tracking frame gap"}'))
        assert node.report()['state']=='FAILED'
        assert 'tracking frame gap' in node.report()['reason']
        node.core_state(String(data='{"continuity_failed":false}'))
        assert node.report()['state']=='FAILED'
    finally:
        node.destroy_node();rclpy.shutdown()


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


@pytest.mark.parametrize('backend',['orb_slam3','vins_fusion'])
def test_visual_normalizer_is_metric_imu_pose_with_camera_only_sources(backend):
    calibration=Path('configs/sensors-livox.json').resolve()
    rclpy.init(args=['--ros-args','-p','backend:='+backend,'-p','calibration:='+str(calibration)])
    node=BackendNormalizer()
    try:
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=11.,clock_type=rclpy.clock.ClockType.ROS_TIME))
        for i in range(25):
            imu=Imu();imu.header.frame_id='imu_link';imu.header.stamp.sec=10;imu.header.stamp.nanosec=i*40_000_000;node.imu(imu)
            image=Image();image.header=Header(stamp=imu.header.stamp,frame_id='camera_optical_frame');node.image(image)
            odom=Odometry();odom.header=Header(stamp=imu.header.stamp,frame_id=backend+'_odom');odom.child_frame_id='imu_link'
            odom.pose.pose.orientation.w=1.;odom.pose.pose.position.x=i*.01;node.pose(odom)
        assert node.report()['ready']
        assert set(node.quality.sources)=={'camera','imu'}
        assert node.latest.twist.twist.linear.x==pytest.approx(.25)
        odom.header.frame_id='camera_optical_frame';node.pose(odom)
        assert node.report()['state']=='FAILED'
    finally:
        node.destroy_node();rclpy.shutdown()


def test_lio_sam_lidar_pose_is_converted_to_body_without_camera_dependency():
    calibration=Path('configs/sensors-mechanical.json').resolve()
    rclpy.init(args=['--ros-args','-p','backend:=lio_sam','-p','calibration:='+str(calibration)])
    node=BackendNormalizer()
    try:
        node.quality.warmup_samples=1
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.2,clock_type=rclpy.clock.ClockType.ROS_TIME))
        imu=Imu();imu.header=Header(frame_id='imu_link');imu.header.stamp.sec=10;imu.header.stamp.nanosec=200000000;node.imu(imu)
        header=Header(frame_id='lidar_link');header.stamp.sec=10
        cloud=measured_cloud(header)
        for field in cloud.fields:
            if field.name=='line':field.name='ring'
        node.points(cloud)
        odom=Odometry();odom.header=Header(stamp=imu.header.stamp,frame_id='lio_sam_odom');odom.child_frame_id='odom_mapping'
        odom.pose.pose.orientation.w=1.;odom.pose.pose.position.z=2.16;node.pose(odom)
        assert node.report()['ready']
        assert node.latest.pose.pose.position.z==pytest.approx(2.)
        assert node.camera is None
    finally:
        node.destroy_node();rclpy.shutdown()
