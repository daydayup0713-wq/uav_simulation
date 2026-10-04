import json
import time
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from sensor_model import prepare_sensors
from uav_lab_tools.sensor_node import Sensors
from conftest import ROOT

def test_sensor_node_starts_and_preserves_raw_imu_source_time(tmp_path,isolated_ros_domain):
    cfg=prepare_sensors(ROOT,tmp_path)
    rclpy.init(args=['--ros-args','-p','calibration_file:='+str(cfg['calibration_path'])],domain_id=isolated_ros_domain)
    sensor=client=None
    try:
        sensor=Sensors();client=rclpy.create_node('sensor_contract_test')
        output=[];transforms=[]
        client.create_subscription(Imu,'/uav001/imu/data',output.append,qos_profile_sensor_data)
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        client.create_subscription(TFMessage,'/tf_static',lambda m:transforms.extend(m.transforms),qos)
        publisher=client.create_publisher(Imu,'/uav001/sim/imu',qos_profile_sensor_data)
        raw=Imu();raw.header.frame_id='imu_link';raw.header.stamp.sec=123;raw.angular_velocity.z=.4;raw.linear_acceleration.z=9.81;raw.orientation.z=.3;raw.orientation.w=.95
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and not (output and transforms):
            publisher.publish(raw);rclpy.spin_once(sensor,timeout_sec=.01);rclpy.spin_once(client,timeout_sec=.01)
        assert output and transforms
        assert output[-1].header.stamp.sec==123 and output[-1].angular_velocity.z==.4
        assert output[-1].orientation_covariance[0]==-1 and output[-1].orientation.z==0
        assert {t.child_frame_id for t in transforms}=={'lidar_link','imu_link','camera_link','camera_optical_frame'}
    finally:
        if client:client.destroy_node()
        if sensor:sensor.destroy_node()
        rclpy.shutdown()
