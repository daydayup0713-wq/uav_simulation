"""Real LIO-SAM must withstand DDS discovery delivering IMU after old clouds."""
import json, os, subprocess, time
from pathlib import Path
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from backend_configs import write_config
from backend_launch import core_command

def test_lio_sam_rejects_corrections_without_a_measured_imu_interval(tmp_path):
    binary=Path('.deps/backends/lio_sam/install/lib/lio_sam/lio_sam_imuPreintegration').resolve()
    if not binary.exists():pytest.skip('private LIO-SAM build absent')
    calibration=json.loads(Path('configs/sensors-mechanical.json').read_text())
    write_config('lio_sam',calibration,tmp_path/'config','probe')
    context=rclpy.context.Context();rclpy.init(context=context,domain_id=81)
    node=rclpy.create_node('lio_input_boundary_probe',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context);executor.add_node(node)
    corrections=node.create_publisher(Odometry,'/uav001/backends/lio_sam/raw_odometry',10)
    imu=node.create_publisher(Imu,'/uav001/imu/data',10)
    poses=[]
    node.create_subscription(Odometry,'/uav001/backends/lio_sam/odometry/imu_incremental',poses.append,qos_profile_sensor_data)
    log=(tmp_path/'lio.log').open('w')
    child=subprocess.Popen(core_command('lio_sam',binary,tmp_path/'config'),env={**os.environ,'ROS_DOMAIN_ID':'81','ROS_LOG_DIR':str(tmp_path)},stdout=log,stderr=log)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    def correction(stamp):
        msg=Odometry();msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        msg.pose.pose.orientation.w=1.;corrections.publish(msg);spin(.1)
    def sample(stamp):
        msg=Imu();msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        msg.orientation.w=1.;msg.linear_acceleration.z=9.81;imu.publish(msg);spin(.01)
    try:
        deadline=time.monotonic()+8
        while corrections.get_subscription_count()<1 or imu.get_subscription_count()<1:
            assert child.poll() is None
            assert time.monotonic()<deadline,'native subscriptions did not become ready'
            spin(.05)
        sample(10.)
        correction(5.);correction(5.1)
        assert child.poll() is None,(tmp_path/'lio.log').read_text()
        for i in range(1,81):sample(10.+i*.005)
        correction(10.1);correction(10.2)
        for i in range(81,101):sample(10.+i*.005)
        assert child.poll() is None,(tmp_path/'lio.log').read_text()
        assert poses,'valid measured intervals did not produce IMU odometry'
    finally:
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        log.close();executor.shutdown();node.destroy_node();context.shutdown()
