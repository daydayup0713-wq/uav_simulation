import rclpy
from sensor_msgs.msg import PointCloud2,PointField
import numpy as np
import pytest


def test_observer_uses_standard_inputs_without_truth_or_control_and_emits_real_cloud(isolated_ros_domain,monkeypatch):
    from uav_lab_experiments.observation_node import Observatory
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(args=['--ros-args','-p','port:=0'],domain_id=isolated_ros_domain)
    node=Observatory()
    try:
        topics={name for name,_ in node.get_subscriber_names_and_types_by_node(node.get_name(),node.get_namespace())}
        assert '/uav001/ground_truth/odometry' not in topics
        assert '/_lab/sensor_physics/odometry' not in topics
        publishers={name for name,_ in node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())}
        assert not any(name.startswith('/fmu/in/') for name in publishers)
        msg=PointCloud2();msg.header.frame_id='odom';msg.header.stamp.sec=1
        msg.height=1;msg.width=2;msg.point_step=12;msg.row_step=24
        msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        msg.data=np.array([[1,2,3],[4,5,6]],dtype='<f4').tobytes()
        node.pending['raw']=msg;node.display()
        assert 'raw' in node.store.snapshot() and not node.errors
    finally:node.destroy_node();rclpy.shutdown()

def test_display_uses_only_a_bounded_nearby_tf_when_sensor_arrives_first(isolated_ros_domain,monkeypatch):
    from uav_lab_experiments.observation_node import Observatory
    from geometry_msgs.msg import TransformStamped
    from builtin_interfaces.msg import Time
    from tf2_ros import TransformException
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(args=['--ros-args','-p','port:=0'],domain_id=isolated_ros_domain)
    node=Observatory()
    try:
        tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='lidar_link'
        tf.header.stamp=Time(sec=2,nanosec=50000000);tf.transform.rotation.w=1.;tf.transform.translation.x=1.
        node.tf_buffer.set_transform(tf,'test')
        matrix,delta=node.display_transform('lidar_link',Time(sec=2,nanosec=100000000))
        assert matrix[0,3]==1 and delta==pytest.approx(.05)
        with pytest.raises(TransformException):node.display_transform('lidar_link',Time(sec=3))
        # A global map belongs to the current map alignment, not its oldest
        # measurement time. Raw measurements retain their bounded-time rule.
        matrix,delta=node.display_transform('lidar_link',Time(sec=1),global_map=True)
        assert matrix[0,3]==1 and delta==pytest.approx(1.05)
    finally:node.destroy_node();rclpy.shutdown()
