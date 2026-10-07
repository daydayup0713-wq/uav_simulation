import importlib.util
import time
import numpy as np
import pytest

rclpy=pytest.importorskip('rclpy')


def test_registered_map_uses_sensor_time_and_quality_expires(monkeypatch,isolated_ros_domain):
    assert importlib.util.find_spec('uav_lab_navigation.node'), 'navigation ROS mapping missing'
    from uav_lab_navigation.node import NavigationNode
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    from sensor_msgs.msg import PointCloud2,PointField
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from uav_lab_localization.benchmark import cloud_xyz
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(domain_id=isolated_ros_domain);node=NavigationNode();client=rclpy.create_node('navigation_map_test')
    received=[]
    client.create_subscription(PointCloud2,'/uav001/navigation/occupied',received.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock=client.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    try:
        end=time.monotonic()+.4
        while time.monotonic()<end:
            c=Clock();c.clock.sec=10;c.clock.nanosec=200000000;clock.publish(c)
            rclpy.spin_once(node,timeout_sec=.01);rclpy.spin_once(client,timeout_sec=.01)
        alignment=TransformStamped();alignment.header.frame_id='lio_odom';alignment.child_frame_id='odom'
        alignment.transform.translation.x=1.;alignment.transform.rotation.w=1.;node.on_alignment(alignment)
        for t,x in ((10.,1.),(10.2,1.2)):
            msg=Odometry();msg.header.frame_id='lio_odom';msg.child_frame_id='lio_base_link'
            msg.header.stamp.sec=10;msg.header.stamp.nanosec=round((t-10)*1e9)
            msg.pose.pose.position.x=x;msg.pose.pose.position.z=1.;msg.pose.pose.orientation.w=1.;node.on_pose(msg)
        diag=DiagnosticArray();diag.status=[DiagnosticStatus(name='uav001/localization',level=DiagnosticStatus.OK,values=[KeyValue(key='ready',value='True')])]
        node.on_quality(diag)
        cloud=PointCloud2();cloud.header.frame_id='lidar_link';cloud.header.stamp.sec=10;cloud.header.stamp.nanosec=100000000
        xyz=np.column_stack((np.full(1000,1.5),np.random.default_rng(3).uniform(-2,2,(1000,2)))).astype('<f4')
        cloud.width=1000;cloud.height=1;cloud.point_step=12;cloud.row_step=12000
        cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')];cloud.data=xyz.tobytes()
        node.on_cloud(cloud)
        # A newer scan may arrive before its LIO pose; retain the bracketed scan.
        import copy
        ahead=copy.deepcopy(cloud);ahead.header.stamp.nanosec=300000000
        node.on_cloud(ahead);node.map_cycle()
        end=time.monotonic()+.3
        while time.monotonic()<end and not received:rclpy.spin_once(client,timeout_sec=.01)
        assert received and received[-1].header.frame_id=='odom'
        assert abs(np.mean(cloud_xyz(received[-1])[:,0])-1.6)<.21
        assert node.report()['ready']
        assert not node.report(now=time.monotonic()+2)['ready']
        assert not any('ground_truth' in topic for topic,_ in node.get_topic_names_and_types() if node.count_subscribers(topic))
        assert not any(topic.startswith('/fmu/in/') for topic,_ in node.get_topic_names_and_types())
    finally:client.destroy_node();node.destroy_node();rclpy.shutdown()


def test_localization_publishes_frozen_alignment_for_navigation(monkeypatch,isolated_ros_domain):
    from uav_lab_localization.localization_node import LocalizationNode
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile,DurabilityPolicy
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(domain_id=isolated_ros_domain);node=LocalizationNode();client=rclpy.create_node('alignment_consumer')
    output=[];client.create_subscription(TransformStamped,'/uav001/localization/control_alignment',output.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        node.lio_to_control=np.eye(4);node.lio_to_control[0,3]=2.
        node.tick()
        end=time.monotonic()+.6
        while time.monotonic()<end and not output:rclpy.spin_once(client,timeout_sec=.01)
        assert output and output[-1].transform.translation.x==2.
        assert output[-1].header.frame_id=='lio_odom' and output[-1].child_frame_id=='odom'
    finally:client.destroy_node();node.destroy_node();rclpy.shutdown()
