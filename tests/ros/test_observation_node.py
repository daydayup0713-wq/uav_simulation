import pytest
rclpy=pytest.importorskip('rclpy')
from sensor_msgs.msg import PointCloud2,PointField
import numpy as np


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


def test_selected_backend_layers_use_local_and_global_pose_pair_without_truth(isolated_ros_domain,monkeypatch,tmp_path):
    import json
    from pathlib import Path
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from uav_lab_experiments.observation_node import Observatory
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    config=Path(__file__).resolve().parents[2]/'configs/navigation-sensors.json'
    rclpy.init(args=['--ros-args','-p','port:=0','-p','backend:=vins_fusion','-p','calibration:='+str(config)],domain_id=isolated_ros_domain)
    node=Observatory()
    try:
        topics={name for name,_ in node.get_subscriber_names_and_types_by_node(node.get_name(),node.get_namespace())}
        assert '/uav001/backends/vins_fusion/registered' in topics
        assert '/uav001/backends/vins_fusion/global_map' in topics
        assert '/uav001/backends/vins_fusion/global_odometry' in topics
        local=Odometry();local.header.frame_id='lio_odom';local.header.stamp=Time(sec=2);local.pose.pose.orientation.w=1.
        local.pose.pose.position.x=2.
        global_pose=Odometry();global_pose.header.frame_id='vins_fusion_map';global_pose.child_frame_id='imu_link'
        global_pose.header.stamp=Time(sec=2);global_pose.pose.pose.orientation.w=1.;global_pose.pose.pose.position.x=12.
        tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='lio_odom';tf.transform.rotation.w=1.
        node.tf_buffer.set_transform_static(tf,'test')
        node.backend_local_pose(local);node.backend_global_pose(global_pose)
        matrix,delta=node.display_transform('vins_fusion_map',Time(sec=1),global_map=True)
        # Both bodies normalized with actual sensor extrinsic, correction only
        # for display; the algorithm/map coordinates stay untouched.
        imu=json.loads(config.read_text())['imu']['xyz']
        assert matrix[0,3]==pytest.approx(-10.+imu[0]) and delta==1.
        assert '/uav001/ground_truth/odometry' not in topics
        with pytest.raises(ValueError):node.display_transform('vins_fusion_map',Time(sec=1),global_map=False)
    finally:node.destroy_node();rclpy.shutdown()


def test_legacy_run_with_no_backend_selection_preserves_old_layers(isolated_ros_domain,monkeypatch,tmp_path):
    import json
    from uav_lab_experiments.observation_node import Observatory
    (tmp_path/'manifest.json').write_text(json.dumps({'backend_selection':None}))
    monkeypatch.setenv('LAB_RUN_DIR',str(tmp_path))
    rclpy.init(args=['--ros-args','-p','port:=0'],domain_id=isolated_ros_domain)
    node=None
    try:
        node=Observatory()
        topics={name for name,_ in node.get_subscriber_names_and_types_by_node(node.get_name(),node.get_namespace())}
        assert '/glim_ros/aligned_points_corrected' in topics
        assert '/uav001/localization/map' in topics
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


def test_glim_observed_topic_frames_use_private_global_tf_and_local_cloud_contract(isolated_ros_domain,monkeypatch):
    from pathlib import Path
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
    from uav_lab_experiments.observation_node import Observatory
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    config=Path(__file__).resolve().parents[2]/'configs/navigation-sensors.json'
    rclpy.init(args=['--ros-args','-p','port:=0','-p','backend:=glim','-p','calibration:='+str(config)],domain_id=isolated_ros_domain)
    node=Observatory()
    try:
        topics={name for name,_ in node.get_subscriber_names_and_types_by_node(node.get_name(),node.get_namespace())}
        assert '/uav001/backends/glim/registered_points' in topics
        assert '/uav001/backends/glim/tf' in topics
        # In pinned GLIM, both Odometry topics carry local T_odom_imu and
        # glim_odom; only its private map->odom TF is global optimization.
        false_global=Odometry();false_global.header.frame_id='glim_odom'
        node.backend_global_pose(false_global)
        assert node.backend_global is None
        alignment=TransformStamped();alignment.header.frame_id='odom';alignment.child_frame_id='lio_odom'
        alignment.transform.rotation.w=1.;alignment.transform.translation.x=3.
        node.tf_buffer.set_transform_static(alignment,'test')
        correction=TransformStamped();correction.header.frame_id='glim_map';correction.child_frame_id='glim_odom'
        correction.header.stamp=Time(sec=2);correction.transform.rotation.w=1.;correction.transform.translation.x=10.
        node.backend_transforms(TFMessage(transforms=[correction]))
        matrix,delta=node.display_transform('glim_map',Time(sec=1),global_map=True)
        assert matrix[0,3]==pytest.approx(-7.) and delta==1.
        msg=PointCloud2();msg.header.frame_id='glim_map';msg.header.stamp=Time(sec=2)
        msg.height=1;msg.width=1;msg.point_step=12;msg.row_step=12
        msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        msg.data=np.array([[1,2,3]],dtype='<f4').tobytes()
        node.pending['registered']=msg;node.display()
        assert 'registered' in node.store.snapshot() and 'registered' not in node.errors
        assert '/uav001/ground_truth/odometry' not in topics
    finally:node.destroy_node();rclpy.shutdown()
