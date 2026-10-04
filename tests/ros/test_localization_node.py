import time
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from uav_lab_localization.localization_node import LocalizationNode


def test_lio_ingress_drops_blank_scans_and_stays_closed_after_source_timeout(monkeypatch, isolated_ros_domain):
    from sensor_msgs.msg import PointCloud2, PointField
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    rclpy.init(domain_id=isolated_ros_domain)
    node = LocalizationNode(); client = rclpy.create_node('lidar_ingress_test')
    received = []
    client.create_subscription(PointCloud2, '/uav001/localization/input/points', received.append, qos_profile_sensor_data)
    try:
        end = time.monotonic()+.4
        while time.monotonic()<end: rclpy.spin_once(client, timeout_sec=.01)
        msg = PointCloud2(); msg.header.frame_id = 'lidar_link'; msg.header.stamp.sec = 10
        msg.height = 1; msg.width = 1000; msg.point_step = 12; msg.row_step = 12000
        msg.fields = [PointField(name=n, datatype=7, count=1, offset=i*4) for i,n in enumerate('xyz')]
        msg.data = np.zeros((1000,3), dtype='<f4').tobytes()
        node.points(msg)
        rclpy.spin_once(client, timeout_sec=.05)
        assert not received and node.rejected_scans == 1
        msg.data = np.random.default_rng(2).uniform(-3,3,(1000,3)).astype('<f4').tobytes()
        node.points(msg)
        end = time.monotonic()+.5
        while time.monotonic()<end and not received: rclpy.spin_once(client, timeout_sec=.01)
        assert received and received[0].header.stamp.sec == 10
        node.quality.was_ready = True
        assert node.quality.check(12, time.monotonic()+2)['state'] == 'FAILED'
        msg.header.stamp.sec = 12; node.points(msg)
        rclpy.spin_once(client, timeout_sec=.05)
        assert len(received) == 1  # Failure is latched even when valid scans return.
    finally:
        client.destroy_node(); node.destroy_node(); rclpy.shutdown()


def test_slow_map_io_does_not_block_clock_or_quality_diagnostics(monkeypatch, tmp_path, isolated_ros_domain):
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from diagnostic_msgs.msg import DiagnosticArray
    from uav_lab_interfaces.srv import LoadMap
    import uav_lab_localization.localization_node as implementation
    from uav_lab_localization.maps import archive_points
    from test_registration import corner
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    rclpy.init(domain_id=isolated_ros_domain)
    node = LocalizationNode(); client = rclpy.create_node('map_io_contract_test')
    archive_points(corner(), tmp_path/'map', node.calibration, {})
    entered, release = threading.Event(), threading.Event()
    original = implementation.load_map
    def delayed(*args):
        entered.set(); release.wait(3); return original(*args)
    monkeypatch.setattr(implementation, 'load_map', delayed)
    diagnostics = []
    client.create_subscription(DiagnosticArray, '/uav001/localization/diagnostics', diagnostics.append, 10)
    clock = client.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    service = client.create_client(LoadMap, '/uav001/localization/load_map')
    executor = MultiThreadedExecutor(num_threads=2); executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True); worker.start()
    try:
        assert service.wait_for_service(timeout_sec=2)
        end = time.monotonic()+.5
        while time.monotonic()<end: rclpy.spin_once(client, timeout_sec=.01)
        node.flight_status = {'armed':'False','landed':'True','fresh':'True'}
        node.flight_received = time.monotonic()
        service.call_async(LoadMap.Request(directory=str(tmp_path/'map')))
        assert entered.wait(2)
        end = time.monotonic()+.6
        while time.monotonic()<end:
            tick = Clock(); tick.clock.sec = 20; clock.publish(tick)
            rclpy.spin_once(client, timeout_sec=.01)
        assert any(d.header.stamp.sec == 20 for d in diagnostics)
    finally:
        release.set(); executor.shutdown(); worker.join(2)
        client.destroy_node(); node.destroy_node(); rclpy.shutdown()


def test_real_ros_publishes_body_twist_and_never_subscribes_truth(monkeypatch, isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    rclpy.init(domain_id=isolated_ros_domain)
    node = client = None
    try:
        node = LocalizationNode()
        client = rclpy.create_node('localization_contract_test')
        output = []
        client.create_subscription(Odometry, '/uav001/localization/odometry', output.append, 20)
        clock = client.create_publisher(Clock, '/clock', qos_profile_sensor_data)
        end = time.monotonic()+1
        while time.monotonic()<end:
            tick = Clock(); tick.clock.sec = 10; clock.publish(tick)
            rclpy.spin_once(node, timeout_sec=.01)
        # Genuine ROS messages serialized by the node; exercise normalization at the boundary.
        for i in range(25):
            t = 10+i*.005
            node.quality.observe_source('imu', t, time.monotonic())
            node.quality.observe_source('points', t, time.monotonic())
            msg = Odometry()
            msg.header.frame_id = 'lio_odom'; msg.child_frame_id = 'lio_base_link'
            msg.header.stamp.sec = 10; msg.header.stamp.nanosec = i*5_000_000
            msg.pose.pose.orientation.z = msg.pose.pose.orientation.w = 2**-.5
            msg.twist.twist.linear.x = 1.
            node.odometry(msg)
            rclpy.spin_once(client, timeout_sec=.01)
        assert output
        assert np.allclose([output[-1].twist.twist.linear.x, output[-1].twist.twist.linear.y], [0, -1])
        assert output[-1].pose.covariance[0] == .01
        assert not any('ground_truth' in t for t, _ in node.get_topic_names_and_types()
                       if node.count_subscribers(t) and t.startswith('/uav001/ground_truth'))
        assert not any(t.startswith('/fmu/in/') for t, _ in node.get_topic_names_and_types())
    finally:
        if client: client.destroy_node()
        if node: node.destroy_node()
        rclpy.shutdown()
