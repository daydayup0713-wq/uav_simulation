"""Real ROS message serialization checks; run via scripts/env.sh pytest."""
import time
import pytest
pytest.importorskip('uav_lab_interfaces.action')
import rclpy
from uav_lab_bridge.node import Bridge
from px4_msgs.msg import VehicleLocalPosition

@pytest.fixture(autouse=True)
def isolated_ros_contract_domain(monkeypatch, tmp_path, isolated_ros_domain):
    # Message serialization tests create fake /fmu publishers. Never allow
    # them onto the live lab domain or into its evidence directory.
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    monkeypatch.setenv('LAB_TEST_DOMAIN', str(isolated_ros_domain))

def initialize_test_context():
    import os
    domain = int(os.environ['LAB_TEST_DOMAIN'])
    rclpy.init(domain_id=domain, args=['--ros-args', '-r', '__ns:=/bridge_contract_test'])


@pytest.mark.parametrize('loss', ['cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_gnss_pos','stale_flags'])
def test_fusion_loss_stops_active_offboard_and_latches_after_recovery(loss):
    initialize_test_context();bridge=Bridge()
    try:
        bridge.external_enabled=True
        now=time.monotonic()
        bridge.policy.update(now,position=(0.,0.,2.),yaw=0.,valid=True,armed=True,offboard=True,landed=False)
        bridge.policy.state='HOLDING';bridge.policy.streaming=True;bridge.policy.setpoint=(0.,0.,2.)
        bridge.external.observe_quality(True,now)
        bridge.external.observe_sample({'source_ns':1},now)  # Future to the paused ROS clock; no new EV publication.
        bridge.fusion_flags={k:k.startswith('cs_ev_') for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_gnss_pos','cs_gnss_vel','cs_gps_hgt','cs_gnss_yaw')}
        bridge.fusion_received=now
        bridge.pump();assert bridge.policy.streaming
        if loss=='stale_flags': bridge.fusion_received=now-2
        else: bridge.fusion_flags[loss]=not bridge.fusion_flags[loss]
        bridge.pump()
        assert bridge.policy.state=='FAILSAFE' and not bridge.policy.streaming
        bridge.fusion_flags.update(cs_ev_pos=True,cs_ev_hgt=True,cs_ev_yaw=True,cs_gnss_pos=False)
        bridge.fusion_received=time.monotonic();bridge.pump()
        assert bridge.policy.state=='FAILSAFE' and not bridge.external_ready(time.monotonic())
    finally:
        bridge.destroy_node();rclpy.shutdown()

def test_delayed_telemetry_callback_cannot_overtake_earlier_source_time(monkeypatch):
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    entered=threading.Event()
    original=Bridge.position_callback
    def delayed(self,msg):
        if msg.timestamp==3_000_000:
            entered.set();time.sleep(.25)
        original(self,msg)
    monkeypatch.setattr(Bridge,'position_callback',delayed)
    initialize_test_context()
    bridge=Bridge();publisher_node=rclpy.create_node('ordered_telemetry_test')
    publisher=publisher_node.create_publisher(VehicleLocalPosition,'/fmu/out/vehicle_local_position_v1',qos_profile_sensor_data)
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(bridge)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    try:
        end=time.monotonic()+2
        while publisher.get_subscription_count()==0 and time.monotonic()<end:time.sleep(.02)
        first=VehicleLocalPosition();first.timestamp=3_000_000;first.xy_valid=first.z_valid=True
        publisher.publish(first);assert entered.wait(2)
        second=VehicleLocalPosition();second.timestamp=3_010_000;second.xy_valid=second.z_valid=True
        publisher.publish(second)
        end=time.monotonic()+2
        while bridge.px4_clock.px4_us!=3_010_000 and time.monotonic()<end:time.sleep(.02)
        time.sleep(.3)
        assert bridge.policy.state!='FAILSAFE'
        assert bridge.px4_clock.px4_us==3_010_000
    finally:
        executor.shutdown();worker.join(2);publisher_node.destroy_node();bridge.destroy_node();rclpy.shutdown()

def test_observation_serializes_real_ros_messages_before_and_after_telemetry():
    initialize_test_context()
    bridge = Bridge()
    try:
        bridge.observe(time.monotonic())
        now = time.monotonic()
        bridge.policy.update(now, position=(1., 2., 3.), yaw=0., valid=True,
                             armed=False, offboard=False, landed=True)
        bridge.observe(now)
        assert len(bridge.path.poses) == 1
        assert bridge.path.poses[0].pose.position.x == 1.
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

def test_estimator_coordinate_reset_latches_failsafe():
    initialize_test_context()
    bridge = Bridge()
    try:
        message = VehicleLocalPosition()
        message.timestamp = 1_000_000
        message.xy_valid, message.z_valid = True, True
        bridge.position_callback(message)
        bridge.policy.streaming = True
        message.timestamp += 10_000
        message.xy_reset_counter = 1
        bridge.position_callback(message)
        assert bridge.policy.state == 'FAILSAFE'
        assert 'reset' in bridge.policy.reason
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

def test_estimator_initialization_reset_before_control_is_allowed():
    initialize_test_context()
    bridge = Bridge()
    try:
        message = VehicleLocalPosition()
        message.timestamp = 1_000_000
        bridge.position_callback(message)
        message.timestamp += 10_000
        message.xy_reset_counter = 1
        bridge.position_callback(message)
        assert bridge.policy.state != 'FAILSAFE'
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

def test_small_inflight_heading_alignment_keeps_target_but_large_reset_fails():
    initialize_test_context()
    bridge = Bridge()
    try:
        message = VehicleLocalPosition()
        message.timestamp = 1_000_000
        bridge.position_callback(message)
        bridge.policy.streaming = True
        bridge.policy.target = (1., 2., 3.)
        message.timestamp += 10_000
        message.heading_reset_counter = 1
        message.delta_heading = .003
        bridge.position_callback(message)
        assert bridge.policy.state != 'FAILSAFE'
        assert bridge.policy.target == (1., 2., 3.)
        message.timestamp += 10_000
        message.heading_reset_counter = 2
        message.delta_heading = .5
        bridge.position_callback(message)
        assert bridge.policy.state == 'FAILSAFE'
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
