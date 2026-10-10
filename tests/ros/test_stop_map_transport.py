"""Use actual bounded ROS wire data and archive the admitted braking map."""
import time,json
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')


def test_bridge_receives_observed_stop_mask_and_archives_admitted_curve(monkeypatch,tmp_path,isolated_ros_domain):
    from uav_lab_bridge.node import Bridge
    from uav_lab_bridge.continuous_trajectory import State,Trajectory
    from uav_lab_interfaces.msg import CollisionSnapshot
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    monkeypatch.setenv('LAB_RUN_DIR',str(tmp_path));monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros'))
    rclpy.init(args=['--ros-args','-p','navigation_required:=true'],domain_id=isolated_ros_domain)
    node=Bridge();source=rclpy.create_node('stop_map_source')
    publisher=source.create_publisher(CollisionSnapshot,'/uav001/navigation/collision_snapshot',qos_profile_sensor_data)
    clock=source.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    try:
        end=time.monotonic()+1.
        while (node.stop_map.snapshot is None or node.get_clock().now().nanoseconds==0) and time.monotonic()<end:
            stamp=Clock();stamp.clock.sec=10;clock.publish(stamp)
            message=CollisionSnapshot();message.header.frame_id='odom';message.header.stamp.sec=10
            message.lower=[-2.5,-2.5,0.];message.resolution=.1;message.shape=[50,50,50]
            message.envelope=[.65,.65,.55];message.version=1;message.free=np.ones(125000,dtype=np.uint8).tobytes()
            publisher.publish(message);rclpy.spin_once(node,timeout_sec=.02)
        assert node.stop_map.snapshot is not None and node.get_clock().now().nanoseconds==10000000000
        now=time.monotonic();policy=node.policy
        policy.update(now,position=(0.,0.,2.),yaw=0.,valid=True,armed=True,offboard=True,landed=False)
        policy.reference=State(np.array([0.,0.,2.]),np.array([.5,0.,0.]),np.zeros(3))
        policy.trajectory=Trajectory.generate([[0.,0.,2.],[2.,0.,2.]])
        policy.sim_time=10.;policy.state='MOVING';policy.hold(now)
        assert policy.trajectory_id=='operator-stop'
        events=[json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
        event=next(row for row in events if row['event']=='stop_admission')
        assert event['source_stamp']==10. and event['sim_ns']==10000000000
        with np.load(tmp_path/event['map_artifact']) as data:assert data['free'].all() and data['free'].size==125000
    finally:
        source.destroy_node();node.destroy_node();rclpy.shutdown()
