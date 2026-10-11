"""Real ROS map node consumes timed beams with source-time interpolation."""
import json,time
from pathlib import Path
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')


def test_timed_navigation_node_integrates_measured_beam_origins(monkeypatch,tmp_path,isolated_ros_domain):
    from uav_lab_navigation.node import NavigationNode
    from uav_lab_navigation.local_map import RollingMap
    from sensor_msgs.msg import PointCloud2,PointField
    from uav_lab_interfaces.msg import CollisionSnapshot
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    directory=tmp_path/'configuration';directory.mkdir()
    calibration=json.loads(Path('configs/sensors-livox.json').read_text())
    (directory/'calibration.json').write_text(json.dumps(calibration))
    monkeypatch.setenv('LAB_RUN_DIR',str(tmp_path));monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros'))
    rclpy.init(args=['--ros-args','-p','rolling_map:=true'],domain_id=isolated_ros_domain)
    node=NavigationNode();publisher=rclpy.create_node('rolling_map_clock')
    clock=publisher.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    received=[]
    publisher.create_subscription(CollisionSnapshot,'/uav001/navigation/collision_snapshot',received.append,qos_profile_sensor_data)
    try:
        assert isinstance(node.grid,RollingMap),'navigation still uses a fixed world grid'
        end=time.monotonic()+.5
        while time.monotonic()<end:
            msg=Clock();msg.clock.sec=10;msg.clock.nanosec=200000000;clock.publish(msg)
            rclpy.spin_once(node,timeout_sec=.01)
        node.alignment=np.eye(4);node.quality_ready=True
        node.quality_at=node.pose_at=time.monotonic()
        node.history.add(10.,[0.,0.,1.],[0.,0.,0.,1.]);node.history.add(10.2,[.2,0.,1.],[0.,0.,0.,1.])
        dtype=np.dtype({'names':['x','y','z','intensity','time','line'],
            'formats':['<f4']*5+['<u2'],'offsets':[0,4,8,12,16,20],'itemsize':24})
        points=np.zeros(2000,dtype=dtype);points['x']=1.5;points['intensity']=1.
        points['y']=np.random.default_rng(5).uniform(-1.,1.,len(points))
        points['time']=np.linspace(0.,.0999,len(points));points['line']=np.arange(len(points))%4
        cloud=PointCloud2(height=1,width=len(points),point_step=24,row_step=len(points)*24,data=points.tobytes())
        cloud.header.frame_id='lidar_link';cloud.header.stamp.sec=10
        cloud.fields=[PointField(name=n,offset=dtype.fields[n][1],datatype=4 if n=='line' else 7,count=1) for n in dtype.names]
        node.on_cloud(cloud);node.map_cycle()
        assert node.map_stamp==pytest.approx(10.0999,abs=1e-6)
        assert node.grid.version>0 and node.report()['ready']
        assert (node.grid.score>0).any()
        end=time.monotonic()+1.
        while not received and time.monotonic()<end:rclpy.spin_once(publisher,timeout_sec=.02)
        assert received,'source-time map was not delivered to the flight stopping gate'
        message=received[-1]
        assert message.header.frame_id=='odom' and message.header.stamp.nanosec==pytest.approx(99900000,abs=200)
        assert list(message.envelope)==pytest.approx([.65,.65,.55])
        cells=np.frombuffer(bytes(message.free),dtype=np.uint8).reshape(message.shape)
        assert np.array_equal(cells,node.grid.snapshot(node.envelope).free)
        assert not cells.all(),'unknown cells became free in the stopping mask'
    finally:
        publisher.destroy_node();node.destroy_node();rclpy.shutdown()
