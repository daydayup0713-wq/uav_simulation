import importlib,importlib.util,json,sqlite3,struct
from pathlib import Path
import numpy as np
import pytest
pytest.importorskip('rclpy')
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from uav_lab_tools.datasets import file_hash


@pytest.mark.parametrize('outside_support',[False,True])
def test_file_replay_reads_real_cdr_and_never_initializes_or_publishes_ros(monkeypatch,tmp_path,outside_support):
    assert importlib.util.find_spec('web_replay'),'publication-free recorded point cloud playback missing'
    import rclpy
    monkeypatch.setattr(rclpy,'init',lambda **kw:pytest.fail('offline replay initialized ROS'))
    directory=tmp_path/'recordings/fixture';bag=directory/'bag';bag.mkdir(parents=True)
    path=bag/'bag_0.db3';connection=sqlite3.connect(path)
    connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    connection.executemany('INSERT INTO topics VALUES (?,?,?)',[(1,'/uav001/lidar/points','sensor_msgs/msg/PointCloud2'),(2,'/uav001/odometry','nav_msgs/msg/Odometry')])
    for index,(source,x) in enumerate([(1.9,0.),(2.1,2.)]):
        message=Odometry();message.header.frame_id='odom';message.child_frame_id='base_link'
        message.header.stamp.sec=int(source);message.header.stamp.nanosec=round((source-int(source))*1e9)
        message.pose.pose.position.x=x;message.pose.pose.orientation.w=1.
        connection.execute('INSERT INTO messages VALUES (?,?,?,?)',(index+1,2,round(source*1e9),serialize_message(message)))
    cloud=PointCloud2();cloud.header.frame_id='lidar_link';cloud.header.stamp.sec=2
    cloud.width=cloud.height=1;cloud.point_step=16;cloud.row_step=16
    cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))]
    cloud.data=struct.pack('<ffff',1.,0.,2.,.5)
    connection.execute('INSERT INTO messages VALUES (?,?,?,?)',(3,1,2_000_000_000,serialize_message(cloud)))
    if outside_support:
        cloud.header.stamp.sec=1
        connection.execute('INSERT INTO messages VALUES (?,?,?,?)',(4,1,1_000_000_000,serialize_message(cloud)))
        cloud.header.stamp.sec=3
        connection.execute('INSERT INTO messages VALUES (?,?,?,?)',(5,1,9_000_000_000,serialize_message(cloud)))
    connection.commit();connection.close()
    (directory/'dataset.json').write_text(json.dumps({'complete':True,'calibration':{'lidar':{'frame':'lidar_link','xyz':[.2,0,0],'rpy':[0,0,0]}},
        'configuration_sha256':{},'bag_sha256':{'bag_0.db3':file_hash(path)}}))
    reader=importlib.import_module('web_replay').RecordedView(tmp_path,'fixture')
    packet=reader.frame(0);length=struct.unpack_from('<I',packet)[0];offset=4+((length+3)//4)*4
    header=json.loads(packet[4:4+length]);values=np.frombuffer(packet,dtype='<f4',offset=offset)
    assert header['frame']=='odom' and header['layer']=='raw'
    assert values[0]==pytest.approx(2.2) and reader.info()['frames']==1
    assert reader.info()['ros_publications']==0
    assert reader.info()['unshown_source_frames']==(2 if outside_support else 0)
    with pytest.raises(ValueError,match='frame'):reader.frame(10000)
