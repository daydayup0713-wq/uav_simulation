"""Read-only SQLite/CDR playback. No ROS context, clock or publishers are created."""
from collections import OrderedDict
import sqlite3,struct
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from workbench_contract import recording_path
from uav_lab_tools.datasets import load_dataset
from uav_lab_experiments.observatory import read_cloud,pack_points


def source_stamp(header):
    if len(header)!=12 or header[:2] not in (b'\x00\x00',b'\x00\x01'):
        raise ValueError('recorded cloud CDR timestamp encoding unsupported')
    seconds,nanoseconds=struct.unpack_from(('>' if header[:2]==b'\x00\x00' else '<')+'iI',header,4)
    if seconds<0 or nanoseconds>=1_000_000_000:raise ValueError('recorded cloud source time invalid')
    return seconds*1_000_000_000+nanoseconds


class RecordedView:
    def __init__(self,root,identifier):
        self.directory=recording_path(root,identifier);self.identifier=identifier
        self.metadata=load_dataset(self.directory);self.frames=[];poses=[];self.cache=OrderedDict()
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
        for path in sorted((self.directory/'bag').glob('*.db3')):
            with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as connection:
                types={name:(index,kind) for index,name,kind in connection.execute('SELECT id,name,type FROM topics')}
                if '/uav001/lidar/points' not in types or '/uav001/odometry' not in types:continue
                cloud,kind=types['/uav001/lidar/points'];odom,otype=types['/uav001/odometry']
                if kind!='sensor_msgs/msg/PointCloud2' or otype!='nav_msgs/msg/Odometry':raise ValueError('recorded display topic type mismatch')
                # Only fetch the fixed CDR Header timestamp prefix while indexing;
                # acquisition source time determines ordering and pose support.
                self.frames.extend((source_stamp(header),path,index) for index,header in connection.execute(
                    'SELECT id,substr(data,1,12) FROM messages WHERE topic_id=? ORDER BY timestamp',(cloud,)))
                for raw, in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(odom,)):
                    message=deserialize_message(raw,Odometry);p,q=message.pose.pose.position,message.pose.pose.orientation
                    if message.header.frame_id!='odom' or message.child_frame_id!='base_link':continue
                    poses.append([message.header.stamp.sec+message.header.stamp.nanosec/1e9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                if len(self.frames)>72000 or len(poses)>72000:raise ValueError('recording exceeds bounded two-hour display index')
        if not self.frames or len(poses)<2:raise ValueError('recorded raw points and source odometry unavailable')
        self.poses=np.asarray(sorted(poses),dtype=float)
        if not np.isfinite(self.poses).all() or np.any(np.diff(self.poses[:,0])<=0):raise ValueError('recorded source odometry time/values invalid')
        selected=[];last=-float('inf');self.unshown_source_frames=0
        for entry in sorted(self.frames):
            source=entry[0]/1e9
            upper=min(int(np.searchsorted(self.poses[:,0],source,side='right')),len(self.poses)-1);lower=max(0,upper-1)
            left,right=self.poses[lower,0],self.poses[upper,0]
            if not left<=source<=right or not 0<right-left<=.3:
                self.unshown_source_frames+=1;continue
            if entry[0]-last>=200_000_000:selected.append(entry);last=entry[0]
        self.frames=selected
        if not self.frames or len(self.frames)>36000:raise ValueError('recorded clouds have no bounded source-time pose support')
        lidar=self.metadata['calibration']['lidar'];self.extrinsic=np.eye(4)
        self.extrinsic[:3,:3]=Rotation.from_euler('xyz',lidar['rpy']).as_matrix();self.extrinsic[:3,3]=lidar['xyz']

    def info(self):
        return {'identifier':self.identifier,'frames':len(self.frames),'display_hz':5,'ros_publications':0,
            'unshown_source_frames':self.unshown_source_frames,'unshown_reason':'outside actual odometry source support or pose bracket gap >300ms; recording unchanged',
            'duration_s':(self.frames[-1][0]-self.frames[0][0])/1e9,
            'frame_times_s':[(f[0]-self.frames[0][0])/1e9 for f in self.frames],
            'actual_path':self.poses[::max(1,len(self.poses)//10000),1:4].tolist(),
            'scope':'recorded raw points transformed by interpolated PX4 odometry; file-only display'}

    def transform(self,source):
        upper=int(np.searchsorted(self.poses[:,0],source,side='right'));upper=min(upper,len(self.poses)-1);lower=max(0,upper-1)
        left,right=self.poses[lower],self.poses[upper]
        if not left[0]<=source<=right[0] or not 0<right[0]-left[0]<=.3:raise ValueError('recorded odometry does not bracket point source time')
        fraction=(source-left[0])/(right[0]-left[0]);matrix=np.eye(4)
        matrix[:3,3]=left[1:4]*(1-fraction)+right[1:4]*fraction
        matrix[:3,:3]=Slerp([left[0],right[0]],Rotation.from_quat([left[4:],right[4:]]))([source]).as_matrix()[0]
        return matrix@self.extrinsic

    def frame(self,index):
        if not isinstance(index,int) or not 0<=index<len(self.frames):raise ValueError('replay frame index out of bounds')
        if index in self.cache:
            self.cache.move_to_end(index);return self.cache[index]
        _,path,identifier=self.frames[index]
        with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as connection:
            raw,=connection.execute('SELECT data FROM messages WHERE id=?',(identifier,)).fetchone()
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        message=deserialize_message(raw,PointCloud2)
        if message.header.frame_id!=self.metadata['calibration']['lidar']['frame']:raise ValueError('replay sensor frame differs from calibration')
        source=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        points,metadata=read_cloud(message,transform=self.transform(source),frame='odom')
        packet=pack_points(points,{**metadata,'layer':'raw','replay':True,'recording':self.identifier,
            'display_transform':'recorded source-time interpolation; no ROS publications'})
        self.cache[index]=packet
        while len(self.cache)>2:self.cache.popitem(last=False)
        return packet
