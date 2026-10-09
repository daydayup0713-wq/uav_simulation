#!/usr/bin/python3
"""Convert a fixed NTU VIRAL clip; algorithm sensors and evaluator truth are separated."""
import argparse,json,shutil,hashlib
from pathlib import Path
import yaml
import rosbag2_py
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2,Imu,Image
from uav_lab_tools.datasets import file_hash
from backend_configs import public_ntu_parameters

ROOT=Path(__file__).resolve().parents[1]
SENSORS={'/os1_cloud_node1/points':('/uav001/lidar/points',PointCloud2),
         '/imu/imu':('/uav001/imu/data',Imu),'/left/image_raw':('/uav001/camera/image_raw',Image)}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=120.)
    args=p.parse_args()
    if args.output.exists() or not 60<=args.duration<=400:p.error('new output required, duration 60..400')
    args.output.mkdir(parents=True);configuration=args.output/'configuration';configuration.mkdir()
    store=get_typestore(Stores.ROS2_HUMBLE)
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    topics={'/clock':'rosgraph_msgs/msg/Clock','/uav001/ground_truth/odometry':'nav_msgs/msg/Odometry'}
    topics.update({target:kind.__module__.split('.')[0]+'/msg/'+kind.__name__ for target,kind in SENSORS.values()})
    for topic,typename in topics.items():writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typename,serialization_format='cdr'))
    counts={t:0 for t in topics};first=last=None
    with Reader(args.source) as reader:
        clip_start=reader.start_time;clip_end=clip_start+int(args.duration*1e9)
        connections=[c for c in reader.connections if c.topic in SENSORS or c.topic=='/leica/pose/relative']
        for conn,received,raw in reader.messages(connections=connections,start=clip_start,stop=clip_end):
            cdr=bytes(store.ros1_to_cdr(raw,conn.msgtype))
            if conn.topic in SENSORS:
                target,kind=SENSORS[conn.topic];msg=deserialize_message(cdr,kind)
            else:
                value=store.deserialize_cdr(cdr,conn.msgtype)
                target='/uav001/ground_truth/odometry';msg=Odometry();msg.header.frame_id='leica_relative_prism';msg.child_frame_id='prism'
                msg.header.stamp.sec=value.header.stamp.sec;msg.header.stamp.nanosec=value.header.stamp.nanosec
                for axis in 'xyz':setattr(msg.pose.pose.position,axis,getattr(value.pose.position,axis))
                # Leica is a position reference. This placeholder is excluded
                # from all attitude and rotation-error claims by report metadata.
                msg.pose.pose.orientation.w=1.;cdr=serialize_message(msg)
            t=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if first is None:first=t
            last=t
            clock=Clock();clock.clock=msg.header.stamp
            writer.write('/clock',serialize_message(clock),t);counts['/clock']+=1
            writer.write(target,cdr,t);counts[target]+=1
    del writer
    for backend in ('fast_livo2','fast_lio2'):
        params,camera,calibration=public_ntu_parameters(backend,args.output.name)
        directory=configuration if backend=='fast_livo2' else configuration/backend
        directory.mkdir(exist_ok=True)
        (directory/'parameters.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'use_sim_time':True,**params}}},sort_keys=False))
        (directory/'camera.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':camera}},sort_keys=False))
    manifest={'schema_version':1,'complete':True,'origin':'https://researchdata.ntu.edu.sg/api/access/datafile/68133',
        'source_file':str(args.source.resolve()),'source_sha256':file_hash(args.source),'sequence':'eee_01',
        'clip_received_ns':[clip_start,clip_end],'requested_duration_s':args.duration,'counts':counts,
        'calibration':calibration,
        'truth':{'kind':'position_only_prism','orientation_valid':False,'uncompensated_prism_to_imu_lever_arm':True},
        'scope':'public real-sensor clip; full sequence was not replayed; no simulation sensor audit',
        'configuration_sha256':{str(f.relative_to(configuration)):file_hash(f) for f in configuration.rglob('*') if f.is_file()},
        'bag_sha256':{f.name:file_hash(f) for f in (args.output/'bag').iterdir() if f.is_file()}}
    (args.output/'dataset.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v for k,v in manifest.items() if k not in ('bag_sha256','configuration_sha256')},indent=2))

if __name__=='__main__':main()
