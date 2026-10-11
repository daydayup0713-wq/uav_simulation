#!/usr/bin/python3
"""Independent raw-scan motion check against recorded evaluation poses/geometry.

Only this evaluator reads simulator geometry and truth. No estimator subscribes
to either input. Compare per-beam registration with a rigid scan-start pose.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def surface_distance(points, boxes):
    result=np.full(len(points),np.inf)
    for lower,upper in boxes:
        outside=np.linalg.norm(np.maximum(np.maximum(lower-points,points-upper),0),axis=1)
        inside=np.minimum(points-lower,upper-points).min(axis=1)
        result=np.minimum(result,np.where(outside>0,outside,np.abs(inside)))
    return result


def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('dataset',type=Path);args=p.parse_args()
    metadata=json.loads((args.dataset/'dataset.json').read_text())
    c=metadata['calibration']
    # Archived configuration is self-contained; never depend on a live run path.
    geometry=json.loads((args.dataset/'configuration/scene/sensor-geometry.json').read_text())
    boxes=np.array(geometry['boxes'])
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.dataset/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    poses,clouds=[],[];scan_index=0;image_saved=False
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/uav001/ground_truth/odometry':
            msg=deserialize_message(data,get_message('nav_msgs/msg/Odometry'))
            t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            v,q=msg.pose.pose.position,msg.pose.pose.orientation
            poses.append([t,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
        elif topic=='/uav001/lidar/points':
            scan_index+=1
            if scan_index%10==0:
                clouds.append(deserialize_message(data,get_message('sensor_msgs/msg/PointCloud2')))
        elif topic=='/uav001/camera/image_raw' and not image_saved:
            from PIL import Image
            msg=deserialize_message(data,get_message('sensor_msgs/msg/Image'))
            Image.frombytes('RGB',(msg.width,msg.height),bytes(msg.data)).save(args.dataset/'camera-sample.png')
            image_saved=True
    poses=np.asarray(poses);rotation=Slerp(poses[:,0],Rotation.from_quat(poses[:,4:]))
    extrinsic=Rotation.from_euler('xyz',c['lidar']['rpy']).as_matrix();offset=np.array(c['lidar']['xyz'])
    motion_errors,rigid_errors=[],[];moving_scans=0;speeds=[]
    for msg in clouds:
        fields={f.name:f.offset for f in msg.fields}
        values={name:np.ndarray((msg.width,),dtype='<f4',buffer=msg.data,offset=fields[name],strides=(msg.point_step,))[::8]
                for name in ('x','y','z','time')}
        xyz=np.column_stack([values[name] for name in 'xyz']);valid=np.isfinite(xyz).all(axis=1)
        xyz=xyz[valid];start=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;times=start+values['time'][valid]
        if not len(times) or start<poses[0,0] or times.max()>poses[-1,0]:continue
        def positions(ts):return np.column_stack([np.interp(ts,poses[:,0],poses[:,i]) for i in (1,2,3)])
        speed=float(np.linalg.norm(positions([start+.05])[0]-positions([start])[0])/.05)
        speeds.append(speed)
        if speed<.2:continue
        local=xyz@extrinsic.T+offset
        moving=np.einsum('nij,nj->ni',rotation(times).as_matrix(),local)+positions(times)
        rigid=local@rotation([start]).as_matrix()[0].T+positions([start])[0]
        motion_errors.extend(surface_distance(moving,boxes));rigid_errors.extend(surface_distance(rigid,boxes));moving_scans+=1
    corrected=float(np.percentile(motion_errors,95)) if motion_errors else None
    rigid=float(np.percentile(rigid_errors,95)) if rigid_errors else None
    passed=moving_scans>=5 and corrected is not None and corrected<.035 and rigid>corrected*1.3
    report={'passed':passed,'moving_scans':moving_scans,'evaluated_points':len(motion_errors),
            'per_beam_surface_p95_m':corrected,'rigid_start_surface_p95_m':rigid,
            'maximum_observed_speed_mps':max(speeds,default=0.),'pose_sample_hz':25,
            'scope':'independent evaluation only; no truth/geometry used by localization'}
    (args.dataset/'timed-measurement-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
