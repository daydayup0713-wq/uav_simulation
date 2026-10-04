"""Fixed-input benchmark: algorithm bag contains only lidar and raw inertial observations."""
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation

from uav_lab_tools.datasets import load_dataset, file_hash, check_replay_domain
from .evaluation import evaluate
from .registration import register, voxel_downsample, RegistrationError

INPUTS = {'/uav001/lidar/points': 'sensor_msgs/msg/PointCloud2',
          '/uav001/imu/data': 'sensor_msgs/msg/Imu'}
TRUTH = '/uav001/ground_truth/odometry'


def cloud_xyz(message):
    fields = {f.name: f for f in message.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in 'xyz'):
        raise ValueError('expected FLOAT32 xyz point fields')
    if message.row_step < message.width * message.point_step or len(message.data) != message.height * message.row_step:
        raise ValueError('invalid point cloud layout')
    dtype = np.dtype({'names': list('xyz'), 'formats': [('>' if message.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[n].offset for n in 'xyz'], 'itemsize': message.point_step})
    values = np.ndarray((message.height, message.width), dtype=dtype, buffer=bytes(message.data),
                        strides=(message.row_step, message.point_step))
    points = np.column_stack([values[n].ravel() for n in 'xyz']).astype(float)
    # No-return lidar rays are unavailable measurements, not fabricated points.
    return points[np.isfinite(points).all(axis=1)]


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def read_dataset(directory, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(directory)/'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if not set(topics) <= types.keys():
        raise ValueError('dataset required topics missing')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, received_ns = reader.read_next()
        message = deserialize_message(data, get_message(types[topic]))
        yield topic, message, data, received_ns


def extract(directory, output):
    import rosbag2_py
    metadata = load_dataset(directory)
    rows, truth, scans, counts = [], [], [], {t: 0 for t in INPUTS}
    for topic, message, data, _ in read_dataset(directory, list(INPUTS)+[TRUTH]):
        t = stamp(message)
        if topic == TRUTH:
            p, q = message.pose.pose.position, message.pose.pose.orientation
            truth.append([t/1e9, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        else:
            if topic.endswith('/data') and message.orientation_covariance[0] != -1:
                raise ValueError('IMU orientation must be unavailable')
            rows.append((t, topic, data))
            counts[topic] += 1
            if topic.endswith('/points'):
                scans.append((t/1e9, cloud_xyz(message)))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/'algorithm-input'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for topic, typename in INPUTS.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename, serialization_format='cdr'))
    for t, topic, data in sorted(rows, key=lambda r: r[0]):
        writer.write(topic, data, t)
    del writer
    truth = np.asarray(truth)
    np.savetxt(output/'truth.tum', truth, fmt='%.9f')
    provenance = {'dataset': str(Path(directory).resolve()), 'input_topics': INPUTS,
                  'bag_sha256': metadata['bag_sha256'], 'calibration': metadata['calibration'],
                  'counts': counts, 'ordering': 'original source timestamps, stable chronological order'}
    (output/'input-manifest.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return truth, scans


def icp_baseline(scans):
    trajectory, pose, previous = [], np.eye(4), None
    failures = []
    for t, points in scans:
        points = voxel_downsample(points, 0.2)
        if previous is not None:
            try:
                pose = pose @ register(points, previous, max_rmse=0.15).transform
            except RegistrationError as error:
                failures.append({'stamp': t, 'reason': str(error)})
                break  # Do not silently bridge a localization gap with an identity step.
        trajectory.append([t, *pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_quat()])
        previous = points
    return np.asarray(trajectory), failures


def run_benchmark(root, directory, output, domain=77, learning=True):
    root, output = Path(root), Path(output)
    if output.exists():
        raise ValueError('benchmark output already exists')
    active = []
    for manifest in (root/'.runtime').glob('*/manifest.json'):
        if (manifest.parent/'ready').exists():
            active.append(int(json.loads(manifest.read_text())['environment']['ROS_DOMAIN_ID']))
    check_replay_domain(domain, active)
    manifest_path = root/'.deps/slam/install/build-manifest.json'
    backend = json.loads(manifest_path.read_text())
    output.mkdir(parents=True)
    truth, scans = extract(directory, output)
    config = root/'configs/slam'
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), OMP_NUM_THREADS='2')
    start = time.monotonic()
    command = [str(root/'.deps/slam/install/lib/glim_ros/glim_rosbag'),
               str(output/'algorithm-input'), '--ros-args', '-p', 'config_path:='+str(config),
               '-p', 'auto_quit:=true', '-p', 'dump_path:='+str(output/'dump'), '-p', 'debug:=true']
    with (output/'backend.log').open('w') as log:
        subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=600)
    elapsed = time.monotonic()-start
    estimate = np.loadtxt(output/'dump/odom_imu.txt', ndmin=2)
    # Exclude the explicit initialization interval, never hide later missing outputs.
    expected = np.array([t for t, _ in scans if t >= scans[0][0]+3.0])
    report = {'backend': backend, 'runtime_s': elapsed,
              'config_sha256': {p.name: file_hash(p) for p in config.glob('*.json')},
              'lio': evaluate(estimate, truth, expected)}
    corrected = np.loadtxt(output/'dump/traj_imu.txt', ndmin=2)
    report['slam'] = evaluate(corrected, truth, expected)
    report['passed'] = (report['lio']['ate_rmse_m'] <= 0.3 and
                        report['lio']['attitude_rmse_deg'] <= 5 and report['lio']['coverage'] >= 0.95)
    if learning:
        start = time.monotonic()
        baseline, failures = icp_baseline(scans)
        np.savetxt(output/'icp_lidar.tum', baseline, fmt='%.9f')
        # The baseline estimates lidar, so compare against the actual lidar pose only in the evaluator.
        lidar_truth = truth.copy()
        calib = json.loads((output/'input-manifest.json').read_text())['calibration']
        lidar_truth[:, 1:4] += Rotation.from_quat(truth[:, 4:]).apply(calib['lidar']['xyz'])
        supported = baseline[(baseline[:, 0] >= truth[0, 0]) & (baseline[:, 0] <= truth[-1, 0])]
        report['learning_icp'] = evaluate(supported, lidar_truth, [t for t, _ in scans
                                         if truth[0, 0] <= t <= truth[-1, 0]]) if len(supported) >= 3 else None
        report['learning_outside_truth_support'] = len(baseline)-len(supported)
        report['learning_failures'] = failures
        report['learning_runtime_s'] = time.monotonic()-start
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report
