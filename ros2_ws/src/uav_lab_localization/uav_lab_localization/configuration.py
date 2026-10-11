"""Generate a CPU GLIM configuration from the actual platform calibration."""
import json
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read_json_comments(path):
    pattern = r'"(?:\\.|[^"\\])*"|/\*[\s\S]*?\*/|//[^\n]*'
    text = re.sub(pattern, lambda m: m[0] if m[0].startswith('"') else '',
                  Path(path).read_text())
    return json.loads(text)


def prepare_slam(root, destination, sensors=None, use_timed_scans=False):
    root, destination = Path(root), Path(destination)
    sensors = sensors or json.loads((root / 'configs/sensors.json').read_text())
    if use_timed_scans and sensors['lidar'].get('measurement_time')!='per_beam':
        raise ValueError('measured beam times required for timed GLIM')
    if not use_timed_scans and sensors['lidar'].get('scan_timing', 'synchronous') != 'synchronous':
        raise ValueError('only synchronous generic lidar is supported')
    source = root / '.deps/glim/config'
    if not source.is_dir():
        source = root / 'configs/slam'
    configs = {p.name: read_json_comments(p) for p in source.glob('*.json')}
    if not configs:
        raise ValueError('pinned GLIM configuration templates missing')
    cfg = configs['config.json']['global']
    cfg.update(config_odometry='config_odometry_cpu.json',
               config_sub_mapping='config_sub_mapping_passthrough.json',
               config_global_mapping='config_global_mapping_pose_graph.json')
    lidar, imu = sensors['lidar'], sensors['imu']
    rl = Rotation.from_euler('xyz', lidar['rpy'])
    ri = Rotation.from_euler('xyz', imu['rpy'])
    translation = rl.inv().apply(np.array(imu['xyz']) - np.array(lidar['xyz']))
    configs['config_sensors.json']['sensors'].update(
        T_lidar_imu=translation.tolist() + (rl.inv() * ri).as_quat().tolist(),
        global_shutter_lidar=not use_timed_scans, autoconf_perpoint_times=False, ring_field='',
        imu_acc_noise=0.02, imu_gyro_noise=0.002)
    configs['config_ros.json']['glim_ros'].update(
        imu_topic='/uav001/localization/input/imu', points_topic='/uav001/localization/input/points',
        image_topic='/uav001/localization/unused_image', acc_scale=1.0, ang_scale=1.0,
        imu_frame_id='lio_base_link', base_frame_id='lio_base_link',
        lidar_frame_id='lio_lidar', odom_frame_id='lio_odom', map_frame_id='lio_map',
        publish_imu2lidar=False, tf_time_offset=0.0,
        extension_modules=['librviz_viewer.so'])
    configs['config_preprocess.json']['preprocess'].update(
        downsample_resolution=0.15, random_downsample_target=4000, distance_near_thresh=0.35,
        distance_far_thresh=20.0, num_threads=2)
    configs['config_odometry_cpu.json']['odometry_estimation'].update(
        ivox_resolution=0.3, ivox_min_dist=0.05, num_threads=2)
    configs['config_sub_mapping_passthrough.json']['sub_mapping'].update(
        keyframe_update_interval_trans=0.3, keyframe_update_interval_rot=0.1,
        max_num_keyframes=8, adaptive_max_num_voxels=-1,
        submap_voxel_resolution=0.2, min_dist_in_voxel=0.1,
        submap_target_num_points=12000)
    configs['config_global_mapping_pose_graph.json']['global_mapping'].update(
        min_travel_dist=3.0, max_neighbor_dist=2.0, min_inliear_fraction=0.65,
        subsample_target=3000, vgicp_voxel_resolution=0.3,
        gicp_max_correspondence_dist=0.5, num_threads=2)
    destination.mkdir(parents=True, exist_ok=True)
    for name, value in configs.items():
        (destination / name).write_text(json.dumps(value, indent=2) + '\n')
    return destination
