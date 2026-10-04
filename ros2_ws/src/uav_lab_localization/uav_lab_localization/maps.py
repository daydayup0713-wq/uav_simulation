"""Immutable, checksum-bound maps and geometric coarse-prior relocalization."""
import json
from pathlib import Path
import shutil
import subprocess
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from uav_lab_tools.datasets import check_files, file_hash
from .registration import validate_cloud, voxel_downsample, register, RegistrationError


def map_change_allowed(status, received, now):
    return (0 <= now-received <= .5 and status.get('armed') == 'False' and
            status.get('landed') == 'True' and status.get('fresh') == 'True')


def archive_points(points, destination, calibration, provenance, native=None):
    points = validate_cloud(points)
    destination = Path(destination)
    if destination.exists():
        raise ValueError('map destination already exists')
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = destination.with_name(destination.name+'.incomplete-'+uuid.uuid4().hex[:8])
    stage.mkdir()
    np.save(stage/'points.npy', points, allow_pickle=False)
    if native:
        shutil.copytree(native, stage/'native')
    files = {str(p.relative_to(stage)): file_hash(p) for p in stage.rglob('*') if p.is_file()}
    metadata = {'schema_version': 1, 'complete': True, 'frame': 'map',
                'calibration': calibration, 'provenance': provenance, 'files': files,
                'relocalization': 'coarse-prior point-to-plane ICP; not global kidnapped-robot recovery'}
    (stage/'map.json').write_text(json.dumps(metadata, indent=2)+'\n')
    stage.rename(destination)
    return metadata


def save_native_map(root, benchmark, destination):
    root, benchmark = Path(root), Path(benchmark)
    if not json.loads((benchmark/'report.json').read_text())['passed']:
        raise ValueError('benchmark has not passed')
    subprocess.run([str(root/'.deps/slam/install/bin/uav_map_export'),
                    str(benchmark/'dump'), str(benchmark/'points.xyz'), str(benchmark/'graph-report.json')], check=True)
    points = voxel_downsample(np.loadtxt(benchmark/'points.xyz'), 0.12)
    metadata = json.loads((benchmark/'input-manifest.json').read_text())
    report = json.loads((benchmark/'graph-report.json').read_text())
    provenance = {'input': metadata, 'benchmark_sha256': file_hash(benchmark/'report.json'),
                  'graph': report, 'backend': json.loads((benchmark/'report.json').read_text())['backend']}
    archive_points(points, destination, metadata['calibration'], provenance, benchmark/'dump')
    return report


def load_map(directory, calibration):
    directory = Path(directory)
    metadata = json.loads((directory/'map.json').read_text())
    if metadata.get('schema_version') != 1 or not metadata.get('complete') or metadata.get('frame') != 'map':
        raise ValueError('unsupported or incomplete map')
    if metadata.get('calibration') != calibration:
        raise ValueError('map calibration mismatch')
    if 'points.npy' not in metadata.get('files', {}):
        raise ValueError('map points missing from manifest')
    check_files(directory, metadata['files'])
    points = validate_cloud(np.load(directory/'points.npy', allow_pickle=False))
    return {'points': points, 'metadata': metadata, 'directory': str(directory.resolve())}


def relocalize(loaded, observed_lidar, initial_map_lidar, odom_lidar):
    initial, odom = np.asarray(initial_map_lidar), np.asarray(odom_lidar)
    result = register(voxel_downsample(observed_lidar, .15), loaded['points'], initial,
                      max_distance=.6, min_inlier_fraction=.75, max_rmse=.1)
    correction = result.transform @ np.linalg.inv(initial)
    if np.linalg.norm(correction[:3, 3]) > .75 or Rotation.from_matrix(correction[:3, :3]).magnitude() > np.deg2rad(20):
        raise RegistrationError('relocalization exceeds coarse-prior correction limits')
    return {'map_to_odom': result.transform @ np.linalg.inv(odom),
            'map_to_lidar': result.transform, 'inlier_fraction': result.inlier_fraction,
            'rmse_m': result.rmse, 'information_eigenvalues': result.information_eigenvalues.tolist()}
