"""Immutable input snapshots and body-pose export for upstream VINS graphs."""
import shutil
from pathlib import Path
import numpy as np
from uav_lab_tools.datasets import file_hash
from uav_lab_experiments.backend_contract import normalized_pose


def read_graph(path):
    rows=np.loadtxt(path,ndmin=2)
    if rows.shape[1]!=26 or not 1<=len(rows)<=10000 or not np.isfinite(rows).all():
        raise ValueError('invalid VINS pose graph')
    ids=rows[:,0]
    if np.any(ids<0) or np.any(ids!=ids.astype(int)) or len(set(ids))!=len(ids):
        raise ValueError('invalid VINS keyframe IDs')
    for columns in (slice(8,12),slice(12,16)):
        if np.any(np.abs(np.linalg.norm(rows[:,columns],axis=1)-1)>.01):
            raise ValueError('invalid VINS pose graph quaternion')
    return rows


def snapshot_vins_map(source_graph, destination):
    source_graph=Path(source_graph).resolve();destination=Path(destination).resolve()
    if not str(source_graph).endswith('pose_graph.txt'):
        raise ValueError('VINS graph filename must end in pose_graph.txt')
    rows=read_graph(source_graph);prefix=str(source_graph).removesuffix('pose_graph.txt')
    sources=[source_graph]
    for index in rows[:,0].astype(int):
        sources.extend(Path(prefix+str(index)+suffix) for suffix in ('_briefdes.dat','_keypoints.txt'))
        image=Path(prefix+str(index)+'_image.png')
        if image.exists():sources.append(image)
    if any(not p.is_file() for p in sources):raise ValueError('VINS keyframe artifact missing')
    if destination.exists() and any(destination.iterdir()):raise ValueError('VINS map snapshot destination occupied')
    destination.mkdir(parents=True,exist_ok=True)
    artifacts={}
    for path in sources:
        target=destination/str(path).removeprefix(prefix)
        if target==path:raise ValueError('VINS map input cannot be overwritten')
        artifacts[str(path)]=file_hash(path);shutil.copy2(path,target)
    return {'source_keyframes':len(rows),'source_graph':str(source_graph),'source_artifacts':artifacts,
            'load_scope':'owned copy; input map never overwritten'}


def graph_body_trajectory(path, imu, first_index=0):
    rows=read_graph(path);current=rows[rows[:,0]>=first_index]
    poses=[normalized_pose(row[1],row[5:8],row[[13,14,15,12]],imu) for row in current]
    cross_session=int(np.count_nonzero((current[:,16]>=0)&(current[:,16]<first_index)))
    return poses,cross_session
