"""Fixed sensor-only occupancy/planning benchmark; ideal fixtures are explicitly labeled."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from .occupancy import VoxelMap
from .planner import plan


def benchmark(directory,output,configuration=None):
    directory,output=Path(directory),Path(output)
    metadata=json.loads((directory/'manifest.json').read_text());raw=directory/'observations.npz'
    digest=hashlib.sha256(raw.read_bytes()).hexdigest()
    if metadata.get('frame')!='odom' or digest!=metadata.get('sha256'):
        raise ValueError('observation frame/checksum mismatch')
    config=configuration or {'resolution_m':.2,'lower':[-6,-6,-1],'upper':[6,6,5],
        'body_halfsize_m':[.4,.4,.15],'clearance_m':.25}
    grid=VoxelMap(config['resolution_m'],config['lower'],config['upper'])
    begun=time.monotonic()
    with np.load(raw,allow_pickle=False) as data:
        if set(data.files)!={'origins','endpoints'} or len(data['origins'])!=len(data['endpoints']):
            raise ValueError('invalid sensor observation archive')
        for origin,endpoints in zip(data['origins'],data['endpoints']):
            grid.integrate(origin,endpoints)
            grid.observe_body(origin-np.array([0,0,.16]),config['body_halfsize_m'])
    collision=grid.snapshot(np.asarray(config['body_halfsize_m'])+config['clearance_m'])
    result=plan(collision,(0,0,2),(2.5,3.6,2))
    rejected=plan(collision,(0,0,2),(1.5,0,2))
    segments_clear=result.success and all(collision.segment_clear(a,b) for a,b in zip(result.points,result.points[1:]))
    length=sum(float(np.linalg.norm(np.asarray(a)-b)) for a,b in zip(result.points,result.points[1:]))
    report={'source':metadata['source'],'input_sha256':digest,'configuration':config,
        'occupied_cells':int((grid.score>0).sum()),'observed_free_cells':int((grid.score<0).sum()),
        'map_version':grid.version,'runtime_s':time.monotonic()-begun,'plan':asdict(result),
        'rejected_goal':asdict(rejected),'length_m':length,'all_segments_clear':segments_clear,
        'passed':bool(segments_clear and not rejected.success)}
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'grid.npz',score=grid.score,seen=grid.seen,lower=grid.lower,resolution=grid.resolution)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
