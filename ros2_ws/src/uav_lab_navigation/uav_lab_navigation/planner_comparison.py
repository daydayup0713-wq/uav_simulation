"""Sequential four-method evaluation on one immutable, body-inflated sensor map."""
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.ndimage import distance_transform_edt
from uav_lab_bridge.continuous_trajectory import State
from .occupancy import CollisionMap
from .planner_backends import map_identity,plan_curve

METHODS=('astar','ego','fast_planner','gcopter')


def load_snapshot(path):
    path=Path(path)
    with np.load(path,allow_pickle=False) as data:
        required={'free','lower','resolution','map_version','source_stamp','envelope','configuration'}
        if not required.issubset(data.files):raise ValueError('map envelope/inflation evidence missing')
        configuration=json.loads(str(data['configuration']))
        expected=np.array(configuration['body_halfsize_m'])+configuration['clearance_m']
        envelope=np.array(data['envelope'])
        if envelope.shape!=(3,) or not np.isfinite(envelope).all() or np.any(envelope<expected-1e-8):
            raise ValueError('body inflation envelope mismatch')
        if data['free'].dtype!=np.dtype(bool) or data['free'].ndim!=3:
            raise ValueError('boolean three-dimensional observed free mask required')
        collision=CollisionMap(float(data['resolution']),data['lower'].copy(),data['free'].copy(),
                               int(data['map_version']),float(data['source_stamp']))
    return collision,{'source':'sensor_observed_inflated_snapshot','frame':'odom',
        'input_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'map_sha256':map_identity(collision),
        'envelope':envelope.tolist(),'configuration':configuration}


def comparison_summary(rows):
    methods={row['backend'] for row in rows}
    same_map=len({row['map_sha256'] for row in rows})==1
    complete=len(rows)==4 and methods==set(METHODS)
    checked=complete and all(row['cases'] and any(c['expected_success'] for c in row['cases'])
                             and all(c['passed'] for c in row['cases']) for row in rows)
    return {'all_methods_present':complete,'same_map':same_map,'passed':bool(checked and same_map)}


def metrics(curve,collision):
    times=np.linspace(0.,curve.duration,max(2,int(np.ceil(curve.duration*50))+1))
    points=np.array([curve.sample(t).position for t in times])
    edt=distance_transform_edt(np.pad(collision.free,1,constant_values=False))[1:-1,1:-1,1:-1]
    centers=np.array([collision.index(p) for p in points],dtype=int)
    margin=edt[tuple(centers.T)]*collision.resolution-np.sqrt(3)*collision.resolution/2
    return {'duration_s':curve.duration,'segments':len(curve.segments),
        'length_m':float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
        'derivative_maxima':curve.derivative_maxima(),'full_curve_observed_clear':curve.collision_free(collision),
        'observed_inflated_grid_margin_m':float(max(0.,margin.min())),
        'margin_basis':'distance to blocked/unknown voxel centers minus voxel circumradius; body already inflated',
        'midpoint_speed_mps':float(np.linalg.norm(curve.sample(curve.duration/2).velocity))}


def benchmark(root,snapshot,output,source_manifest,timeout=8.):
    root=Path(root).resolve();snapshot=Path(snapshot).resolve();source_manifest=Path(source_manifest).resolve()
    collision,source=load_snapshot(snapshot)
    manifest=json.loads(source_manifest.read_text())
    if not snapshot.is_relative_to(source_manifest.parent) or not manifest.get('run_id'):
        raise ValueError('snapshot must belong to its simulation run manifest')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(snapshot,output/'map.npz');shutil.copyfile(source_manifest,output/'simulation-manifest.json')
    source['run_id']=manifest['run_id'];source['manifest_sha256']=hashlib.sha256(source_manifest.read_bytes()).hexdigest()
    (output/'input.json').write_text(json.dumps(source,indent=2)+'\n')
    cases=[{'id':'normal','start':[0.,0.,2.],'goal':[2.5,3.6,2.],'expected_success':True},
           {'id':'nonzero_initial_pva','start':[0.,0.,2.],'velocity':[.1,0.,0.],'acceleration':[.02,0.,0.],
            'goal':[2.5,3.6,2.],'expected_success':True},
           {'id':'obstacle_rejection','start':[0.,0.,2.],'goal':[1.5,0.,2.],'expected_success':False},
           {'id':'outside_observed_map','start':[0.,0.,2.],'goal':[20.,20.,2.],'expected_success':False}]
    rows=[]
    for backend in METHODS:
        row={'backend':backend,'map_sha256':source['map_sha256'],'cases':[]}
        for case in cases:
            initial=State(np.array(case['start']),np.array(case.get('velocity',[0.,0.,0.])),
                          np.array(case.get('acceleration',[0.,0.,0.])))
            result=plan_curve(root,backend,collision,initial,case['goal'],output/backend/case['id'],timeout=timeout)
            outcome={**case,'success':result['success'],'reason':result['reason'],'runtime_s':result['runtime_s'],
                'fallback':result['fallback'],'implementation_sha256':result['implementation_sha256'],
                'source_ref':result['core_source_ref'],'core_peak_rss_bytes':result.get('core_peak_rss_bytes'),
                'core_cpu_s':result.get('core_cpu_s'),'evidence':str((output/backend/case['id']/'pipeline-report.json').relative_to(output))}
            if result['success']:outcome.update(metrics(result['curve'],collision))
            # A rejection must name its cause; a crashed/missing binary is not a
            # successful blocked-goal check. Normal cases require admitted curves.
            blocked_reason=('BLOCKED','UNOBSERVED','OUTSIDE_BOUNDS','blocked or unknown')
            outcome['passed']=(result['success'] if case['expected_success'] else
                not result['success'] and any(r in result['reason'] for r in blocked_reason))
            row['cases'].append(outcome)
        rows.append(row)
    report={'schema':1,'scope':'fixed observed map, offline planner comparison; not flight qualification',
            'input':source,'rows':rows,**comparison_summary(rows)}
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report
