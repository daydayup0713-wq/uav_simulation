"""Recheck map, actual trajectories and native source identity before promotion."""
import json
from pathlib import Path
import numpy as np
from uav_lab_bridge.continuous_trajectory import State,Trajectory
from .planner_backends import validate_curve
from .planner_comparison import load_snapshot
from .native_provenance import fingerprint,verified_build


def replay_checks(root,backend,directory):
    root=Path(root);directory=Path(directory).resolve()
    report=json.loads((directory/'report.json').read_text())
    matches=[r for r in report.get('rows',[]) if r.get('backend')==backend]
    if len(matches)!=1:raise ValueError('planner replay evidence row missing')
    collision,source=load_snapshot(directory/'map.npz')
    if report['input']['input_sha256']!=source['input_sha256'] or report['input']['map_sha256']!=source['map_sha256']:
        raise ValueError('planner replay map identity changed')
    row=matches[0]
    cases=row['cases']
    if {c['id'] for c in cases}!={'normal','nonzero_initial_pva','obstacle_rejection','outside_observed_map'} or len(cases)!=4:
        raise ValueError('required planner replay evidence cases missing')
    build=verified_build(root,backend) if backend!='astar' else None
    for case in cases:
        path=(directory/case['evidence']).resolve()
        if not path.is_relative_to(directory):raise ValueError('planner replay artifact escaped directory')
        payload=json.loads(path.read_text())
        if payload['backend']!=backend or payload['map_sha256']!=source['map_sha256']:
            raise ValueError('planner replay backend/map identity mismatch')
        if payload['implementation_sha256']!=fingerprint(root,backend) or payload['fallback'] is not False:
            raise ValueError('planner replay implementation changed or fallback used')
        if build and payload['binary_sha256']!=build['binary_sha256']:raise ValueError('planner replay binary changed')
        if case['expected_success']:
            if not case['success'] or not payload['success']:raise ValueError('normal native planner evidence failed')
            initial=State(np.array(case['start']),np.array(case.get('velocity',[0.,0.,0.])),
                          np.array(case.get('acceleration',[0.,0.,0.])))
            validate_curve({'schema':1,'success':True,'map_sha256':payload['map_sha256'],
                            'trajectory':payload['trajectory']},collision,initial,case['goal'])
            if build:
                core_path=Path(payload['source_result']).resolve()
                if not core_path.is_relative_to(path.parent):raise ValueError('native core evidence escaped case')
                core=json.loads(core_path.read_text())
                if not core['success'] or core['backend']!=backend or core['map_sha256']!=source['map_sha256']:
                    raise ValueError('actual upstream core result missing')
                original=core['trajectory']['segments'];admitted=payload['trajectory']['segments']
                if len(original)!=len(admitted) or any(abs(a['duration']-b['duration'])>1e-9
                    or not np.allclose(a['coefficients'],b['coefficients'],rtol=0,atol=1e-9)
                    for a,b in zip(original,admitted)):
                    raise ValueError('admitted position curve differs from actual upstream solution')
        elif payload['success'] or not any(r in payload['reason'] for r in ('BLOCKED','UNOBSERVED','OUTSIDE_BOUNDS','blocked or unknown')):
            raise ValueError('blocked/unknown rejection evidence failed')
    return {'all_passed':True,'full_curve_rechecked':True,'exact_initial_pva':True,
            'same_observed_map':True,'native_source_identity':True,'flight_qualified':False}
