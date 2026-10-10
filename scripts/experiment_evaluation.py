"""Independent truth, geometry and semantic-route evaluation. Never an algorithm input."""
import numpy as np
from scipy.spatial.transform import Rotation
from verify_navigation import clearance


def evaluate(geometry,route,truth,references,events,expected_runs=1):
    airborne=[s for s in truth if s['position'][2]>1. and s['phase'] in ('MOVING','HOLDING')]
    moving=[s for s in truth if s['phase']=='MOVING']
    net=[]
    for sample in airborne:
        # Conservative world AABB enclosing the entire rotated body.
        half=np.abs(Rotation.from_quat(sample['quaternion']).as_matrix())@np.array([.4,.4,.3])
        net.extend(clearance(sample['position'],low,high,half) for low,high in geometry['boxes'])
    errors=[]
    for identifier in {r['id'] for r in references}:
        rows=[r for r in references if r['id']==identifier]
        times=np.array([r['stamp'] for r in rows]);positions=np.array([r['position'] for r in rows])
        if len(rows)<2 or np.any(np.diff(times)<0):continue
        for sample in moving:
            if times[0]<=sample['stamp']<=times[-1]:
                target=np.array([np.interp(sample['stamp'],times,positions[:,axis]) for axis in range(3)])
                errors.append(float(np.linalg.norm(target-sample['position'])))
    controls=route['controls']*expected_runs;cursor=0;distances=[]
    for target in controls:
        minimum=float('inf');reached=False
        while cursor<len(moving):
            error=float(np.linalg.norm(np.array(moving[cursor]['position'])-target));cursor+=1
            minimum=min(minimum,error)
            if error<=.3:reached=True;break
        if not reached:break
        distances.append(minimum)
    successes=[e for e in events if e['event']=='route_result' and e.get('success')]
    handoffs=[e for e in events if e['event']=='route_control_scheduled']
    expected_indices=list(range(len(route['controls'])-1))*expected_runs
    p95=float(np.percentile(errors,95)) if errors else None
    checks={'body_clearance':bool(net) and min(net)>.25,
        'truth_tracking':len(errors)>=20 and p95<=.3,
        'ordered_controls':len(distances)==len(controls),
        'completed_runs':len(successes)==expected_runs and all(e['controls']==len(route['controls']) for e in successes),
        'nonstop_handoffs':[e['index'] for e in handoffs]==expected_indices and all(
            np.linalg.norm(e['join_velocity'])>1e-6 and e['reference_error_m']<=.200001 for e in handoffs)}
    return {'passed':all(checks.values()),'failed_checks':[k for k,v in checks.items() if not v],
        'source':'Gazebo truth and physical scene geometry; evaluator only; no fitted control alignment',
        'body_halfsize_m':[.4,.4,.3],'body_scope':'attitude-rotated conservative full-body envelope',
        'min_body_clearance_m':min(net) if net else None,'airborne_samples':len(airborne),
        'truth_tracking_p95_m':p95,'truth_tracking_samples':len(errors),
        'covered_controls':len(distances),'expected_controls':len(controls),'ordered_control_errors_m':distances,
        'successful_runs':len(successes),'expected_runs':expected_runs,'handoffs':len(handoffs),
        'minimum_handoff_speed_mps':min((float(np.linalg.norm(e['join_velocity'])) for e in handoffs),default=None)}
