#!/usr/bin/python3
"""Promote an exact pair only after independent native repeated-flight/fault rechecks."""
import argparse,json,re
from pathlib import Path
import numpy as np
from control_provenance import matches
from handoff_evidence import recheck
from experiment_evaluation import evaluate
from uav_lab_experiments.registry import Registry,digest
from uav_lab_experiments.backend_contract import consumed_input_group
from uav_lab_experiments.evaluation import evaluate_continuous

ROOT=Path(__file__).resolve().parents[1]


def rows(path):return [json.loads(line) for line in Path(path).read_text().splitlines()]


def identity_checks(root,run,selection):
    root,run=Path(root),Path(run)
    catalog=json.loads((root/'configs/backends.json').read_text())
    names=(selection['localization'],selection['planning'])
    expected={item['id']:item['implementation_sha256'] for item in catalog['backends'] if item['id'] in names}
    if set(expected)!=set(names):raise ValueError('backend implementation identity missing')
    directory=run/'configuration/selected-localization'
    recorded=json.loads((directory/'provenance.json').read_text())
    if recorded['implementation_sha256']!=expected[names[0]]:
        raise ValueError('native localization implementation changed')
    from backend_provenance import verify_runtime_artifacts
    verify_runtime_artifacts(json.loads((directory/'build-manifest.json').read_text()))
    if names[1]!='astar':
        from uav_lab_navigation.native_provenance import verified_build
        build=verified_build(root,names[1])
        events=[row for row in rows(run/'navigation.jsonl') if row['event']=='native_plan' and row.get('success')]
        if not events or any(row.get('backend')!=names[1] or row.get('implementation_sha256')!=expected[names[1]]
            or row.get('binary_sha256')!=build['binary_sha256'] for row in events):
            raise ValueError('native planner implementation/binary changed')
    return expected


def normal_checks(root,directory):
    root,directory=Path(root),Path(directory)
    try:
        acceptance=json.loads((directory/'acceptance.json').read_text())
        run=root/'.runtime'/acceptance['run_id'];manifest=json.loads((run/'manifest.json').read_text())
        if (not acceptance['passed'] or acceptance['profile']!='navigation' or acceptance['runs']!=3
                or not acceptance['backend_pair']['scene'] or not matches(root,manifest['control_implementation'])):
            raise ValueError('native repeated/current source evidence required')
        implementations=identity_checks(root,run,manifest['backend_selection'])
        scene=run/'configuration/scene';route=json.loads((scene/'route.json').read_text())
        if not 20<=len(route['controls'])<=50:raise ValueError('complex semantic control evidence required')
        tracking=json.loads((directory/'tracking.samples.json').read_text())
        continuous=evaluate_continuous(tracking['references'],tracking['actual'],tracking['heartbeats_wall'],tracking['setpoints_wall'])
        samples=json.loads((directory/'geometry.samples.json').read_text())
        geometry=evaluate(json.loads((scene/'sensor-geometry.json').read_text()),route,samples['truth'],samples['references'],rows(run/'navigation.jsonl'),3)
        handoffs=recheck(rows(run/'events.jsonl'))
        if not all(report['passed'] for report in (continuous,geometry,handoffs)):
            raise ValueError('independent repeated-flight evidence failed')
        if handoffs['joins']<3*(len(route['controls'])-1):raise ValueError('missing accepted handoff evidence')
        if not list((run/'px4').rglob('*.ulg')):raise ValueError('PX4 runtime log evidence missing')
        params=manifest['parameters']
        if any(params[key]!=0 for key in ('EKF2_GPS_CTRL','SIM_GPS_USED','SENS_EN_GPSSIM')):
            raise ValueError('no-GNSS closed loop evidence required')
        fusion=[event for event in rows(run/'events.jsonl') if event['event']=='fusion']
        if not any(all(event[k] for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw')) for event in fusion):
            raise ValueError('actual PX4 external fusion evidence missing')
        return {'run':run,'manifest':manifest,'acceptance':acceptance,'continuous':continuous,'geometry':geometry,'handoffs':handoffs,
            'backend_implementations':implementations}
    except (OSError,KeyError,TypeError) as error:raise ValueError('incomplete native pair evidence: '+str(error)) from error


def fault_checks(root,directories,pair):
    from fault_scenario import CASES
    from px4_msgs.msg import VehicleStatus
    found={}
    for directory in directories:
        for case in CASES:
            path=Path(directory)/case/'fault.json'
            if path.exists():found[case]=path
    if set(found)!=set(CASES):raise ValueError('complete native fault evidence required')
    for case,path in found.items():
        report=json.loads(path.read_text());run=Path(root)/'.runtime'/report['run_id']
        manifest=json.loads((run/'manifest.json').read_text());selection=manifest['backend_selection']
        identity_checks(root,run,selection)
        if (not report['passed'] or not matches(root,manifest['control_implementation'])
                or selection['localization']!=pair['localization'] or selection['planning']!=pair['planning']
                or report['final']['armed'] is not False or report['final']['landed'] is not True):
            raise ValueError('fault pair/current implementation/final state evidence mismatch')
        if case in ('localization_exit','source_stale'):
            if not any(e['armed'] and e['nav_state']==VehicleStatus.NAVIGATION_STATE_AUTO_LAND for e in report['events']):
                raise ValueError('independent airborne failsafe transition missing')
            arm=[c for c in report['commands'] if c['arguments']==['arm']][-1]
            if arm['returncode']==0:raise ValueError('source recovery control latch missing')
        if case=='source_stale' and not report['source_process_resumed']:raise ValueError('source recovery untested')
        if case=='hold':
            if not any(np.linalg.norm(r['velocity'])<1e-6 for r in report['references']):raise ValueError('constrained stop evidence missing')
            stop_checks(run)
        if case=='web_disconnect':
            if report['motion_returncode']!=0 or not report['browser']['client_closed'] or not (run/'web-failure.json').is_file():
                raise ValueError('actual Web isolation evidence missing')
        elif report['motion_returncode']==0:raise ValueError('interrupted motion incorrectly succeeded')
    return found



def pair_identifier(value):
    if not isinstance(value,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,60}',value):
        raise ValueError('invalid bounded pair run id')
    return value


def stop_checks(run):
    from uav_lab_bridge.stop_admission import StopAdmission
    from uav_lab_bridge.continuous_trajectory import Trajectory
    run=Path(run).resolve();config=json.loads((run/'configuration/navigation.json').read_text())
    events=[event for event in rows(run/'events.jsonl') if event['event']=='stop_admission']
    if not events:raise ValueError('observed stop-map admission evidence missing')
    paths=[]
    for event in events:
        path=(run/event['map_artifact']).resolve()
        if not path.is_relative_to(run) or digest(path)!=event['map_sha256']:
            raise ValueError('original stop map artifact changed')
        with np.load(path,allow_pickle=False) as data:
            free=data['free']
            if free.ndim!=3 or free.size>2000000:raise ValueError('unbounded stop map')
            gate=StopAdmission(np.asarray(config['body_halfsize_m'])+config['clearance_m'])
            source=float(data['source_stamp']);version=int(data['map_version'])
            if source!=event['source_stamp'] or version!=event['map_version']:
                raise ValueError('stop map source identity changed')
            gate.observe({'lower':data['lower'],'resolution':float(data['resolution']),'shape':list(free.shape),
                'free':free.astype(np.uint8).tobytes(),'envelope':data['envelope'],'frame':'odom',
                'source_stamp':source,'version':version},0.)
            gate.admit(Trajectory.from_dict(event['trajectory']),0.,event['sim_ns']/1e9)
        paths.append(path)
    return paths


def publish_pair(registry,destination,run_id,report,source_artifacts):
    pair_identifier(run_id);pair=report['pair'];prepared=[]
    from uav_lab_experiments.registry import STAGES
    destination=Path(destination);pair_path=destination/(run_id+'.json')
    if pair_path.exists():raise ValueError('pair evidence already exists')
    for backend in (pair['localization'],pair['planning']):
        spec=registry.backends[backend]
        if STAGES.index(registry.describe(backend,pair['input_group'])['stage'])<STAGES.index('replay_passed'):
            raise ValueError('previous qualification stage missing')
        for stage in ('realtime_passed','closed_loop_qualified'):
            row={'schema':1,'backend':backend,'source_ref':spec['source_ref'],
                'implementation_sha256':spec.get('implementation_sha256'),'input_group':pair['input_group'],
                'stage':stage,'run_id':run_id+'-'+backend+'-'+stage,'success':True,
                'checks':{'all_passed':True,'exact_pair':pair,'native_independent_rechecks':True},
                'artifacts':report['artifacts']+source_artifacts}
            if not registry._valid(row,backend,pair['input_group']):raise ValueError('invalid pair artifact evidence')
            if (registry.evidence_dir/backend/(row['run_id']+'.json')).exists():raise ValueError('stage evidence already exists')
            prepared.append(row)
    written=[]
    try:
        for row in prepared:written.append(registry.record(row))
        destination.mkdir(parents=True,exist_ok=True)
        # Publish the enabling pair witness only after both stage chains exist.
        with pair_path.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    except Exception:
        # Only files created by this transaction; never older evidence.
        for path in written:path.unlink()
        raise
    return pair_path

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--normal',type=Path,required=True)
    parser.add_argument('--fault-directory',type=Path,action='append',required=True);parser.add_argument('--run-id',required=True)
    args=parser.parse_args();pair_identifier(args.run_id);normal=normal_checks(ROOT,args.normal);manifest=normal['manifest'];selection=manifest['backend_selection']
    pair={'localization':selection['localization'],'planning':selection['planning'],
        'input_group':consumed_input_group(selection['localization'],manifest['calibration'])}
    faults=fault_checks(ROOT,args.fault_directory,pair)
    paths=[p for p in args.normal.rglob('*') if p.is_file()]
    run=normal['run'];paths.extend(p for p in (run/'configuration').rglob('*') if p.is_file())
    paths.extend([run/'manifest.json',run/'events.jsonl',run/'navigation.jsonl',*list((run/'px4').rglob('*.ulg'))])
    # Every accepted curve retains its original core and observed-map artifacts.
    for event in rows(run/'navigation.jsonl'):
        if event['event']=='continuous_plan':
            path=run/event['map_artifact']
            if digest(path)!=event['map_sha256']:raise ValueError('admitted map artifact changed')
            from uav_lab_bridge.continuous_trajectory import Trajectory
            from uav_lab_navigation.occupancy import CollisionMap
            data=np.load(path);collision=CollisionMap(float(data['resolution']),data['lower'],data['free'],int(data['map_version']))
            if not Trajectory.from_dict(event['trajectory']).collision_free(collision):raise ValueError('full accepted curve not admitted by observed map')
            paths.append(path)
        if event['event']=='native_plan':paths.extend(p for p in Path(event['evidence']).rglob('*') if p.is_file())
    for path in faults.values():
        report=json.loads(path.read_text());fault_run=ROOT/'.runtime'/report['run_id']
        paths.extend([path,fault_run/'manifest.json',fault_run/'events.jsonl'])
        if report['case']=='hold':paths.extend(stop_checks(fault_run))
    registry=Registry(ROOT/'configs/backends.json',ROOT/'.runtime/backend-evidence')
    artifacts=[{'path':str(p.resolve()),'sha256':digest(p)} for p in sorted(set(paths))]
    report={'schema':1,'success':True,'pair':pair,'control_implementation':manifest['control_implementation'],
        'backend_implementations':normal['backend_implementations'],
        'normal_run_id':run.name,'normal':{k:normal[k] for k in ('continuous','geometry','handoffs')},
        'faults':{k:str(v.resolve()) for k,v in faults.items()},'artifacts':artifacts}
    source_paths=[ROOT/name for name in manifest['control_implementation']['files']]
    source_artifacts=[{'path':str(p),'sha256':digest(p)} for p in source_paths]
    path=publish_pair(registry,ROOT/'.runtime/pair-evidence',args.run_id,report,source_artifacts)
    print(json.dumps({'qualified':pair,'report':str(path)}))



if __name__=='__main__':
    try:main()
    except (ValueError,OSError,KeyError) as error:raise SystemExit(str(error))
