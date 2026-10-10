"""Ground-selected private cores and explicit controlled-qualification policy."""
import json
from pathlib import Path
from uav_lab_experiments.registry import Registry,STAGES
from uav_lab_experiments.backend_contract import consumed_input_group


def validate_pair(registry,localization,planning,group,qualification=False):
    if group not in registry.groups:raise ValueError('unknown experiment input group')
    local=registry.describe(localization,group);planner=registry.describe(planning)
    if local['role']!='localization' or planner['role']!='planning':raise ValueError('experiment backend role mismatch')
    if group not in local['groups'] or group not in planner['groups']:raise ValueError('unsupported experiment input group')
    required='replay_passed' if qualification else 'closed_loop_qualified'
    if any(STAGES.index(item['stage'])<STAGES.index(required) for item in (local,planner)):
        raise ValueError('experiment requires '+required+' evidence; failed methods cannot control flight')
    planner_group=json.loads(Path(planner['evidence'][0]).read_text())['input_group']
    return {'localization':localization,'planning':planning,'localization_input_group':group,
            'localization_stage':local['stage'],'planning_stage':planner['stage'],
            'planning_input':'observed inflated occupancy grid','planning_evidence_group':planner_group,
            'qualification_run':bool(qualification),'flight_qualified':not qualification,
            'evidence':{'localization':local['evidence'],'planning':planner['evidence']}}


def binding(root,localization,planning,calibration,qualification=False):
    root=Path(root);registry=Registry(root/'configs/backends.json',root/'.runtime/backend-evidence')
    group=consumed_input_group(localization,calibration)
    selection=validate_pair(registry,localization,planning,group,qualification)
    if not qualification:
        from control_provenance import qualified_pair
        selection['pair_evidence']=qualified_pair(root,localization,planning,group)
        if not selection['pair_evidence']:raise ValueError('current control implementation and exact backend pair require native qualification')
    return selection


def prepare_backend(root,calibration,output,run_id,selection):
    from backend_provenance import verified_private_build,save_provenance
    from bootstrap_backends import verify_sources
    from backend_configs import write_config
    from backend_launch import core_commands
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    backend=selection['localization'];lock=json.loads((root/'dependencies/backends.lock.json').read_text())
    verify_sources(root,lock,lock['backends'][backend]['repositories'])
    prefix=root/'.deps/backends'/backend/'install';build=verified_private_build(prefix,backend)
    provenance=save_provenance(root,lock,backend,output,['supervised-closed-loop',run_id])
    spec=json.loads((root/'configs/backends.json').read_text())['backends']
    identity=next(item['implementation_sha256'] for item in spec if item['id']==backend)
    if provenance['implementation_sha256']!=identity:raise ValueError('selected localization source implementation changed')
    (output/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
    (output/'build-manifest.json').write_text(json.dumps(build,indent=2)+'\n')
    write_config(backend,calibration,output/'configuration',run_id)
    wrapper=[root/'scripts/backend_env.sh',backend];commands=[]
    if backend in ('fast_livo2','fast_livo2_rtk'):
        commands.append(('backend-camera',wrapper+['/opt/ros/humble/lib/demo_nodes_cpp/parameter_blackboard',
            '--ros-args','--params-file',output/'configuration/camera.yaml']))
    if backend in ('fast_lio2','fast_livo2','fast_livo2_rtk','lio_sam'):
        commands.append(('backend-input',wrapper+['/usr/bin/python3','-m','uav_lab_experiments.algorithm_sensor_node',
            '--ros-args','-p','backend:='+backend,'-p','calibration:='+str(output/'calibration.json'),
            '-p','trace_file:='+str(output/'inputs.jsonl')]))
    commands.append(('backend-normalizer',wrapper+['/usr/bin/python3','-m','uav_lab_experiments.backend_node',
        '--ros-args','-p','backend:='+backend,'-p','active:=true','-p','calibration:='+str(output/'calibration.json')]))
    for i,command in enumerate(core_commands(backend,Path(build['binary']),output/'configuration')):
        commands.append(('backend-core-'+str(i),wrapper+command))
    return commands
