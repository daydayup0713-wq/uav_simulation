#!/usr/bin/python3
"""Register installed/offline planner stages from locked real cores and map reports."""
import argparse,json
from pathlib import Path
from uav_lab_navigation.native_provenance import verified_build,implementation_files,fingerprint
from uav_lab_navigation.planner_qualification import replay_checks
from uav_lab_experiments.registry import Registry,digest
from bootstrap_backends import verify_sources

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['astar','ego','fast_planner','gcopter'],required=True)
    parser.add_argument('--stage',choices=['installed','replay_passed'],required=True)
    parser.add_argument('--group',required=True);parser.add_argument('--run-id',required=True)
    parser.add_argument('--directory',type=Path)
    args=parser.parse_args();registry=Registry(ROOT/'configs/backends.json',ROOT/'.runtime/backend-evidence')
    spec=registry.backends[args.backend];identity=fingerprint(ROOT,args.backend)
    if spec.get('implementation_sha256')!=identity:raise ValueError('planner catalog implementation changed')
    artifacts=implementation_files(ROOT)
    if args.backend!='astar':
        lock=json.loads((ROOT/'dependencies/planners.lock.json').read_text())
        verify_sources(ROOT,lock,lock['backends'][args.backend]['repositories'])
        build=verified_build(ROOT,args.backend)
        artifacts.extend([ROOT/'.deps/planners'/args.backend/'install/build-manifest.json',
                          *[Path(p) for p in build['runtime_artifacts']]])
    checks={'all_passed':True,'installed_private_core':True,'flight_qualified':False}
    if args.stage=='replay_passed':
        if not args.directory:raise ValueError('native comparison directory required')
        checks=replay_checks(ROOT,args.backend,args.directory)
        manifest=json.loads((args.directory/'simulation-manifest.json').read_text())
        expected='synchronous_lidar_imu' if manifest['sensor_profile'] is None else {
            'livox':'livox_visual_inertial','livox-rtk':'livox_visual_inertial_rtk','mechanical':'mechanical_inertial'}[manifest['sensor_profile']]
        if args.group!=expected:raise ValueError('planner observed map input group mismatch')
        artifacts.extend(p for p in args.directory.rglob('*') if p.is_file())
    report={'schema':1,'backend':args.backend,'source_ref':spec['source_ref'],'implementation_sha256':identity,
        'input_group':args.group,'stage':args.stage,'run_id':args.run_id,'success':True,'checks':checks,
        'artifacts':[{'path':str(p.resolve()),'sha256':digest(p)} for p in artifacts]}
    print(registry.record(report))


if __name__=='__main__':
    try:main()
    except (ValueError,OSError,KeyError) as error:raise SystemExit(str(error))
