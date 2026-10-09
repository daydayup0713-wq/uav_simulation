#!/usr/bin/python3
"""Publish artifact-backed stages only for verified private builds/native reports."""
import argparse,json
from pathlib import Path
from backend_provenance import ADAPTER_FILES,implementation_fingerprint,verify_runtime_artifacts
from bootstrap_backends import verify_sources
from uav_lab_experiments.registry import Registry,digest
from uav_lab_experiments.qualification import qualification_checks
from uav_lab_experiments.backend_contract import consumed_input_group

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['glim','fast_lio2','fast_livo2','fast_livo2_rtk','lio_sam','orb_slam3','vins_fusion'],required=True)
    parser.add_argument('--stage',choices=['installed','replay_passed','realtime_passed'],required=True)
    parser.add_argument('--group',required=True);parser.add_argument('--run-id',required=True)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    lock=json.loads((ROOT/'dependencies/backends.lock.json').read_text())
    names=lock['backends'][args.backend]['repositories'];verify_sources(ROOT,lock,names)
    registry=Registry(ROOT/'configs/backends.json',ROOT/'.runtime/backend-evidence')
    spec=registry.backends[args.backend]
    fingerprint=implementation_fingerprint(ROOT,lock,args.backend)
    if spec.get('implementation_sha256')!=fingerprint:raise ValueError('catalog implementation identity changed; update catalog before qualifying')
    prefix=ROOT/'.deps/backends'/args.backend/'install';manifest=prefix/(args.backend+'-build-manifest.json')
    build=json.loads(manifest.read_text());binary=Path(build['binary']).resolve()
    verify_runtime_artifacts(build)
    if not binary.is_relative_to(prefix.resolve()) or digest(binary)!=build['binary_sha256']:raise ValueError('private binary identity changed')
    for name in names:
        actual=build['repositories'][name];expected=lock['repositories'][name]
        if actual['ref']!=expected['ref'] or actual['tree_sha256']!=expected['tree_sha256']:raise ValueError('build source identity changed')
    artifacts=[manifest,binary,*[Path(name) for name in build.get('runtime_artifacts',{})],ROOT/'dependencies/backends.lock.json',*[ROOT/name for name in ADAPTER_FILES]]
    checks={'all_passed':True,'private_binary_hash':True,'locked_sources_and_patches':True,'flight_qualified':False}
    if args.stage!='installed':
        if not args.report:raise ValueError('native experiment report required')
        report=json.loads(args.report.read_text());checks=qualification_checks(args.stage,report)
        if report['backend']!=args.backend or report['implementation_sha256']!=fingerprint:raise ValueError('native experiment implementation identity changed')
        directory=args.report.resolve().parent
        calibration=json.loads((directory/'calibration.json').read_text())
        group=consumed_input_group(args.backend,calibration)
        if group!=args.group:raise ValueError('native experiment sensor group mismatch')
        artifacts.extend([args.report.resolve(),directory/'estimate.tum',directory/'truth.tum',directory/'provenance.json',
                          directory/'configuration/parameters.yaml',directory/'configuration/camera.yaml'])
        if args.stage=='realtime_passed':artifacts.extend([directory/'simulation-manifest.json',directory/'algorithm-subscriptions.json'])
        if args.backend=='fast_livo2_rtk':artifacts.append(directory/'configuration/rtk-output/TUM/opt_trajectory_after.txt')
    report={'schema':1,'backend':args.backend,'source_ref':spec['source_ref'],'implementation_sha256':fingerprint,
            'input_group':args.group,'stage':args.stage,'run_id':args.run_id,'success':True,'checks':checks,
            'artifacts':[{'path':str(p),'sha256':digest(p)} for p in artifacts]}
    print(registry.record(report))


if __name__=='__main__':
    try:main()
    except (KeyError,OSError,ValueError) as error:raise SystemExit(str(error))
