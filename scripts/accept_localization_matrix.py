#!/usr/bin/python3
"""Sequential native localization comparisons; failed algorithms remain reports."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import psutil
from export_backend_comparison import summarize

ROOT=Path(__file__).resolve().parents[1]
BACKENDS=('glim','fast_lio2','fast_livo2','fast_livo2_rtk','lio_sam','orb_slam3','vins_fusion')


def owned_replay(command,**kwargs):
    process=subprocess.Popen(command,start_new_session=True,**kwargs)
    try:
        return subprocess.CompletedProcess(command,process.wait())
    except BaseException:
        descendants=[]
        try:descendants=psutil.Process(process.pid).children(recursive=True)
        except psutil.NoSuchProcess:pass
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=20)
            except subprocess.TimeoutExpired:process.terminate()
        # The replay normally closes its own separate process groups. Only
        # descendants of this exact owned child are fallback cleanup targets.
        for child in reversed(descendants):
            try:child.terminate()
            except psutil.NoSuchProcess:pass
        _,alive=psutil.wait_procs(descendants,timeout=3)
        for child in alive:
            try:child.kill()
            except psutil.NoSuchProcess:pass
        if process.poll() is None:process.kill()
        process.wait()
        raise


def run_matrix(jobs,output,fingerprints,replay=owned_replay):
    output=Path(output).resolve()
    if output.exists():raise ValueError('matrix output already exists')
    names=[job['backend'] for job in jobs]
    if not names or len(set(names))!=len(names) or any(name not in BACKENDS for name in names):
        raise ValueError('distinct known localization backends required')
    output.mkdir(parents=True)
    (output/'jobs.json').write_text(json.dumps(jobs,indent=2)+'\n')
    reports=[];errors=[];exits={}
    try:
        for job in jobs:
            name=job['backend'];directory=output/name
            command=[str(ROOT/'scripts/backend_env.sh'),name,'env','ROS_DOMAIN_ID=77',
                     '/usr/bin/python3',str(ROOT/'scripts/replay_backend.py'),'--backend',name,
                     '--dataset',str(job['dataset']),'--output',str(directory)]
            if job.get('duration'):command+=['--duration',str(job['duration'])]
            if job.get('vins_map'):command+=['--vins-map',str(job['vins_map'])]
            print('START '+name,flush=True)
            with (output/(name+'-runner.log')).open('x') as stream:
                result=replay(command,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT)
            exits[name]=result.returncode
            path=directory/'report.json'
            if not path.is_file():errors.append({'backend':name,'reason':'native report missing','exit_code':result.returncode});continue
            data=json.loads(path.read_text())
            if data.get('backend')!=name:raise ValueError('report backend identity mismatch')
            reports.append(path)
            print('RESULT '+name+' '+('quality passed' if data.get('success') else 'quality failed; report retained'),flush=True)
    except KeyboardInterrupt:errors.append({'reason':'operator interrupted; owned replay stopped'})
    comparison=summarize(reports,fingerprints)
    runs=[r for group in comparison['groups'] for case in group['cases'] for r in case['runs']]
    passed=sum(r.get('success') is True and (r.get('quality') or {}).get('passed') is True for r in runs)
    result={'schema':1,'complete':len(reports)==len(jobs) and not errors,'qualification':'none',
            'quality_passed':passed,'quality_failed':len(reports)-passed,'errors':errors,
            'native_exit_codes':exits,'comparison':comparison}
    (output/'matrix.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def stop_request(*_):
    raise KeyboardInterrupt()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--backends',default=','.join(BACKENDS))
    args=parser.parse_args();jobs=json.loads(args.config.read_text())['jobs']
    selected=args.backends.split(',')
    if any(name not in BACKENDS for name in selected):parser.error('unknown localization backend')
    jobs=[job for job in jobs if job['backend'] in selected]
    if {job['backend'] for job in jobs}!=set(selected):parser.error('selected dataset job missing')
    for job in jobs:
        if not (ROOT/job['dataset']/'dataset.json').is_file():parser.error('local dataset missing: '+job['dataset'])
    catalog=json.loads((ROOT/'configs/backends.json').read_text())
    fingerprints={row['id']:row.get('implementation_sha256') for row in catalog['backends']}
    signal.signal(signal.SIGTERM,stop_request)
    result=run_matrix(jobs,args.output,fingerprints)
    print(json.dumps({k:v for k,v in result.items() if k!='comparison'},indent=2))
    return 0 if result['complete'] else 1


if __name__=='__main__':raise SystemExit(main())
