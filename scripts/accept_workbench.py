#!/usr/bin/python3
"""Sequential native candidates/scenes/faults; never overlaps lab instances."""
import argparse,json,os,subprocess,time
from pathlib import Path


def matrix_cases(rendering):
    cases=[('ego',['scripts/accept_continuous.py','--profile','navigation','--localization-backend','glim',
            '--planner-backend','ego','--scene','circle-eight','--qualification','--runs','3']),
        ('faults',['scripts/accept_faults.py']),
        *[(planner,['scripts/accept_continuous.py','--profile','navigation','--localization-backend','glim',
                    '--planner-backend',planner,'--scene','circle-eight','--qualification','--runs','3'])
          for planner in ('astar','fast_planner','gcopter')],
        *[(scene,['scripts/accept_continuous.py','--profile','navigation','--localization-backend','glim',
                  '--planner-backend','ego','--scene',scene,'--qualification','--runs','1'])
          for scene in ('helix','multi-room','corridor','dense','outdoor-rtk')],
        ('fast-livo-ego',['scripts/accept_continuous.py','--profile','navigation','--localization-backend','fast_livo2',
             '--planner-backend','ego','--sensor-profile','livox','--scene','circle-eight','--qualification','--runs','3']),
        ('rtk-inputs',['scripts/accept_rtk_faults.py'])]
    return [(name,[*arguments,'--rendering',rendering]) for name,arguments in cases]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rendering',choices=('auto','mesa-display'),default='auto')
    parser.add_argument('--after',type=Path);args=parser.parse_args();root=Path(os.environ['LAB_ROOT'])
    args.output.mkdir(parents=True,exist_ok=False);results=[]
    if args.after:
        deadline=time.monotonic()+3600
        while not args.after.is_file() and time.monotonic()<deadline:time.sleep(1)
        if not args.after.is_file():raise RuntimeError('predecessor acceptance missing')
        if not json.loads(args.after.read_text())['passed']:raise RuntimeError('predecessor acceptance failed; inspect before continuing')
    cases=matrix_cases(args.rendering)
    for name,arguments in cases:
        output=args.output/name
        with (args.output/(name+'.log')).open('w') as log:
            process=subprocess.run(['/usr/bin/python3',*arguments,'--output',str(output)],cwd=root,
                                   stdout=log,stderr=subprocess.STDOUT,timeout=4200)
        report={'case':name,'exit_code':process.returncode,'output':str(output.resolve()),'passed':process.returncode==0}
        results.append(report);(args.output/'progress.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(report),flush=True)
    (args.output/'matrix.json').write_text(json.dumps({'cases':results,'passed':all(r['passed'] for r in results),
        'scope':'negative quality/scene results remain explicit; per-case success does not promote any backend'},indent=2)+'\n')
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
