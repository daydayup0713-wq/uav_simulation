#!/usr/bin/python3
"""Sequential owned PX4 fault cases; every result and startup failure is retained."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
from accept_continuous import supervisor_arguments
from supervise import owned_run_ready
from fault_scenario import CASES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--localization-backend',default='glim');parser.add_argument('--planner-backend',default='ego')
    parser.add_argument('--rendering',choices=('auto','mesa-display'),default='auto')
    parser.add_argument('--cases',nargs='+',choices=CASES,default=list(CASES));parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=Path(os.environ['LAB_ROOT']);args.output.mkdir(parents=True,exist_ok=False);results=[]
    for case in args.cases:
        directory=args.output/case;directory.mkdir();supervisor=None;run=None;passed=False;reason=''
        with (directory/'supervisor.log').open('w') as log:
            try:
                supervisor=subprocess.Popen([str(root/'scripts/start_lab.sh'),
                    *supervisor_arguments('navigation',args.localization_backend,args.planner_backend,qualification=True,
                                          web=case=='web_disconnect',rendering=args.rendering)],stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+150
                while time.monotonic()<deadline:
                    if supervisor.poll() is not None:raise RuntimeError('supervisor startup failed')
                    if owned_run_ready(root/'.runtime',supervisor.pid):break
                    time.sleep(.25)
                else:raise RuntimeError('owned run readiness expired')
                run=Path((root/'.runtime/current-run').read_text().strip())
                process=subprocess.run([str(root/'scripts/env.sh'),'/usr/bin/python3',str(root/'scripts/fault_scenario.py'),
                    '--case',case,'--run',str(run),'--output',str(directory/'fault.json')],capture_output=True,text=True,timeout=220)
                (directory/'observer.log').write_text(process.stdout+process.stderr)
                passed=process.returncode==0
                if not passed:reason='fault observation failed; see observer.log/fault.json'
            except (RuntimeError,OSError,subprocess.TimeoutExpired,KeyboardInterrupt) as error:reason=str(error)
            finally:
                if supervisor:
                    supervisor.send_signal(signal.SIGTERM)
                    try:supervisor.wait(timeout=20)
                    except subprocess.TimeoutExpired:passed=False;reason+='; supervisor teardown timeout'
        report={'case':case,'passed':passed,'reason':reason,'run_id':run.name if run else None}
        (directory/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n');results.append(report)
        print(json.dumps(report),flush=True)
    report={'passed':all(r['passed'] for r in results),'cases':results,
        'backend_pair':{'localization':args.localization_backend,'planning':args.planner_backend}}
    (args.output/'matrix.json').write_text(json.dumps(report,indent=2)+'\n');return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
