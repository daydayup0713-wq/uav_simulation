#!/usr/bin/python3
"""Collect native navigation demo and simultaneous independent evaluation."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--runs',type=int,default=3)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=Path(os.environ['LAB_ROOT']);args.output.mkdir(parents=True,exist_ok=False)
    demo_returncode=1;reason='';landing_cleanup=None
    with (args.output/'geometry.log').open('w') as log:
        observer=subprocess.Popen(['/usr/bin/python3',str(root/'scripts/verify_navigation.py'),'--duration','350','--output',str(args.output/'geometry.json')],stdout=log,stderr=subprocess.STDOUT)
        try:
            demo=subprocess.run([str(root/'scripts/labctl'),'nav','demo','--runs',str(args.runs)],capture_output=True,text=True,timeout=320)
            demo_returncode=demo.returncode
            (args.output/'demo.log').write_text(demo.stdout+demo.stderr)
        except (subprocess.TimeoutExpired,KeyboardInterrupt) as error:
            reason='acceptance watchdog interrupted demo: '+str(error)
            try:
                landing=subprocess.run([str(root/'scripts/labctl'),'land'],capture_output=True,text=True,timeout=75)
                landing_cleanup={'returncode':landing.returncode,'stdout':landing.stdout,'stderr':landing.stderr}
            except subprocess.TimeoutExpired:landing_cleanup={'returncode':1,'reason':'bounded landing request expired'}
            (args.output/'demo.log').write_text(reason+'\n')
        finally:
            observer.send_signal(signal.SIGTERM);observer.wait(timeout=10)
    report={'demo_returncode':demo_returncode,'geometry_returncode':observer.returncode,'reason':reason,'landing_cleanup':landing_cleanup,
        'passed':demo_returncode==0 and observer.returncode==0}
    (args.output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
