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
    with (args.output/'geometry.log').open('w') as log:
        observer=subprocess.Popen(['/usr/bin/python3',str(root/'scripts/verify_navigation.py'),'--duration','350','--output',str(args.output/'geometry.json')],stdout=log,stderr=subprocess.STDOUT)
        try:
            demo=subprocess.run([str(root/'scripts/labctl'),'nav','demo','--runs',str(args.runs)],capture_output=True,text=True,timeout=320)
            (args.output/'demo.log').write_text(demo.stdout+demo.stderr)
        finally:
            observer.send_signal(signal.SIGTERM);observer.wait(timeout=10)
    report={'demo_returncode':demo.returncode,'geometry_returncode':observer.returncode,'passed':demo.returncode==0 and observer.returncode==0}
    (args.output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
