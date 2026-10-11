#!/usr/bin/python3
"""Own a deterministic sensor flight and archive source data for comparisons."""
import argparse,json,os,subprocess,time
from pathlib import Path
from supervise import owned_run_ready

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sensor-profile',choices=['livox','livox-rtk','mechanical'],default='livox')
    p.add_argument('--scene',choices=['circle-eight','helix','multi-room','corridor','dense','outdoor-rtk'],default='circle-eight')
    p.add_argument('--duration',type=float,default=200.);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists() or not 60<=args.duration<=1800:p.error('new output, duration 60..1800 required')
    args.output.mkdir(parents=True);supervisor=record=None;run=None;commands=[];success=False;reason=''
    def command(arguments,timeout=300):
        r=subprocess.run([str(ROOT/'scripts/labctl'),*arguments],cwd=ROOT,capture_output=True,text=True,timeout=timeout)
        commands.append({'arguments':arguments,'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
        if r.returncode:raise RuntimeError('command rejected: '+' '.join(arguments))
    with (args.output/'supervisor.log').open('w') as log,(args.output/'record.log').open('w') as recording:
        try:
            supervisor=subprocess.Popen([str(ROOT/'scripts/start_lab.sh'),'--profile','sensors','--sensor-profile',args.sensor_profile,'--scene',args.scene,'--headless'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+150
            while not owned_run_ready(ROOT/'.runtime',supervisor.pid):
                if supervisor.poll() is not None or time.monotonic()>deadline:raise RuntimeError('sensor startup failed')
                time.sleep(.25)
            run=Path((ROOT/'.runtime/current-run').read_text().strip())
            record=subprocess.Popen([str(ROOT/'scripts/labctl'),'record','--duration',str(args.duration)],cwd=ROOT,stdout=recording,stderr=subprocess.STDOUT)
            time.sleep(2.)
            for arguments in [['arm'],['takeoff','--height','2'],['experiments','route',str(run/'configuration/scene/route.json')],['land']]:command(arguments)
            record.wait(timeout=args.duration+60)
            if record.returncode:raise RuntimeError('sensor archive integrity gate failed')
            success=True
        except (RuntimeError,subprocess.TimeoutExpired,KeyboardInterrupt) as error:
            reason=str(error) or 'interrupted'
            if run and supervisor and supervisor.poll() is None:
                try:command(['land'],90)
                except (RuntimeError,subprocess.TimeoutExpired):pass
        finally:
            if record and record.poll() is None:record.terminate();record.wait(timeout=25)
            if supervisor and supervisor.poll() is None:
                supervisor.terminate()
                try:supervisor.wait(timeout=25)
                except subprocess.TimeoutExpired:reason+='; supervisor cleanup timed out';success=False
            (args.output/'acceptance.json').write_text(json.dumps({'passed':success,'reason':reason,'run_id':run.name if run else None,
                'sensor_profile':args.sensor_profile,'scene':args.scene,'requested_duration_s':args.duration,'commands':commands,
                'scope':'stock PX4 flight captures algorithm input; this does not qualify a new localization backend'},indent=2)+'\n')
    print(json.dumps({'passed':success,'reason':reason,'run_id':run.name if run else None}))
    return 0 if success else 1

if __name__=='__main__':raise SystemExit(main())
