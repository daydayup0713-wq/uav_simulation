#!/usr/bin/python3
"""Own a short continuous-flight acceptance run; execute immediately after READY."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import threading

from supervise import owned_run_ready


def supervisor_arguments(profile,localization=None,planning='astar',sensors=None,scene=None,qualification=False,web=False,rendering='auto'):
    args=['--profile',profile,'--continuous','--headless','--rendering',rendering]
    for key,value in [('--localization-backend',localization),('--sensor-profile',sensors),('--scene',scene)]:
        if value is not None:args += [key,value]
    if localization:args += ['--planner-backend',planning]
    if qualification:args.append('--qualification')
    if web:args.append('--web')
    return args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['flight', 'navigation'], default='flight')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--web',action='store_true')
    parser.add_argument('--rendering',choices=('auto','mesa-display'),default='auto')
    parser.add_argument('--localization-backend')
    parser.add_argument('--planner-backend',default='astar',choices=['astar','ego','fast_planner','gcopter'])
    parser.add_argument('--sensor-profile',choices=['livox','livox-rtk','mechanical'])
    parser.add_argument('--scene',choices=['circle-eight','helix','multi-room','corridor','dense','outdoor-rtk'])
    parser.add_argument('--qualification',action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.runs <= 3:
        parser.error('one to three sequential runs required')
    root = Path(os.environ['LAB_ROOT'])
    args.output.mkdir(parents=True, exist_ok=False)
    commands, reason, supervisor, observer, geometry = [], '', None, None, None
    passed, run = False, None
    resource_rows=[];resource_stop=threading.Event();resource_thread=None
    def sample_resources():
        import psutil
        begun=time.monotonic()
        while not resource_stop.is_set():
            processes={}
            try:
                identifiers=json.loads((run/'processes.json').read_text())
                for name,pid in identifiers.items():
                    if name not in ('backend-normalizer','backend-input','backend-camera','navigation') and not name.startswith('backend-core-'):continue
                    try:
                        parent=psutil.Process(pid)
                        for item in [parent,*parent.children(recursive=True)]:
                            cpu=item.cpu_times();processes[item.pid]={'pid':item.pid,'rss':item.memory_info().rss,'cpu_s':cpu.user+cpu.system}
                    except (psutil.NoSuchProcess,psutil.AccessDenied):continue
                resource_rows.append({'wall_s':time.monotonic()-begun,'phase':'closed_loop','processes':list(processes.values())})
            except (OSError,ValueError):pass
            resource_stop.wait(.5)
    def command(arguments, timeout=90):
        process = subprocess.run([str(root / 'scripts/labctl'), *arguments], cwd=root,
                                 capture_output=True, text=True, timeout=timeout)
        commands.append({'arguments': arguments, 'returncode': process.returncode,
                         'stdout': process.stdout, 'stderr': process.stderr})
        if process.returncode:
            raise RuntimeError('command failed: ' + ' '.join(arguments) + ': ' + process.stdout + process.stderr)
        return process
    with (args.output / 'supervisor.log').open('w') as log, (args.output / 'observer.log').open('w') as observation:
        try:
            supervisor = subprocess.Popen([str(root / 'scripts/start_lab.sh'),
                *supervisor_arguments(args.profile,args.localization_backend,args.planner_backend,args.sensor_profile,
                                      args.scene,args.qualification,args.web,args.rendering)], cwd=root, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 150
            while time.monotonic() < deadline:
                try:
                    candidate=Path((root/'.runtime/current-run').read_text().strip())
                    if candidate.parent==root/'.runtime' and json.loads((candidate/'manifest.json').read_text())['supervisor_pid']==supervisor.pid:
                        run=candidate
                except (OSError,KeyError,ValueError):pass
                if supervisor.poll() is not None:
                    raise RuntimeError('supervisor exited during startup')
                if owned_run_ready(root / '.runtime', supervisor.pid):
                    break
                time.sleep(.25)
            else:
                raise RuntimeError('supervisor readiness deadline expired')
            run = Path((root / '.runtime/current-run').read_text().strip())
            resource_thread=threading.Thread(target=sample_resources,daemon=True);resource_thread.start()
            observer = subprocess.Popen(['/usr/bin/python3', str(root / 'scripts/verify_continuous.py'),
                                         '--duration','3600','--output', str(args.output / 'tracking.json')], stdout=observation,
                                        stderr=subprocess.STDOUT)
            if args.profile == 'navigation':
                evaluator=(['/usr/bin/python3',str(root/'scripts/verify_experiment.py'),'--run',str(run),'--runs',str(args.runs)]
                    if args.scene else ['/usr/bin/python3',str(root/'scripts/verify_navigation.py'),'--duration','600'])
                geometry = subprocess.Popen([*evaluator,'--output',str(args.output/'geometry.json')],
                                             stdout=observation,stderr=subprocess.STDOUT)
                navigation=(['nav','--timeout','3300','route-demo',str(run/'configuration/scene/route.json')]
                    if args.scene else ['nav','demo'])
                command([*navigation,'--runs',str(args.runs)],timeout=3400 if args.scene else 550)
            else:
                for _ in range(args.runs):
                    command(['arm'])
                    command(['takeoff', '--height', '2'])
                    command(['experiments', 'route', str(root / 'configs/routes/continuous-turn.json')])
                    command(['land'])
            passed = True
        except (RuntimeError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            reason = str(error)
            if run is not None:
                try:
                    command(['land'], timeout=75)
                except (RuntimeError, subprocess.TimeoutExpired):
                    pass
        finally:
            resource_stop.set()
            if resource_thread:resource_thread.join(timeout=2.)
            for process in (observer, geometry):
                if process is not None:
                    process.send_signal(signal.SIGTERM)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                    passed = passed and process.returncode == 0
                    if process.returncode:
                        reason += '; evaluator failed; see tracking/geometry report and observer.log'
            if supervisor is not None:
                supervisor.send_signal(signal.SIGTERM)
                try:
                    supervisor.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    # Only this run's owned supervisor is targeted. Its own finally
                    # performs process-group cleanup; retain a teardown failure.
                    reason += '; supervisor teardown deadline expired'
                    passed = False
    report = {'passed': passed, 'reason': reason, 'run_id': run.name if run else None,
              'profile': args.profile, 'runs': args.runs, 'web':args.web, 'commands': commands}
    report['backend_pair']={'localization':args.localization_backend,'planning':args.planner_backend,
                            'qualification_run':args.qualification,'sensor_profile':args.sensor_profile,'scene':args.scene}
    from uav_lab_experiments.benchmark_report import resource_summary
    report['resources']=resource_summary(resource_rows)
    for phase in report['resources'].values():
        phase['scope']='sampled owned navigation, localization, camera parameter and native planner processes; excludes flight bridge, PX4, Gazebo, rendering, recorder and browser; 0.5s sampling may miss short-lived children'
    (args.output/'resources.samples.json').write_text(json.dumps(resource_rows)+'\n')
    (args.output / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'passed': passed, 'reason': reason, 'run_id': report['run_id']}))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
