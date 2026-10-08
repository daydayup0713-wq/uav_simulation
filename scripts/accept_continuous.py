#!/usr/bin/python3
"""Own a short continuous-flight acceptance run; execute immediately after READY."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from supervise import owned_run_ready


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['flight', 'navigation'], default='flight')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.runs <= 3:
        parser.error('one to three sequential runs required')
    root = Path(os.environ['LAB_ROOT'])
    args.output.mkdir(parents=True, exist_ok=False)
    commands, reason, supervisor, observer, geometry = [], '', None, None, None
    passed, run = False, None
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
            supervisor = subprocess.Popen([str(root / 'scripts/start_lab.sh'), '--profile', args.profile,
                                            '--continuous', '--headless'], cwd=root, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 150
            while time.monotonic() < deadline:
                if supervisor.poll() is not None:
                    raise RuntimeError('supervisor exited during startup')
                if owned_run_ready(root / '.runtime', supervisor.pid):
                    break
                time.sleep(.25)
            else:
                raise RuntimeError('supervisor readiness deadline expired')
            run = Path((root / '.runtime/current-run').read_text().strip())
            observer = subprocess.Popen(['/usr/bin/python3', str(root / 'scripts/verify_continuous.py'),
                                         '--output', str(args.output / 'tracking.json')], stdout=observation,
                                        stderr=subprocess.STDOUT)
            if args.profile == 'navigation':
                geometry = subprocess.Popen(['/usr/bin/python3', str(root / 'scripts/verify_navigation.py'),
                                              '--duration', '600', '--output', str(args.output / 'geometry.json')],
                                             stdout=observation, stderr=subprocess.STDOUT)
                command(['nav', 'demo', '--runs', str(args.runs)], timeout=550)
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
              'profile': args.profile, 'runs': args.runs, 'commands': commands}
    (args.output / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'passed': passed, 'reason': reason, 'run_id': report['run_id']}))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
