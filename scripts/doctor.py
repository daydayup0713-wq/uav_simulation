#!/usr/bin/python3
"""Report host and private runtime requirements, without changing the host."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
from check_messages import check_messages

ROOT = Path(__file__).resolve().parents[1]

def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=8)
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, '', str(exc)

def checks(runtime=False, sensors=False):
    report = []
    def add(name, ok, detail, required=True):
        report.append({'name': name, 'ok': bool(ok), 'required': required, 'detail': str(detail)})
    os_release = Path('/etc/os-release').read_text()
    add('Ubuntu 22.04', 'VERSION_ID="22.04"' in os_release, 'requires Ubuntu 22.04')
    add('Python 3.10', sys.version_info[:2] == (3,10), sys.version.split()[0])
    add('ROS Humble', Path('/opt/ros/humble/setup.bash').exists(), '/opt/ros/humble')
    for name in ('git','cmake','ninja','colcon','gz'):
        add(name, shutil.which(name), shutil.which(name) or 'missing')
    code, out, err = command(['gz','sim','--versions'])
    add('Gazebo Harmonic', code==0 and any(v.strip().startswith('8.') for v in out.splitlines()), out or err)
    code, out, err = command(['dpkg-query','-W','-f=${db:Status-Status}', 'ros-humble-ros-gzharmonic-bridge'])
    add('Harmonic ROS bridge', code==0 and out=='installed', out or err)
    if runtime:
        paths = ['.deps/px4/build/px4_sitl_default/bin/px4', '.deps/agent-install/bin/MicroXRCEAgent',
                 'ros2_ws/install/setup.bash', 'ros2_ws/install/uav_lab_tools/lib/uav_lab_tools/labctl']
        for path in paths:
            add(path, (ROOT/path).exists(), 'exists' if (ROOT/path).exists() else 'run bootstrap/build')
        lock = json.loads((ROOT/'dependencies/lock.json').read_text())
        for name, path in [('px4','.deps/px4'), ('px4_msgs','ros2_ws/src/px4_msgs'), ('agent','.deps/agent')]:
            code, out, err = command(['git','-C',str(ROOT/path),'rev-parse','HEAD'])
            add(name+' revision', code==0 and out==lock['repositories'][name]['ref'], out or err)
            code, out, err = command(['git','-C',str(ROOT/path),'status','--porcelain','--untracked-files=no'])
            add(name+' clean', code==0 and not out, out or err or 'clean')
        stamp = ROOT/'.deps/agent-install/build-manifest.json'
        expected = {'repository': lock['repositories']['agent'], 'transitives': lock['agent_transitives'], 'p2p_profile': False}
        add('private Agent build manifest', stamp.exists() and json.loads(stamp.read_text()) == expected,
            'matches lock' if stamp.exists() else 're-run bootstrap --only agent')
        for name, match in check_messages(ROOT).items():
            add(name+' schema', match, 'matching fields/constants/version' if match else 'schema mismatch or missing')
    code, out, err = command(['nvidia-smi','--query-gpu=name,memory.total','--format=csv,noheader'])
    add('GPU/CUDA', code==0, out or err, required=sensors)
    return report

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', action='store_true')
    p.add_argument('--sensors', action='store_true')
    p.add_argument('--json', action='store_true')
    args = p.parse_args()
    report = checks(args.runtime, args.sensors)
    passed = all(v['ok'] for v in report if v['required'])
    if args.json:
        print(json.dumps({'passed': passed, 'checks': report}, ensure_ascii=False, indent=2))
    else:
        for v in report:
            state = 'OK' if v['ok'] else 'FAIL' if v['required'] else 'OPTIONAL'
            print(f"{state:8} {v['name']}: {v['detail']}")
    return 0 if passed else 1

if __name__ == '__main__':
    raise SystemExit(main())
