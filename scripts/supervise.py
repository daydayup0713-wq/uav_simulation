#!/usr/bin/python3
"""Own the lab process groups; gate startup on actual world/telemetry readiness."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]

def isolated_environment(base, run_dir):
    return {**base, 'GZ_PARTITION': 'uav-lab-'+uuid.uuid4().hex,
            'LAB_RUN_DIR': str(run_dir), 'ROS_LOG_DIR': str(Path(run_dir)/'ros')}

def owned_run_ready(runtime, supervisor_pid):
    try:
        runtime = Path(runtime)
        run = Path((runtime/'current-run').read_text().strip())
        return (run.parent == runtime and (run/'ready').exists()
                and json.loads((run/'manifest.json').read_text())['supervisor_pid'] == supervisor_pid)
    except (OSError, KeyError, ValueError):
        return False

def check_port(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(('0.0.0.0', port))
        except OSError as exc:
            raise RuntimeError(f'UDP port {port} unavailable: {exc}') from exc

class ManagedProcesses:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.processes = []
        self.handles = []

    def start(self, name, command, **kwargs):
        log = (self.directory / (name+'.log')).open('w')
        self.handles.append(log)
        process = subprocess.Popen([str(v) for v in command], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, **kwargs)
        self.processes.append((name, process))
        (self.directory/'processes.json').write_text(json.dumps({n: p.pid for n,p in self.processes}, indent=2)+'\n')
        return process

    def check(self, ignore=()):
        for name, process in self.processes:
            if name not in ignore and process.poll() is not None:
                raise RuntimeError(f'{name} exited ({process.returncode}); see {self.directory / (name+".log")}')

    def close(self):
        for name, process in reversed(self.processes):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic()+5
        for _, process in self.processes:
            try:
                process.wait(timeout=max(.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        # A wrapper can exit before its descendants. Kill the owned group even
        # when Popen.poll() says the group leader is gone.
        for _, process in self.processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3)
        for handle in self.handles:
            handle.close()

def wait_for(manager, check, description, timeout=30):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        manager.check()
        try:
            if check():
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(.2)
    raise RuntimeError('readiness timeout: '+description)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--rviz', action='store_true')
    options = p.parse_args()
    runtime = ROOT / '.runtime'
    runtime.mkdir(exist_ok=True)
    lock = (runtime / 'lab.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('another lab instance owns the runtime lock', flush=True)
        return 1
    run_dir = runtime / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    run_dir.mkdir()
    manager = ManagedProcesses(run_dir)
    env = isolated_environment(dict(os.environ), run_dir)
    env.update(GZ_SIM_RESOURCE_PATH=str(ROOT / '.deps/px4/Tools/simulation/gz/models'))
    (runtime / 'current-run').write_text(str(run_dir)+'\n')
    metadata = {'run_id': run_dir.name, 'supervisor_pid': os.getpid(), 'dependencies': json.loads((ROOT/'dependencies/lock.json').read_text()),
                'environment': {k: env.get(k) for k in ('ROS_DOMAIN_ID','GZ_PARTITION','RMW_IMPLEMENTATION')},
                'headless': options.headless,
                'parameters': {'COM_RC_IN_MODE': 4, 'COM_OF_LOSS_T': 1, 'COM_OBL_RC_ACT': 4,
                               'COM_DL_LOSS_T': 300, 'NAV_DLL_ACT': 0, 'UXRCE_DDS_SYNCT': 0,
                               'UXRCE_DDS_PTCFG': 1,
                               'UXRCE_DDS_DOM_ID': int(env.get('ROS_DOMAIN_ID','42'))}}
    snapshots = run_dir/'configuration'
    snapshots.mkdir()
    metadata['configuration_sha256'] = {}
    for relative in ('simulation/worlds/lab.sdf', 'configs/px4-start.sh', 'configs/lab.rviz', 'dependencies/lock.json'):
        contents = (ROOT/relative).read_bytes()
        (snapshots/Path(relative).name).write_bytes(contents)
        metadata['configuration_sha256'][relative] = hashlib.sha256(contents).hexdigest()
    metadata['frames'] = {'world': 'ENU', 'body': 'FLU', 'tf': 'odom -> base_link',
                          'px4_world': 'NED', 'px4_body': 'FRD'}
    metadata['platform_commit'] = subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'], text=True).strip()
    metadata['platform_dirty'] = bool(subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain'], text=True).strip())
    (run_dir / 'manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        check_port(8888)
        agent_env = {**env, 'LD_LIBRARY_PATH': str(ROOT/'.deps/agent-install/lib')+':'+str(ROOT/'.deps/agent-install/lib64')}
        manager.start('agent', [ROOT/'.deps/agent-install/bin/MicroXRCEAgent', 'udp4', '-p', '8888'], env=agent_env)
        manager.start('gazebo', ['gz', 'sim', '-r', '-s', ROOT/'simulation/worlds/lab.sdf'], env=env)
        def world_ready():
            result = subprocess.run(['gz', 'service', '-i', '-s', '/world/lab/scene/info'], env=env,
                                    capture_output=True, text=True, timeout=3)
            return 'Service providers' in result.stdout
        wait_for(manager, world_ready, 'Gazebo scene', 45)
        if not options.headless:
            manager.start('gazebo-gui', ['gz', 'sim', '-g'], env=env)
        manager.start('clock', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                              '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], env=env)
        px4_env = {**env, 'PX4_SYS_AUTOSTART': '4001', 'PX4_SIM_MODEL': 'gz_x500',
                   'PX4_GZ_STANDALONE': '1', 'PX4_GZ_WORLD': 'lab', 'PX4_GZ_MODEL_NAME': 'x500_0'}
        px4_env.update({'PX4_PARAM_'+k: str(v) for k,v in metadata['parameters'].items()})
        build = ROOT/'.deps/px4/build/px4_sitl_default'
        manager.start('px4', [build/'bin/px4', '-d', build/'etc', '-w', run_dir/'px4', '-s', ROOT/'configs/px4-start.sh'], env=px4_env)
        manager.start('bridge', ['ros2', 'run', 'uav_lab_bridge', 'bridge'], env=env)
        def vehicle_ready():
            result = subprocess.run(['ros2','run','uav_lab_tools','labctl','--timeout','2','status'],
                                    env=env, capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        wait_for(manager, vehicle_ready, 'PX4 fresh valid telemetry', 60)
        if options.rviz:
            manager.start('rviz', ['rviz2','-d', ROOT/'configs/lab.rviz'], env=env)
        (run_dir/'ready').write_text('ready\n')
        print('LAB READY '+str(run_dir), flush=True)
        failure_started = None
        while True:
            broken = [name for name, process in manager.processes if name in ('bridge','agent','clock') and process.poll() is not None]
            if broken and failure_started is None:
                failure_started = time.monotonic()
                (run_dir/'failure.txt').write_text('control link exited: '+','.join(broken)+'; preserving physics/PX4 for failsafe landing\n')
                print('control link failed; keeping physics/PX4 alive for 60s failsafe window', flush=True)
            manager.check(ignore=('bridge','agent','clock') if failure_started is not None else ())
            if failure_started is not None and time.monotonic()-failure_started > 60:
                raise RuntimeError('control link failed; failsafe observation window complete')
            time.sleep(.5)
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, OSError) as exc:
        (run_dir/'failure.txt').write_text(str(exc)+'\n')
        print(str(exc), flush=True)
        return 1
    finally:
        manager.close()
        (run_dir/'ready').unlink(missing_ok=True)
        (run_dir/'closed').write_text('all owned process groups stopped\n')
        lock.close()

if __name__ == '__main__':
    raise SystemExit(main())
