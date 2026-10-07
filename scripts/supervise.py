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
from sensor_model import select_profile, prepare_sensors
from slam_config import prepare_slam

ROOT = Path(__file__).resolve().parents[1]

def isolated_environment(base, run_dir):
    return {**base, 'GZ_PARTITION': 'uav-lab-'+uuid.uuid4().hex,
            'LAB_RUN_DIR': str(run_dir), 'ROS_LOG_DIR': str(Path(run_dir)/'ros')}

def source_identity(root, environment):
    revision = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], capture_output=True, text=True)
    if revision.returncode != 0:
        return environment.get('LAB_REVISION') or 'source-archive', None
    status = subprocess.run(['git', '-C', str(root), 'status', '--porcelain'], capture_output=True, text=True)
    return revision.stdout.strip(), bool(status.stdout.strip()) if status.returncode == 0 else None

def owned_run_ready(runtime, supervisor_pid):
    try:
        runtime = Path(runtime)
        run = Path((runtime/'current-run').read_text().strip())
        return (run.parent == runtime and (run/'ready').exists()
                and json.loads((run/'manifest.json').read_text())['supervisor_pid'] == supervisor_pid)
    except (OSError, KeyError, ValueError):
        return False

class FlightReadiness:
    """Wait out EKF/barometer initialization, not merely DDS discovery."""
    def __init__(self, settle=5., require_preflight=True):
        self.settle, self.since = settle, None
        self.require_preflight = require_preflight

    def update(self, status, now):
        requirements = [('fresh', 'True'), ('armed', 'False'), ('landed', 'True')]
        requirements.append(('preflight' if self.require_preflight else 'external_ready', 'True'))
        healthy = all(status.get(key) == expected for key, expected in requirements)
        if not healthy:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now-self.since >= self.settle

def read_vehicle_status(environment):
    try:
        result = subprocess.run(['ros2','run','uav_lab_tools','labctl','--timeout','2','status'],
                                env=environment, capture_output=True, text=True, timeout=5)
        return json.loads(result.stdout) if result.returncode == 0 else {}
    except (subprocess.TimeoutExpired, ValueError):
        return {}

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
    p.add_argument('--profile', choices=('flight', 'sensors', 'localization', 'slam', 'navigation'), default='flight')
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
                'headless': options.headless, 'profile': options.profile,
                'parameters': {'COM_RC_IN_MODE': 4, 'COM_OF_LOSS_T': 1, 'COM_OBL_RC_ACT': 4,
                               'COM_DL_LOSS_T': 300, 'NAV_DLL_ACT': 0, 'UXRCE_DDS_SYNCT': 0,
                               'UXRCE_DDS_PTCFG': 1,
                               'UXRCE_DDS_DOM_ID': int(env.get('ROS_DOMAIN_ID','42'))}}
    if options.profile in ('slam', 'navigation'):
        metadata['parameters'].update(EKF2_GPS_CTRL=0, SIM_GPS_USED=0, SENS_EN_GPSSIM=0,
            EKF2_EV_CTRL=11, EKF2_HGT_REF=3, EKF2_BARO_CTRL=0, EKF2_MAG_TYPE=5, EKF2_EV_DELAY=0)
    snapshots = run_dir/'configuration'
    snapshots.mkdir()
    localization = options.profile in ('localization', 'slam', 'navigation')
    if localization and not (ROOT/'.deps/slam/install/build-manifest.json').is_file():
        raise RuntimeError('private CPU localization backend missing; run bootstrap_slam.py')
    sensor = prepare_sensors(ROOT, run_dir, 'navigation' if options.profile == 'navigation' else 'sensors') if options.profile != 'flight' else None
    world_path = sensor['world'] if sensor else select_profile(ROOT, options.profile)
    if sensor:
        env['GZ_SIM_RESOURCE_PATH'] = str(sensor['resource_path'])+':'+env['GZ_SIM_RESOURCE_PATH']
        metadata['calibration'] = sensor['calibration']
    metadata['configuration_sha256'] = {}
    files = ['simulation/worlds/lab.sdf', 'configs/px4-start.sh', 'configs/lab.rviz', 'dependencies/lock.json']
    if sensor:
        files += ['simulation/worlds/room.sdf', 'configs/sensors.json', 'configs/sensors.rviz', 'configs/recording-qos.yaml']
    if options.profile == 'navigation':
            files += ['simulation/worlds/navigation.sdf', 'configs/navigation-sensors.json', 'configs/navigation.json']
    for relative in files:
        contents = (ROOT/relative).read_bytes()
        # Generated world has the payload include; do not replace it with its template.
        if relative != 'simulation/worlds/room.sdf':
            (snapshots/Path(relative).name).write_bytes(contents)
        metadata['configuration_sha256'][relative] = hashlib.sha256(contents).hexdigest()
    metadata['snapshot_sha256'] = {str(path.relative_to(snapshots)): hashlib.sha256(path.read_bytes()).hexdigest()
                                   for path in snapshots.rglob('*') if path.is_file()}
    metadata['frames'] = {'world': 'ENU', 'body': 'FLU', 'tf': 'odom -> base_link',
                          'px4_world': 'NED', 'px4_body': 'FRD'}
    metadata['platform_commit'], metadata['platform_dirty'] = source_identity(ROOT, env)
    (run_dir / 'manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        check_port(8888)
        agent_env = {**env, 'LD_LIBRARY_PATH': str(ROOT/'.deps/agent-install/lib')+':'+str(ROOT/'.deps/agent-install/lib64')}
        manager.start('agent', [ROOT/'.deps/agent-install/bin/MicroXRCEAgent', 'udp4', '-p', '8888'], env=agent_env)
        gz_command = ['gz', 'sim', '-r', '-s', world_path]
        if sensor and options.headless:
            gz_command.append('--headless-rendering')
        manager.start('gazebo', gz_command, env=env)
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
        bridge_command = ['ros2', 'run', 'uav_lab_bridge', 'bridge']
        if options.profile in ('slam', 'navigation'):
            bridge_command += ['--ros-args','-p','external_odometry:=true']
        if options.profile == 'navigation':
            bridge_command += ['-p','navigation_required:=true']
        manager.start('bridge', bridge_command, env=env)
        if sensor:
            manager.start('sensor-bridge', ['ros2','run','ros_gz_bridge','parameter_bridge',
                                           '--ros-args','-p','config_file:='+str(sensor['bridge'])], env=env)
            manager.start('sensors', ['ros2','run','uav_lab_tools','sensors','--ros-args',
                                     '-p','calibration_file:='+str(sensor['calibration_path'])], env=env)
        flight_readiness = FlightReadiness(require_preflight=options.profile not in ('slam', 'navigation'))
        def vehicle_ready():
            return flight_readiness.update(read_vehicle_status(env), time.monotonic())
        if not localization:
            wait_for(manager, vehicle_ready, 'PX4 fresh landed telemetry and stable preflight checks', 60)
        if sensor:
            result = subprocess.run(['ros2','run','uav_lab_tools','datactl','sensors','--duration','3'],
                                    env=env, capture_output=True, text=True, timeout=20)
            (run_dir/'sensor-readiness.json').write_text(result.stdout)
            if result.returncode:
                raise RuntimeError('sensor data readiness failed: '+result.stdout.strip()+' '+result.stderr.strip())
        if localization:
            slam_config = prepare_slam(ROOT, snapshots/'slam', sensor['calibration'])
            manager.start('lio', ['ros2','run','glim_ros','glim_rosnode','--ros-args',
                                  '-p','config_path:='+str(slam_config), '-p','use_sim_time:=true',
                                  '-r','/tf:=/uav001/localization/raw_tf'], env=env)
            manager.start('localization', ['ros2','run','uav_lab_localization','localization'], env=env)
            def lio_ready():
                result = subprocess.run(['ros2','run','uav_lab_localization','slamctl','status','--timeout','2'],
                                        env=env, capture_output=True, text=True, timeout=5)
                (run_dir/'localization-readiness.json').write_text(result.stdout)
                return result.returncode == 0
            wait_for(manager, lio_ready, 'validated continuous LIO', 45)
            wait_for(manager, vehicle_ready, 'PX4 ready after localization initialization', 60)
        if options.profile == 'navigation':
            manager.start('navigation', ['ros2','run','uav_lab_navigation','navigation'], env=env)
            def navigation_ready():
                result = subprocess.run(['ros2','run','uav_lab_navigation','navctl','--timeout','2','status'],
                                        env=env, capture_output=True, text=True, timeout=5)
                (run_dir/'navigation-readiness.json').write_text(result.stdout)
                return result.returncode == 0
            wait_for(manager, navigation_ready, 'fresh sensor-time navigation map', 45)
        if options.rviz:
            manager.start('rviz', ['rviz2','-d', ROOT/'configs'/('navigation.rviz' if options.profile == 'navigation' else 'slam.rviz' if localization else 'sensors.rviz' if sensor else 'lab.rviz')], env=env)
        (run_dir/'ready').write_text('ready\n')
        print('LAB READY '+str(run_dir), flush=True)
        failure_started = None
        while True:
            control_components = ('bridge','agent','clock','lio','localization','navigation')
            broken = [name for name, process in manager.processes if name in control_components and process.poll() is not None]
            if broken and failure_started is None:
                failure_started = time.monotonic()
                (run_dir/'failed-processes.json').write_text(json.dumps({name: {'pid': process.pid, 'returncode': process.poll()} for name, process in manager.processes if name in broken}, indent=2)+'\n')
                (run_dir/'failure.txt').write_text('control link exited: '+','.join(broken)+'; preserving physics/PX4 for failsafe landing\n')
                print('control link failed; keeping physics/PX4 alive for 60s failsafe window', flush=True)
            manager.check(ignore=control_components if failure_started is not None else ())
            if failure_started is not None and time.monotonic()-failure_started > 60:
                raise RuntimeError('control link failed; failsafe observation window complete')
            time.sleep(.5)
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, OSError) as exc:
        with (run_dir/'failure.txt').open('a') as failure:
            failure.write(str(exc)+'\n')
        print(str(exc), flush=True)
        return 1
    finally:
        manager.close()
        (run_dir/'ready').unlink(missing_ok=True)
        (run_dir/'closed').write_text('all owned process groups stopped\n')
        lock.close()

if __name__ == '__main__':
    raise SystemExit(main())
