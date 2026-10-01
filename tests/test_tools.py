import subprocess
import sys
import time
import pytest
from uav_lab_tools.arguments import parser
from supervise import ManagedProcesses, check_port, isolated_environment, owned_run_ready
import socket
from pathlib import Path

def test_cli_rejects_nonfinite_and_missing_position():
    for argv in (['goto', '--x', 'nan', '--y', '0', '--z', '2'], ['goto'], ['takeoff', '--height', '-1']):
        with pytest.raises(SystemExit):
            parser().parse_args(argv)
    assert parser().parse_args(['goto', '--x', '3', '--y', '0', '--z', '2']).z == 2

def test_port_conflict_is_detected_without_killing_listener():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        with pytest.raises(RuntimeError, match='port'):
            check_port(sock.getsockname()[1])
        assert sock.fileno() >= 0

def test_cleanup_terminates_only_owned_processes(tmp_path):
    other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    manager = ManagedProcesses(tmp_path)
    child = manager.start('test', [sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        manager.close()
        assert child.poll() is not None
        assert other.poll() is None
    finally:
        other.terminate()
        other.wait(timeout=3)

def test_ros_runtime_does_not_load_agents_private_dds_libraries():
    root = Path(__file__).resolve().parents[1]
    if not (root/'ros2_ws/install/setup.bash').exists():
        pytest.skip('build ROS workspace first')
    result = subprocess.check_output([str(root/'scripts/env.sh'), '/usr/bin/env'], text=True)
    ld = next(v for v in result.splitlines() if v.startswith('LD_LIBRARY_PATH='))
    assert 'agent-install' not in ld

def test_cleanup_reaches_descendants_after_group_leader_exits(tmp_path):
    manager = ManagedProcesses(tmp_path)
    pidfile = tmp_path/'descendant.pid'
    code = 'import subprocess,pathlib; p=subprocess.Popen(["/usr/bin/python3","-c","import time; time.sleep(60)"]); pathlib.Path('+repr(str(pidfile))+').write_text(str(p.pid))'
    parent = manager.start('parent', [sys.executable, '-c', code])
    parent.wait(timeout=3)
    descendant = int(pidfile.read_text())
    manager.close()
    stat = Path(f'/proc/{descendant}/stat')
    try:
        deadline = time.monotonic()+1
        while stat.exists() and stat.read_text().split()[2] != 'Z' and time.monotonic()<deadline:
            time.sleep(.01)
        assert not stat.exists() or stat.read_text().split()[2] == 'Z'
    finally:
        import os, signal
        try:
            os.kill(descendant, signal.SIGKILL)
        except ProcessLookupError:
            pass

def test_each_run_has_a_unique_gazebo_partition(tmp_path):
    base = {'GZ_PARTITION': 'existing-world'}
    first = isolated_environment(base, tmp_path/'a')
    second = isolated_environment(base, tmp_path/'b')
    assert first['GZ_PARTITION'] != second['GZ_PARTITION']
    assert first['GZ_PARTITION'] != 'existing-world'
    assert base['GZ_PARTITION'] == 'existing-world'

def test_readiness_does_not_accept_another_supervisors_run(tmp_path):
    import json
    run = tmp_path/'run'
    run.mkdir()
    (tmp_path/'current-run').write_text(str(run))
    (run/'ready').write_text('ready')
    (run/'manifest.json').write_text(json.dumps({'supervisor_pid': 123}))
    assert not owned_run_ready(tmp_path, 456)
    assert owned_run_ready(tmp_path, 123)
    (run/'ready').unlink()
    assert not owned_run_ready(tmp_path, 123)
