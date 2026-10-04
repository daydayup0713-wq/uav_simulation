import importlib.util
import hashlib
import json
from pathlib import Path
import pytest

def module():
    assert importlib.util.find_spec('uav_lab_tools.datasets') is not None, 'dataset tools missing'
    from uav_lab_tools import datasets
    return datasets

def test_replay_rejects_control_and_unknown_topics():
    d=module()
    assert d.replay_topics({'/clock':'rosgraph_msgs/msg/Clock','/uav001/lidar/points':'sensor_msgs/msg/PointCloud2'}) == ['/clock','/uav001/lidar/points']
    for topic in ('/fmu/in/vehicle_command','/fmu/out/vehicle_status_v1','/malicious/topic'):
        with pytest.raises(ValueError,match='forbidden'):
            d.replay_topics({topic:'px4_msgs/msg/VehicleCommand'})

def test_replay_rejects_wrong_type_even_on_allowed_topic():
    with pytest.raises(ValueError,match='type'):
        module().replay_topics({'/clock':'px4_msgs/msg/VehicleCommand'})

@pytest.mark.parametrize('domain,active',[(42,[42]),(-1,[]),(233,[])])
def test_replay_domain_conflicts_rejected(domain,active):
    with pytest.raises(ValueError,match='domain'):
        module().check_replay_domain(domain,active)

def test_replay_independent_domain_allowed():
    module().check_replay_domain(77,[42])

def test_archive_checksums_detect_tampering(tmp_path):
    config=tmp_path/'configuration';config.mkdir()
    (config/'calibration.json').write_text('{}')
    (tmp_path/'dataset.json').write_text(json.dumps({'complete':True,'configuration_sha256':{'calibration.json':hashlib.sha256(b'{}').hexdigest()}}))
    assert module().load_dataset(tmp_path)['complete']
    (config/'calibration.json').write_text('{"changed":true}')
    with pytest.raises(ValueError,match='checksum'):
        module().load_dataset(tmp_path)

def test_incomplete_recording_is_not_claimed_as_valid(tmp_path):
    (tmp_path/'dataset.json').write_text(json.dumps({'complete':False,'reason':'interrupted'}))
    with pytest.raises(ValueError,match='incomplete'):
        module().load_dataset(tmp_path)

def test_checksum_path_traversal_rejected(tmp_path):
    (tmp_path/'dataset.json').write_text(json.dumps({'complete':True,'configuration_sha256':{'../outside': 'deadbeef'}}))
    with pytest.raises(ValueError,match='path'):
        module().load_dataset(tmp_path)

def test_slow_simulation_replay_uses_receive_timeline():
    from uav_lab_tools import data_cli
    assert getattr(data_cli,'playback_timeout',lambda report:0)({'sim_duration_s':30.,'bag_duration_s':120.})>=135

def test_owned_cleanup_stops_child_after_wrapper_already_exited(tmp_path):
    import os,signal,subprocess,time
    from uav_lab_tools.data_cli import stop_owned
    pidfile=tmp_path/'child.pid'
    code="import os,signal,time,sys; child=os.fork();\nif child: os._exit(0)\nsignal.signal(signal.SIGINT,signal.SIG_IGN)\nopen(sys.argv[1],'w').write(str(os.getpid()))\ntime.sleep(300)\n"
    wrapper=subprocess.Popen(['/usr/bin/python3','-c',code,str(pidfile)],start_new_session=True)
    def alive(pid):
        try:return open('/proc/'+str(pid)+'/stat').read().split()[2]!='Z'
        except FileNotFoundError:return False
    try:
        end=time.monotonic()+2
        while not pidfile.exists() and time.monotonic()<end:time.sleep(.02)
        child=int(pidfile.read_text());wrapper.wait(timeout=2)
        assert alive(child)
        stop_owned(wrapper)
        end=time.monotonic()+1
        while alive(child) and time.monotonic()<end:time.sleep(.02)
        assert not alive(child)
    finally:
        try:os.killpg(wrapper.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        wrapper.wait(timeout=2)
