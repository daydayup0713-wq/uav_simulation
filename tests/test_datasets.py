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
