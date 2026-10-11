import os
from pathlib import Path
import pytest

pytest.importorskip('rclpy')


@pytest.mark.parametrize('previous_profile', ['', '/previous-run/configuration/foreign.xml'])
def test_replay_owns_locked_large_sensor_transport_for_observer_and_all_children(tmp_path,monkeypatch,previous_profile):
    import replay_backend
    root=Path(__file__).resolve().parents[2]
    configuration=tmp_path/'configuration';configuration.mkdir()
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE',previous_profile)
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
    original={'ROS_DOMAIN_ID':'77','ROS_LOCALHOST_ONLY':'1',
        'FASTRTPS_DEFAULT_PROFILES_FILE':previous_profile,'ROS_LOG_DIR':'owned-logs',
        'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp'}
    environment=replay_backend.configure_replay_transport(configuration,original)
    profile=configuration/'fastdds-local.xml'
    assert profile.read_bytes()==(root/'configs/fastdds-local.xml').read_bytes()
    for key in ('ROS_LOCALHOST_ONLY','FASTRTPS_DEFAULT_PROFILES_FILE','RMW_IMPLEMENTATION'):
        assert environment[key]==os.environ[key]
    assert environment['FASTRTPS_DEFAULT_PROFILES_FILE']==str(profile.resolve())
    assert environment['ROS_LOCALHOST_ONLY']=='0'
    assert environment['RMW_IMPLEMENTATION']=='rmw_fastrtps_cpp'
    assert environment['ROS_DOMAIN_ID']=='77'
    assert environment['ROS_LOG_DIR']=='owned-logs'
    assert original['ROS_LOCALHOST_ONLY']=='1'
