"""A scene-only run and its operators must use the same owned DDS transport."""
from pathlib import Path

import pytest

import supervise


@pytest.mark.parametrize('options', [
    {'scene': 'circle-eight', 'sensor_profile': None},
    {'scene': None, 'sensor_profile': 'livox'},
])
def test_experiment_transport_archives_and_uses_owned_profile(tmp_path, options):
    configuration = tmp_path / 'configuration'
    configuration.mkdir()
    source = tmp_path / 'locked.xml'
    source.write_text('<profiles>locked local transport</profiles>')
    inherited = {'FASTRTPS_DEFAULT_PROFILES_FILE': '/foreign/run.xml',
                 'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '77'}
    environment, relative = supervise.configure_run_transport(
        inherited, configuration, source, **options)
    assert relative == 'configuration/fastdds-local.xml'
    assert environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(configuration / 'fastdds-local.xml')
    assert environment['ROS_LOCALHOST_ONLY'] == '0'
    assert environment['ROS_DOMAIN_ID'] == '77'
    assert Path(environment['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes() == source.read_bytes()
    assert inherited['FASTRTPS_DEFAULT_PROFILES_FILE'] == '/foreign/run.xml'


def test_legacy_flight_has_no_accidental_profile_or_previous_run_inheritance(tmp_path):
    configuration = tmp_path / 'configuration'
    configuration.mkdir()
    environment, relative = supervise.configure_run_transport(
        {'FASTRTPS_DEFAULT_PROFILES_FILE': '/previous/run.xml', 'ROS_LOCALHOST_ONLY': '0'},
        configuration, tmp_path / 'unused.xml')
    assert relative is None
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in environment
    assert environment['ROS_LOCALHOST_ONLY'] == '1'
    assert not (configuration / 'fastdds-local.xml').exists()
