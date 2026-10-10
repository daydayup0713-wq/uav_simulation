import builtins
import pytest
from uav_lab_tools import data_cli


def test_explicit_replay_rate_preserves_source_time_and_scales_watchdog():
    options=data_cli.parser().parse_args(['replay','dataset','--rate','0.5'])
    assert options.rate==.5
    assert data_cli.playback_timeout({'bag_duration_s':100},rate=.5)>=215


@pytest.fixture
def forbid_ros_import(monkeypatch):
    original=builtins.__import__
    def checked(name,*args,**kwargs):
        if name=='rclpy' or name.startswith('rclpy.'):
            raise AssertionError('invalid options must be rejected before importing ROS')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',checked)


@pytest.mark.parametrize('value',['nan','0','-1','10'])
def test_invalid_replay_rate_rejected_before_ros_initialization(value,capsys,forbid_ros_import):
    assert data_cli.run_cli(['replay','dataset','--rate',value])==1
    assert 'rate' in capsys.readouterr().out


@pytest.mark.parametrize('command',['record','sensors'])
@pytest.mark.parametrize('value',['nan','0','3601'])
def test_invalid_duration_rejected_before_importing_ros(command,value,capsys,forbid_ros_import):
    assert data_cli.run_cli([command,'--duration',value])==1
    assert 'duration' in capsys.readouterr().out
