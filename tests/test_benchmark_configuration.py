import copy
import json
from pathlib import Path
from uav_lab_localization.benchmark import benchmark_config


def test_fixed_replay_uses_recorded_extrinsics_in_an_immutable_local_config(tmp_path):
    root=Path(__file__).resolve().parents[1]
    current=json.loads((root/'configs/sensors.json').read_text())
    recorded=copy.deepcopy(current);recorded['lidar']['xyz']=[.1,.2,.5]
    original=(root/'configs/slam/config_sensors.json').read_bytes()
    config=benchmark_config(root,tmp_path,recorded)
    actual=json.loads((config/'config_sensors.json').read_text())['sensors']
    assert actual['T_lidar_imu'][:3]==[-.1,-.2,-.5]
    assert (root/'configs/slam/config_sensors.json').read_bytes()==original
    assert config.parent==tmp_path


def test_relative_output_provides_an_absolute_backend_config(tmp_path,monkeypatch):
    root=Path(__file__).resolve().parents[1]
    sensors=json.loads((root/'configs/sensors.json').read_text())
    monkeypatch.chdir(tmp_path)
    config=benchmark_config(root,Path('relative-output'),sensors)
    assert config.is_absolute() and config.is_dir()
