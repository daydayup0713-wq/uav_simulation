import json
from pathlib import Path
from sensor_model import prepare_sensors


def test_long_corridor_has_a_separate_measured_dense_scan_contract_and_preserves_baseline(tmp_path):
    root=Path(__file__).resolve().parents[1]
    legacy=json.loads((root/'configs/navigation-sensors.json').read_text())
    configuration=prepare_sensors(root,tmp_path,'navigation',scene='corridor')['calibration']
    assert configuration['lidar']['horizontal_samples']==360
    assert configuration['lidar']['vertical_samples']==128
    assert json.loads((root/'configs/navigation-sensors.json').read_text())==legacy
    assert configuration['lidar']['hz']==legacy['lidar']['hz']
