import pytest
from backend_launch import core_command


def test_core_outputs_are_private_and_cannot_publish_px4_commands(tmp_path):
    first=core_command('fast_livo2','/private/mapping',tmp_path)
    second=core_command('fast_livo2_rtk','/private/mapping',tmp_path)
    assert '/Odometry:=/uav001/backends/fast_livo2/raw_odometry' in first
    assert '/ublox_driver/receiver_pvt:=/uav001/backends/fast_livo2_rtk/gnss_pvt' in second
    assert '/tf:=/uav001/backends/fast_livo2/tf' in first
    assert not any('/fmu/' in item or 'ground_truth' in item for item in first+second)
    with pytest.raises(ValueError):core_command('fake','/private/mapping',tmp_path)
