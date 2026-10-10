"""The CPU fixture must satisfy the same source-window contract as rendered data."""
import pytest
pytest.importorskip('rosbag2_py')


def test_analytical_fixture_records_source_clock_covering_imu_and_last_scan(tmp_path):
    from make_lio_fixture import generate
    from uav_lab_localization.benchmark import read_dataset
    dataset=tmp_path/'fixture';generate(dataset)
    try:clocks=[msg.clock.sec+msg.clock.nanosec/1e9 for _,msg,*unused in read_dataset(dataset,['/clock'])]
    except ValueError:clocks=[]
    assert clocks, 'analytical replay fixture omitted its source clock'
    assert len(clocks)==8001 and clocks[0]==100. and clocks[-1]==140.
    assert all(b>a for a,b in zip(clocks,clocks[1:]))
    primary=[msg.header.stamp.sec+msg.header.stamp.nanosec/1e9 for _,msg,*unused in read_dataset(dataset,['/uav001/lidar/points'])]
    assert primary[0]==clocks[0] and primary[-1]==clocks[-1]
