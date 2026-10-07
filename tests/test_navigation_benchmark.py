import importlib.util
import numpy as np
from uav_lab_navigation.fixture import write_fixture


def test_fixed_observations_support_bypass_and_block_interior_goal(tmp_path):
    assert importlib.util.find_spec('uav_lab_navigation.benchmark'), 'navigation benchmark missing'
    from uav_lab_navigation.benchmark import benchmark
    write_fixture(tmp_path/'inputs')
    result=benchmark(tmp_path/'inputs',tmp_path/'output')
    assert result['passed'] and result['plan']['success']
    assert result['rejected_goal']['success'] is False
    assert result['all_segments_clear']
    assert result['length_m']>np.linalg.norm([2.5,3.6,0])
    assert result['input_sha256']
