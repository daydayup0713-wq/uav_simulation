import importlib.util
import hashlib
import json
import numpy as np
from test_occupancy import implementation


def test_fixed_sensor_observations_map_obstacle_without_geometry_input(tmp_path):
    assert importlib.util.find_spec('uav_lab_navigation.fixture'), 'fixed ray fixture missing'
    from uav_lab_navigation.fixture import write_fixture
    write_fixture(tmp_path/'fixture')
    meta=json.loads((tmp_path/'fixture/manifest.json').read_text())
    raw=tmp_path/'fixture/observations.npz'
    assert hashlib.sha256(raw.read_bytes()).hexdigest()==meta['sha256']
    with np.load(raw,allow_pickle=False) as data:
        assert set(data.files)=={'origins','endpoints'}
        grid=implementation().VoxelMap()
        for origin,endpoints in zip(data['origins'],data['endpoints']):
            grid.integrate(origin,endpoints)
    assert grid.state((1.5,0,2)) != implementation().FREE  # surface or occluded interior
    assert grid.state((0,2,2)) == implementation().FREE
    assert len(grid.occupied_points())>100
