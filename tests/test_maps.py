import json
import numpy as np
import pytest

from uav_lab_localization.maps import archive_points, load_map, relocalize, map_change_allowed
from test_registration import corner


CALIBRATION = {'lidar': {'xyz': [0, 0, .16], 'rpy': [0, 0, 0]}}


def test_map_change_requires_fresh_landed_disarmed_status():
    status = {'armed': 'False', 'landed': 'True', 'fresh': 'True'}
    assert map_change_allowed(status, 10, 10.2)
    assert not map_change_allowed(status, 10, 11)
    assert not map_change_allowed({**status, 'armed': 'True'}, 10, 10.2)
    assert not map_change_allowed({**status, 'landed': 'False'}, 10, 10.2)
    assert not map_change_allowed({}, 10, 10.2)


def test_immutable_archive_roundtrip_and_corruption(tmp_path):
    path = tmp_path/'map'
    archive_points(corner(), path, CALIBRATION, {'test': 'fixture'})
    loaded = load_map(path, CALIBRATION)
    assert np.allclose(loaded['points'], corner())
    with pytest.raises(ValueError, match='exists'):
        archive_points(corner(), path, CALIBRATION, {})
    with pytest.raises(ValueError, match='calibration'):
        load_map(path, {'lidar': {'xyz': [0, 0, .2]}})
    with (path/'points.npy').open('ab') as stream:
        stream.write(b'corrupted')
    with pytest.raises(ValueError, match='checksum'):
        load_map(path, CALIBRATION)


def test_scan_relocalization_does_not_change_continuous_odom(tmp_path):
    path = tmp_path/'map'
    archive_points(corner(), path, CALIBRATION, {})
    loaded = load_map(path, CALIBRATION)
    scan = corner()-[.2, -.1, .15]
    odom = np.eye(4)
    odom[:3, 3] = [3, 4, 5]
    before = odom.copy()
    result = relocalize(loaded, scan, np.eye(4), odom)
    assert np.allclose(odom, before)
    assert np.allclose(result['map_to_odom'][:3, 3], [-2.8, -4.1, -4.85], atol=.01)
    with pytest.raises(ValueError, match='overlap|correspondences'):
        relocalize(loaded, scan+100, np.eye(4), odom)


def test_archive_paths_cannot_escape_root(tmp_path):
    path = tmp_path/'map'
    archive_points(corner(), path, CALIBRATION, {})
    manifest = json.loads((path/'map.json').read_text())
    manifest['files']['../outside'] = 'bad'
    (path/'map.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='path'):
        load_map(path, CALIBRATION)
