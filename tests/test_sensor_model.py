import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

def module():
    assert importlib.util.find_spec('sensor_model') is not None, 'sensor model generator missing'
    import sensor_model
    return sensor_model

def test_sensor_extrinsics_and_scan_generated_together(tmp_path):
    result = module().prepare_sensors(ROOT, tmp_path)
    model = ET.parse(result['model']).getroot().find('model')
    lidar = model.find(".//sensor[@name='lab_lidar']")
    assert [float(x) for x in lidar.findtext('pose').split()] == [0, 0, .16, 0, 0, 0]
    assert int(lidar.findtext('lidar/scan/horizontal/samples')) == 360
    assert int(lidar.findtext('lidar/scan/vertical/samples')) == 16
    assert lidar.findtext('gz_frame_id') == 'lidar_link'
    assert result['calibration']['lidar']['xyz'] == [0, 0, .16]
    assert model.find('include/uri').text == 'model://x500'
    world = ET.parse(result['world']).getroot().find('world')
    assert world.find("include/uri").text == 'model://x500_sensors'

def test_optical_rotation_right_down_forward():
    m = module()
    quaternion = m.quaternion_from_rpy([-math.pi/2, 0, -math.pi/2])
    assert quaternion == pytest.approx([-.5, .5, -.5, .5])
    # Optical z forward, optical x right, optical y down, expressed in FLU.
    def rotate(v):
        x,y,z,w = quaternion
        qv = (x,y,z)
        cross = lambda a,b: (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
        uv = cross(qv,v); uuv = cross(qv,uv)
        return [v[i]+2*(w*uv[i]+uuv[i]) for i in range(3)]
    assert rotate((0,0,1)) == pytest.approx((1,0,0))
    assert rotate((1,0,0)) == pytest.approx((0,-1,0))
    assert rotate((0,1,0)) == pytest.approx((0,0,-1))

def test_profile_selection_keeps_cpu_world():
    m = module()
    assert m.select_profile(ROOT, 'flight') == ROOT/'simulation/worlds/lab.sdf'
    with pytest.raises(ValueError, match='profile'):
        m.select_profile(ROOT, '../../etc/passwd')

def test_navigation_profile_has_archived_wide_lidar_and_physical_divider(tmp_path):
    m=module()
    assert 'profile' in __import__('inspect').signature(m.prepare_sensors).parameters, 'navigation sensor profile missing'
    result=m.prepare_sensors(ROOT,tmp_path,profile='navigation')
    lidar=ET.parse(result['model']).getroot().find(".//sensor[@name='lab_lidar']")
    assert int(lidar.findtext('lidar/scan/vertical/samples'))==64
    assert float(lidar.findtext('lidar/scan/vertical/max_angle'))==pytest.approx(math.pi/2-.01)
    world=ET.parse(result['world']).getroot().find('world')
    assert world.find("model[@name='navigation_divider']/link/collision/geometry/box/size") is not None
    assert result['calibration']['lidar']['vertical_samples']==64


def test_complex_scene_can_keep_synchronous_baseline_sensor_contract(tmp_path):
    result=module().prepare_sensors(ROOT,tmp_path,profile='navigation',scene='circle-eight')
    lidar=ET.parse(result['model']).getroot().find(".//sensor[@name='lab_lidar']")
    assert lidar is not None and result['calibration']['lidar'].get('measurement_time')!='per_beam'
    assert (tmp_path/'configuration/scene/route.json').exists()
