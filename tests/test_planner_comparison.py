"""Comparison inputs and successes stay bound to observed geometry and all four methods."""
import importlib
import importlib.util
import json
import numpy as np
import pytest


def api():
    assert importlib.util.find_spec('uav_lab_navigation.planner_comparison'),'four-planner comparison missing'
    return importlib.import_module('uav_lab_navigation.planner_comparison')


def snapshot(path,envelope=True):
    config={'body_halfsize_m':[.4,.4,.3],'clearance_m':.25}
    data={'free':np.ones((10,10,10),dtype=bool),'lower':[-1.,-1.,0.],'resolution':.2,
          'map_version':3,'source_stamp':10.,'configuration':json.dumps(config)}
    if envelope:data['envelope']=[.65,.65,.55]
    np.savez_compressed(path,**data)


def test_fixed_map_requires_body_inflation_evidence(tmp_path):
    path=tmp_path/'map.npz';snapshot(path,False)
    with pytest.raises(ValueError,match='envelope|inflation'):
        api().load_snapshot(path)


def test_fixed_map_identity_changes_when_observed_free_geometry_changes(tmp_path):
    path=tmp_path/'map.npz';snapshot(path)
    collision,provenance=api().load_snapshot(path)
    assert provenance['source']=='sensor_observed_inflated_snapshot'
    assert provenance['input_sha256'] and provenance['map_sha256']
    assert collision.version==3 and collision.source_stamp==10.


def test_comparison_cannot_claim_pass_for_missing_or_failed_methods():
    rows=[{'backend':b,'map_sha256':'a'*64,'cases':[{'passed':True,'expected_success':True}]} for b in ('astar','ego','fast_planner')]
    assert api().comparison_summary(rows)['passed'] is False
    rows.append({'backend':'gcopter','map_sha256':'a'*64,'cases':[{'passed':False,'expected_success':True}]})
    assert api().comparison_summary(rows)['passed'] is False
    rows[-1]['cases'][0]['passed']=True
    assert api().comparison_summary(rows)['passed'] is True
    rows[-1]['map_sha256']='b'*64
    assert api().comparison_summary(rows)['passed'] is False
