import importlib,importlib.util
import numpy as np


def evaluate(*args,**kw):
    assert importlib.util.find_spec('experiment_evaluation'),'independent complex-world evaluator missing'
    return importlib.import_module('experiment_evaluation').evaluate(*args,**kw)


def fixture():
    route={'controls':[[1.,0.,2.],[2.,0.,2.]]}
    truth=[{'stamp':float(t),'position':[float(t),0.,2.],'quaternion':[0.,0.,0.,1.],'phase':'MOVING'} for t in np.linspace(0,2,101)]
    references=[{'stamp':float(t),'id':'route','position':[float(t),0.,2.]} for t in np.linspace(0,2,101)]
    events=[{'event':'route_control_scheduled','index':0,'reference_error_m':.1,'join_velocity':[.2,0,0]},
            {'event':'route_result','success':True,'controls':2}]
    geometry={'boxes':[[[-2.,-2.,-.2],[4.,2.,0.]],[[0.,2.,0.],[2.,2.2,4.]]]}
    return geometry,route,truth,references,events


def test_actual_geometry_and_ordered_semantic_coverage_and_nonstop_handoffs():
    report=evaluate(*fixture(),expected_runs=1)
    assert report['passed'] and report['covered_controls']==2
    assert report['truth_tracking_p95_m']<1e-8


def test_skipped_controls_collision_and_stopped_handoff_cannot_be_success():
    geometry,route,truth,references,events=fixture()
    truth[50]['position']=[1.,2.,2.]
    events[0]['join_velocity']=[0.,0.,0.]
    report=evaluate(geometry,route,truth,references,events,expected_runs=1)
    assert not report['passed'] and 'body_clearance' in report['failed_checks'] and 'nonstop_handoffs' in report['failed_checks']
    report=evaluate(geometry,{'controls':[[40.,0.,2.],[2.,0.,2.]]},truth,references,events,expected_runs=1)
    assert 'ordered_controls' in report['failed_checks']


def test_attitude_rotates_full_body_envelope_before_clearance():
    geometry,route,truth,references,events=fixture()
    # Rotation projects a square body beyond its axis-aligned halfsize.
    geometry['boxes']=[[[.95,.53,1.],[1.05,.6,3.]]]
    truth[50]['quaternion']=[0.,0.,float(np.sin(np.pi/8)),float(np.cos(np.pi/8))]
    report=evaluate(geometry,route,truth,references,events,expected_runs=1)
    assert report['min_body_clearance_m']<0
