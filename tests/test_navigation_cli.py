import pytest
pytest.importorskip('rclpy')
from uav_lab_navigation.cli import Navigator


def test_demo_failure_stops_and_attempts_landing():
    nav=object.__new__(Navigator)
    class Operator:
        calls=[]
        def status(self):return {'armed':'False','landed':'True'}
        def service(self,name):self.calls.append(name)
        def flight(self,name,**kw):self.calls.append(name)
    nav.operator=Operator();nav.plan=lambda target:{'success':False,'reason':'GOAL_BLOCKED_OR_UNOBSERVED'}
    nav.goto=lambda target:{'success':False,'reason':'NO_PATH'}
    import pytest
    with pytest.raises(RuntimeError):nav.demo(1)
    assert nav.operator.calls==['arm','TAKEOFF','LAND']


def test_demo_waits_for_landing_diagnostic_after_confirmed_action():
    nav=object.__new__(Navigator)
    class Operator:
        stale=False
        def status(self):return {'armed':'True' if self.stale else 'False','landed':'True'}
        def spin(self,*args):self.stale=False
        def service(self,name):pass
        def flight(self,name,**kw):
            if name=='LAND':self.stale=True
    nav.operator=Operator();nav.plan=lambda target:{'success':False,'reason':'GOAL_BLOCKED_OR_UNOBSERVED'}
    nav.goto=lambda target:{'success':True,'reason':'reached'}
    assert nav.demo(1)['success']


def test_complex_route_demo_arms_explicitly_and_lands_on_route_failure(tmp_path):
    import json
    route=tmp_path/'route.json'
    route.write_text(json.dumps({'frame':'odom','controls':[[1,0,2],[0,0,2]]}))
    nav=object.__new__(Navigator)
    class Operator:
        calls=[]
        def status(self):return {'armed':'False','landed':'True'}
        def service(self,name):self.calls.append(name)
        def flight(self,name,**kw):self.calls.append(name)
    nav.operator=Operator();nav.route=lambda controls:{'success':False,'reason':'unobserved goal'}
    with pytest.raises(RuntimeError,match='unobserved'):
        nav.route_demo(route,1)
    assert nav.operator.calls==['arm','TAKEOFF','LAND']
