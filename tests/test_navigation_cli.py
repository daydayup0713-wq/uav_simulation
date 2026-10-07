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
