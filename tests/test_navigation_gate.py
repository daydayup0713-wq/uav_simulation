import pytest
from uav_lab_bridge.navigation_gate import NavigationGate


def test_source_time_heartbeat_and_permanent_failure():
    gate=NavigationGate()
    assert not gate.ready(10,20)
    gate.observe(True,19.8,'odom',10)
    assert gate.ready(10.2,20.1)
    assert not gate.ready(10.2,22)
    assert gate.failed
    gate.observe(True,22,'odom',10.3)
    assert not gate.ready(10.3,22)


@pytest.mark.parametrize('source,frame',[(21,'odom'),(float('nan'),'odom'),(19.8,'map')])
def test_invalid_source_does_not_authorize_motion(source,frame):
    gate=NavigationGate();gate.observe(True,source,frame,10)
    assert not gate.ready(10,20)
