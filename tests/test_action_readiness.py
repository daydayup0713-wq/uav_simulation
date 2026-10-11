from types import SimpleNamespace
import pytest
from uav_lab_tools.cli import Operator


def operator(failures):
    op=object.__new__(Operator);calls=[]
    class Action:
        def wait_for_server(self,**kw):return True
        def send_goal_async(self,goal):calls.append(goal.operation);return len(calls)
    op.action=Action();op.action_type=SimpleNamespace(Goal=lambda:SimpleNamespace(operation=0))
    def wait(future,timeout=None):
        if future<=failures:raise RuntimeError('operation timeout')
        return SimpleNamespace(accepted=False)
    op.wait=wait
    return op,calls


def test_lost_first_discovery_reply_is_probed_without_submitting_a_flight_command():
    op,calls=operator(1)
    assert hasattr(op,'prepare_action'),'cold action request/reply handshake missing'
    op.prepare_action();op.prepare_action()
    assert calls==[255,255]


def test_uncertain_discovery_never_retries_or_submits_real_motion():
    op,calls=operator(3)
    assert hasattr(op,'prepare_action'),'cold action request/reply handshake missing'
    with pytest.raises(RuntimeError,match='discovery'):op.prepare_action()
    assert calls==[255,255,255]
