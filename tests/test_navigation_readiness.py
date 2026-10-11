"""Cold DDS reply discovery must never retry a real navigation request."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from uav_lab_interfaces.action import Navigate, NavigateRoute
from uav_lab_navigation.cli import Navigator, pose


@pytest.mark.parametrize('kind,name', [(Navigate, 'navigate'), (NavigateRoute, 'route')])
@pytest.mark.parametrize('lost_replies', [1, 3])
def test_cold_navigation_reply_loss_only_retries_invalid_probes(monkeypatch, kind, name, lost_replies):
    import uav_lab_navigation.cli as cli
    calls = []
    confirmed = False
    class Action:
        def __init__(self, *args): pass
        def wait_for_server(self, **kwargs): return True
        def send_goal_async(self, goal):
            invalid = not goal.controls if kind is NavigateRoute else goal.goal.header.frame_id == ''
            calls.append(invalid)
            return ('probe', sum(calls)) if invalid else ('motion', confirmed)
        def destroy(self): pass
    def wait(future, timeout=None):
        nonlocal confirmed
        if future[0] == 'probe':
            if future[1] <= lost_replies: raise RuntimeError('operation timeout')
            confirmed = True
            return SimpleNamespace(accepted=False)
        if future[0] == 'motion':
            if not future[1]: raise RuntimeError('operation timeout')
            return SimpleNamespace(accepted=True, get_result_async=lambda: ('result', None))
        return SimpleNamespace(result=SimpleNamespace(success=True, reason='reached'))
    nav = object.__new__(Navigator)
    nav.node = None
    nav.operator = SimpleNamespace(wait=wait, service=lambda name: None)
    monkeypatch.setattr(cli, 'ActionClient', Action)
    goal = Navigate.Goal(goal=pose([1, 1, 2])) if kind is Navigate else NavigateRoute.Goal(controls=[pose([1, 1, 2]), pose([0, 0, 2])])
    if lost_replies == 3:
        with pytest.raises(RuntimeError, match='discovery'): nav.execute_action(kind, goal, name)
        assert calls == [True, True, True]
    else:
        assert nav.execute_action(kind, goal, name)['success']
        assert calls == [True, True, False]
