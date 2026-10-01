import json
import subprocess
from supervise import FlightReadiness, read_vehicle_status

def test_transient_valid_position_does_not_pass_flight_startup():
    check = FlightReadiness()
    state = dict(fresh='True', armed='False', landed='True', preflight='True')
    assert not check.update(state, 0)
    assert not check.update(state, 3)
    assert not check.update({**state, 'preflight': 'False'}, 4)
    assert not check.update(state, 5)
    assert not check.update(state, 9)
    assert check.update(state, 10)
    assert not check.update({}, 11)

def test_status_query_timeout_restarts_continuous_readiness_window(monkeypatch):
    check = FlightReadiness()
    state = dict(fresh='True', armed='False', landed='True', preflight='True')
    results = iter([subprocess.CompletedProcess([], 0, json.dumps(state)),
                    subprocess.TimeoutExpired('status', 5),
                    subprocess.CompletedProcess([], 0, json.dumps(state))])
    def query(*args, **kwargs):
        result = next(results)
        if isinstance(result, Exception):
            raise result
        return result
    monkeypatch.setattr(subprocess, 'run', query)
    assert not check.update(read_vehicle_status({}), 0)
    assert not check.update(read_vehicle_status({}), 5)
    assert not check.update(read_vehicle_status({}), 6)
    assert check.update(state, 11)
