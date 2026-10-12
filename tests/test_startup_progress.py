import json
from supervise import ManagedProcesses,wait_for


def test_readiness_progress_comes_from_real_checks(tmp_path):
    manager=ManagedProcesses(tmp_path)
    wait_for(manager,lambda:True,'validated continuous LIO',1.)
    path=tmp_path/'startup-progress.json'
    assert path.is_file(),'supervisor must expose actual readiness stage'
    value=json.loads(path.read_text())
    assert value['stage']=='validated continuous LIO'
    assert value['state']=='SUCCEEDED' and value['elapsed_s']>=0


def test_readiness_timeout_is_terminal_and_keeps_reason(tmp_path):
    import pytest
    manager=ManagedProcesses(tmp_path)
    with pytest.raises(RuntimeError,match='timeout'):
        wait_for(manager,lambda:False,'fresh sensor-time navigation map',.01)
    value=json.loads((tmp_path/'startup-progress.json').read_text())
    assert value['state']=='FAILED' and 'navigation' in value['reason']
