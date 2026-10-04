import json
import subprocess
from uav_lab_localization.cli import main


def test_backend_exit_is_a_structured_cli_failure(monkeypatch, capsys):
    def failed(*args): raise subprocess.CalledProcessError(-6, ['glim_rosbag'])
    monkeypatch.setattr('uav_lab_localization.benchmark.run_benchmark', failed)
    assert main(['benchmark','unused','--output','unused-output']) == 1
    output = json.loads(capsys.readouterr().out)
    assert output['success'] is False and 'SIGABRT' in output['reason']
