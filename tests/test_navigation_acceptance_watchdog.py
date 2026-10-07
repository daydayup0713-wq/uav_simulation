import subprocess
import sys
from types import SimpleNamespace
from accept_navigation import main


def test_expired_demo_requests_land_and_preserves_failed_evidence(monkeypatch,tmp_path):
    monkeypatch.setenv('LAB_ROOT',str(tmp_path));monkeypatch.setattr(sys,'argv',['accept_navigation','--output',str(tmp_path/'result')])
    calls=[]
    class Observer:
        returncode=0
        def send_signal(self,*args):pass
        def wait(self,**kw):return 0
    monkeypatch.setattr(subprocess,'Popen',lambda *args,**kw:Observer())
    def command(cmd,**kwargs):
        calls.append(cmd)
        if 'demo' in cmd:raise subprocess.TimeoutExpired(cmd,320)
        return SimpleNamespace(returncode=0,stdout='landed',stderr='')
    monkeypatch.setattr(subprocess,'run',command)
    assert main()==1
    assert any(cmd[-1]=='land' for cmd in calls)
    assert (tmp_path/'result/acceptance.json').exists()
