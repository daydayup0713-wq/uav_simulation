import os
import subprocess
import time
import socket
import pytest
from supervise import ManagedProcesses


def test_optional_observer_exit_does_not_stop_control_processes(tmp_path):
    from supervise import WEB_COMPONENTS
    manager=ManagedProcesses(tmp_path)
    try:
        manager.start('web-observatory',['/usr/bin/python3','-c','pass'],env=dict(os.environ))
        manager.start('bridge',['/usr/bin/python3','-c','import time;time.sleep(30)'],env=dict(os.environ))
        next(process for name,process in manager.processes if name=='web-observatory').wait(timeout=3)
        manager.check(ignore=WEB_COMPONENTS)
        bridge=next(process for name,process in manager.processes if name=='bridge')
        assert bridge.poll() is None
    finally:manager.close()


def test_web_port_preflight_rejects_listener_but_allows_recently_closed_transport():
    from supervise import check_web_port
    listener=socket.socket();listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    listener.bind(('127.0.0.1',0));port=listener.getsockname()[1];listener.listen()
    with pytest.raises(RuntimeError):check_web_port(port)
    client=socket.create_connection(('127.0.0.1',port));accepted,_=listener.accept()
    accepted.close();client.close();listener.close()
    check_web_port(port)
