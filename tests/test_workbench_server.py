import time
from workbench_server import Workbench,validate_http_origin
import pytest
import json


def test_disconnected_status_is_valid_json_without_fabricated_freshness(tmp_path):
    app=Workbench(tmp_path,watch=False)
    result=json.loads(json.dumps(app.status(),allow_nan=False))
    assert result['flight']['received_at'] is None and not result['allowed'] and not result['connected']
    app.close()


def test_replay_selection_and_stale_state_cannot_submit_processes(tmp_path):
    app=Workbench(tmp_path,watch=False)
    with pytest.raises(ValueError,match='fresh'):app.request('arm',{})
    app.mode='replay'
    with pytest.raises(ValueError,match='replay'):app.request('arm',{})
    with pytest.raises(ValueError,match='replay'):app.request('select',{'localization':'glim','planning':'ego','scene':'helix','sensor_profile':None})
    assert not app.jobs
    app.close()


def test_ingested_stale_or_unknown_ground_flags_do_not_authorize_arm(tmp_path):
    app=Workbench(tmp_path,watch=False)
    app.ingest({'kind':'telemetry','diagnostics':{'uav001/flight':{'age_s':2,'message':'READY',
        'values':{'armed':'False','landed':'True','offboard':'False'}}}},time.monotonic())
    with pytest.raises(ValueError,match='fresh'):app.request('arm',{})
    assert not app.jobs
    app.close()


def test_fresh_telemetry_from_previous_run_cannot_authorize_new_run(tmp_path,monkeypatch):
    app=Workbench(tmp_path,watch=False)
    monkeypatch.setattr(app,'active_run',lambda:(tmp_path/'new-run',{'supervisor_pid':1}))
    message={'kind':'telemetry','run_id':'old-run','diagnostics':{'uav001/flight':{'age_s':0,'message':'READY',
        'values':{'armed':'False','landed':'True','offboard':'False'}}}}
    app.ingest(message,time.monotonic())
    assert app.status()['allowed']==[]
    assert not app.jobs
    message['run_id']='new-run';app.ingest(message,time.monotonic())
    assert 'arm' in app.status()['allowed']
    app.close()


def test_http_commands_require_same_loopback_origin_and_json():
    assert validate_http_origin('http://127.0.0.1:8780','application/json',200)
    for origin,kind,length in [('https://malicious.example','application/json',20),
                               (None,'application/json',20),('http://127.0.0.1:8780','text/plain',20),
                               ('http://localhost:8780','application/json',17000)]:
        with pytest.raises(ValueError):validate_http_origin(origin,kind,length)


def test_actual_http_replay_and_unknown_state_reject_commands_without_ros(tmp_path):
    import urllib.request,urllib.error,threading
    from workbench_server import Server
    app=Workbench(tmp_path,watch=False)
    server=Server(app,port=0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    address='http://127.0.0.1:'+str(server.server_address[1])
    try:
        result=json.load(urllib.request.urlopen(address+'/api/status'))
        assert result['allowed']==[] and result['flight']['received_at'] is None
        for command,origin in [('arm','http://127.0.0.1:8780'),('select','https://malicious.example')]:
            request=urllib.request.Request(address+'/api/'+command,data=b'{}',method='POST',
                headers={'Content-Type':'application/json','Origin':origin,'Host':'127.0.0.1:8780'})
            with pytest.raises(urllib.error.HTTPError) as error:urllib.request.urlopen(request)
            assert error.value.code==400 and json.load(error.value)['success'] is False
        assert not app.jobs and not app.processes
    finally:server.shutdown();server.server_close();thread.join(timeout=2);app.close()
