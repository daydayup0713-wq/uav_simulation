import time
from workbench_server import Workbench


def test_status_explains_protection_without_authorizing_arm(tmp_path,monkeypatch):
    app=Workbench(tmp_path,watch=False)
    monkeypatch.setattr(app,'active_run',lambda:(tmp_path/'run',{'supervisor_pid':1}))
    app.ingest({'kind':'telemetry','run_id':'run','diagnostics':{
        'uav001/flight':{'level':2,'message':'FAILSAFE','age_s':0.,'values':{
        'armed':'False','landed':'True','offboard':'False','reason':'external localization lost'}}}},time.monotonic())
    value=app.status()
    assert 'arm' not in value['allowed']
    assert '保护' in value.get('blocked',{}).get('arm','')
    assert value['components']['uav001/flight']['reason']=='external localization lost'
    app.close()


def test_status_explains_no_run_and_retains_legacy_fields(tmp_path):
    app=Workbench(tmp_path,watch=False)
    value=app.status()
    assert value.get('blocked',{}).get('arm')
    assert {'allowed','selection','connected','jobs','backends','mode','active_run'}<=value.keys()
    app.close()


def test_disabled_reasons_translate_authorization_without_changing_it():
    from workbench_server import DISABLED_REASONS
    assert '等待新鲜' in DISABLED_REASONS['fresh flight state required']
    assert '解锁' in DISABLED_REASONS['explicit armed Offboard state required']
