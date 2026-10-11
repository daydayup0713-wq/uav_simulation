"""Experimental closed-loop launch is explicit and source qualification fails closed."""
import importlib,importlib.util,json
from pathlib import Path
import pytest
from uav_lab_experiments.registry import Registry,digest


def api():
    assert importlib.util.find_spec('experiment_configuration'),'experiment startup policy missing'
    return importlib.import_module('experiment_configuration')


def registry(tmp_path,local_stage='replay_passed',planner_stage='replay_passed'):
    path=tmp_path/'catalog.json';proof=tmp_path/'proof';proof.write_text('unit interface evidence')
    path.write_text(json.dumps({'schema':1,'groups':{'livox':[],'synchronous':[]},'backends':[
        {'id':'lio','role':'localization','source_ref':'a'*40,'groups':['livox','synchronous']},
        {'id':'ego','role':'planning','source_ref':'b'*40,'groups':['livox','synchronous']}]}))
    value=Registry(path,tmp_path/'evidence')
    for key,last,group in [('lio',local_stage,'livox'),('ego',planner_stage,'synchronous')]:
        stages=['installed','replay_passed','realtime_passed','closed_loop_qualified']
        for stage in stages[:stages.index(last)+1]:
            value.record({'schema':1,'backend':key,'source_ref':value.backends[key]['source_ref'],
                'input_group':group,'stage':stage,'run_id':key+'-'+stage,'success':True,'checks':{'all_passed':True},
                'artifacts':[{'path':str(proof),'sha256':digest(proof)}]})
    return value


def test_replay_pair_is_allowed_only_for_explicit_supervised_qualification(tmp_path):
    value=registry(tmp_path)
    with pytest.raises(ValueError,match='closed_loop'):
        api().validate_pair(value,'lio','ego','livox',qualification=False)
    result=api().validate_pair(value,'lio','ego','livox',qualification=True)
    assert result['qualification_run'] and result['localization_stage']=='replay_passed'
    assert result['planning_evidence_group']=='synchronous'


def test_failed_replay_cannot_be_bypassed_by_installation_or_different_localization_inputs(tmp_path):
    value=registry(tmp_path,local_stage='installed')
    with pytest.raises(ValueError,match='replay_passed'):
        api().validate_pair(value,'lio','ego','livox',qualification=True)
    with pytest.raises(ValueError,match='replay_passed'):
        api().validate_pair(value,'lio','ego','synchronous',qualification=True)


def test_role_or_sensor_contract_mismatch_is_rejected(tmp_path):
    value=registry(tmp_path)
    with pytest.raises(ValueError,match='role'):
        api().validate_pair(value,'ego','lio','livox',qualification=True)
    with pytest.raises(ValueError,match='group'):
        api().validate_pair(value,'lio','ego','unknown',qualification=True)
