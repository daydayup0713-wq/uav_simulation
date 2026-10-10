import importlib,importlib.util,json
import pytest


def test_native_pair_revalidation_rejects_changed_estimator_source_and_binary(tmp_path):
    mod=api()
    assert hasattr(mod,'identity_checks')
    catalog=tmp_path/'configs';catalog.mkdir()
    (catalog/'backends.json').write_text(json.dumps({'backends':[
        {'id':'glim','implementation_sha256':'a'*64},{'id':'astar','implementation_sha256':'b'*64}]}))
    run=tmp_path/'run';directory=run/'configuration/selected-localization';directory.mkdir(parents=True)
    source=directory/'provenance.json';source.write_text(json.dumps({'implementation_sha256':'a'*64}))
    binary=tmp_path/'native';binary.write_bytes(b'actual executable')
    from uav_lab_experiments.registry import digest
    (directory/'build-manifest.json').write_text(json.dumps({'runtime_artifacts':{str(binary):digest(binary)}}))
    selection={'localization':'glim','planning':'astar'}
    assert mod.identity_checks(tmp_path,run,selection)=={'glim':'a'*64,'astar':'b'*64}
    source.write_text(json.dumps({'implementation_sha256':'c'*64}))
    with pytest.raises(ValueError,match='implementation'):mod.identity_checks(tmp_path,run,selection)
    source.write_text(json.dumps({'implementation_sha256':'a'*64}));binary.write_bytes(b'changed executable')
    with pytest.raises(ValueError,match='artifact'):mod.identity_checks(tmp_path,run,selection)


def api():
    assert importlib.util.find_spec('qualify_pair'),'native pair report revalidation missing'
    return importlib.import_module('qualify_pair')


def test_hold_qualification_rechecks_the_original_observed_stop_map(tmp_path):
    import numpy as np
    from uav_lab_bridge.continuous_trajectory import State,Trajectory
    from uav_lab_experiments.registry import digest
    mod=api();assert hasattr(mod,'stop_checks')
    configuration=tmp_path/'configuration';configuration.mkdir()
    (configuration/'navigation.json').write_text(json.dumps({'body_halfsize_m':[.4,.4,.3],'clearance_m':.25}))
    curve=Trajectory.stop(State(np.array([0.,0.,2.]),np.array([.5,0.,0.])))
    path=tmp_path/'stop.npz';free=np.ones((50,50,50),dtype=bool)
    def save():np.savez(path,free=free,lower=[-2.5,-2.5,0.],resolution=.1,source_stamp=10.,envelope=[.65,.65,.55],map_version=1)
    save()
    event={'event':'stop_admission','sim_ns':10100000000,'trajectory':curve.to_dict(),
        'source_stamp':10.,'map_version':1,'map_artifact':'stop.npz','map_sha256':digest(path)}
    (tmp_path/'events.jsonl').write_text(json.dumps(event)+'\n')
    assert mod.stop_checks(tmp_path)==[path]
    free[27:,:,:]=False;save();event['map_sha256']=digest(path)
    (tmp_path/'events.jsonl').write_text(json.dumps(event)+'\n')
    with pytest.raises(ValueError,match='blocked or unknown'):mod.stop_checks(tmp_path)


def test_success_flags_without_independent_samples_and_owned_run_cannot_promote_pair(tmp_path):
    normal=tmp_path/'normal';normal.mkdir()
    (normal/'acceptance.json').write_text(json.dumps({'passed':True,'runs':3,'profile':'navigation','run_id':'invented',
        'backend_pair':{'localization':'glim','planning':'ego','scene':'circle-eight'}}))
    with pytest.raises(ValueError,match='evidence'):
        api().normal_checks(tmp_path,normal)


def test_pair_identifier_is_validated_before_any_evidence_read_or_publication():
    mod=api()
    assert hasattr(mod,'pair_identifier')
    for name in ('../../escape','', 'a'*61, 'bad.name'):
        with pytest.raises(ValueError):mod.pair_identifier(name)
    assert mod.pair_identifier('current-normal-01')=='current-normal-01'


def test_pair_publication_checks_both_previous_stages_before_writing(tmp_path):
    from uav_lab_experiments.registry import Registry
    mod=api()
    assert hasattr(mod,'publish_pair')
    source=tmp_path/'artifact';source.write_text('actual')
    catalog=tmp_path/'catalog.json';catalog.write_text(json.dumps({'schema':1,'groups':{'sync':{}},'backends':[
        {'id':name,'role':role,'source_ref':'a'*40,'groups':['sync']} for name,role in [('glim','localization'),('ego','planning')]]}))
    registry=Registry(catalog,tmp_path/'stages')
    report={'pair':{'localization':'glim','planning':'ego','input_group':'sync'},'artifacts':[]}
    with pytest.raises(ValueError,match='previous'):
        mod.publish_pair(registry,tmp_path/'pairs','native-01',report,[])
    assert not (tmp_path/'pairs').exists() and not (tmp_path/'stages').exists()
