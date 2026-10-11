"""Registration must qualify the executable actually used for measurement."""
import json
import sys
import pytest
from uav_lab_experiments.registry import digest


def test_same_sources_rebuilt_binary_cannot_reuse_old_measurements(tmp_path,monkeypatch):
    import register_backend as module
    import backend_provenance
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(module,'ADAPTER_FILES',[])
    monkeypatch.setattr(module,'verify_sources',lambda *args:None)  # Upstream checkout validation has separate tests.
    lock={'repositories':{},'backends':{'glim':{'repositories':[]}}}
    dependencies=tmp_path/'dependencies';dependencies.mkdir()
    (dependencies/'backends.lock.json').write_text(json.dumps(lock))
    fp=backend_provenance.implementation_fingerprint(tmp_path,lock,'glim',files=[])
    monkeypatch.setattr(module,'implementation_fingerprint',lambda *args:fp)
    config=tmp_path/'configs';config.mkdir()
    (config/'backends.json').write_text(json.dumps({'schema':1,'groups':{'synchronous_lidar_imu':{}},
        'backends':[{'id':'glim','role':'localization','source_ref':'a'*40,
                    'groups':['synchronous_lidar_imu'],'implementation_sha256':fp}]}))
    prefix=tmp_path/'.deps/backends/glim/install';prefix.mkdir(parents=True)
    old=prefix/'measured_node';old.write_bytes(b'first build used for replay')
    new=prefix/'rebuilt_node';new.write_bytes(b'new build with unchanged sources')
    def build(binary):return {'binary':str(binary),'binary_sha256':digest(binary),
        'runtime_artifacts':{str(binary):digest(binary)},'repositories':{},'compiled_platform_inputs':{}}
    (prefix/'glim-build-manifest.json').write_text(json.dumps(build(new)))
    base=['register_backend.py','--backend','glim','--group','synchronous_lidar_imu','--run-id','installed-current']
    monkeypatch.setattr(sys,'argv',base+['--stage','installed']);module.main()
    replay=tmp_path/'measured';(replay/'configuration').mkdir(parents=True)
    for name in ('estimate.tum','truth.tum','provenance.json','configuration/parameters.yaml','configuration/camera.yaml'):
        (replay/name).write_text('retained measured artifact')
    (replay/'calibration.json').write_text(json.dumps({'lidar':{'kind':'generic'}}))
    measured=build(old);(replay/'build-manifest.json').write_text(json.dumps(measured))
    report={'success':True,'quality':{'passed':True},'scope':'offline replay; no realtime or flight qualification',
        'backend':'glim','implementation_sha256':fp,'build':measured,'player_exit_code':0,
        'replay_completion':{'completed':True,'mode':'eof','intentional_duration_stop':False,'expected_count':100,'observed_count':100}}
    (replay/'report.json').write_text(json.dumps(report))
    monkeypatch.setattr(sys,'argv',base[:-1]+['replay-current','--stage','replay_passed','--report',str(replay/'report.json')])
    with pytest.raises(ValueError,match='measured build'):
        module.main()
    assert not (tmp_path/'.runtime/backend-evidence/glim/replay-current.json').exists()
    (replay/'build-manifest.json').write_text(json.dumps(build(new)))
    report['build']=build(new);(replay/'report.json').write_text(json.dumps(report))
    (replay/'recorded-source-window.json').write_text(json.dumps({'expected_stamps':[1.,2.,3.]}))
    module.main()
    evidence=json.loads((tmp_path/'.runtime/backend-evidence/glim/replay-current.json').read_text())
    assert any(row['path']==str(replay/'build-manifest.json') and row['sha256']==digest(replay/'build-manifest.json')
               for row in evidence['artifacts'])
