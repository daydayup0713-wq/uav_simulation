import hashlib
import json
import pytest


def test_qualification_requires_a_runtime_artifact_inventory():
    from backend_provenance import verify_runtime_artifacts
    with pytest.raises(ValueError,match='runtime artifact inventory'):
        verify_runtime_artifacts({})


def test_changed_linked_core_or_vocabulary_invalidates_build(tmp_path):
    from backend_provenance import verify_runtime_artifacts
    core = tmp_path/'libcore.so';core.write_bytes(b'compiled core')
    vocabulary = tmp_path/'vocabulary.txt';vocabulary.write_bytes(b'feature vocabulary')
    manifest = {'runtime_artifacts':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in (core,vocabulary)}}
    verify_runtime_artifacts(manifest)
    core.write_bytes(b'different core')
    with pytest.raises(ValueError,match='runtime artifact identity'):
        verify_runtime_artifacts(manifest)


def test_live_preflight_verifies_linked_libraries_as_well_as_the_executable(tmp_path,monkeypatch):
    from backend_provenance import verified_private_build
    import backend_provenance
    monkeypatch.setattr(backend_provenance,'ROOT',tmp_path)
    interface=tmp_path/'localization/interfaces/livox_ros_driver2';interface.mkdir(parents=True)
    (interface/'CustomMsg.msg').write_text('source interface')
    binary=tmp_path/'node';binary.write_bytes(b'fixed executable')
    library=tmp_path/'core.so';library.write_bytes(b'fixed core')
    manifest={'binary':str(binary),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
              'runtime_artifacts':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary,library)},
              'compiled_platform_inputs':backend_provenance.compiled_platform_inputs(tmp_path,'fast_livo2')}
    (tmp_path/'fast_livo2-build-manifest.json').write_text(json.dumps(manifest))
    assert verified_private_build(tmp_path,'fast_livo2')['binary']==str(binary)
    library.write_bytes(b'replaced core')
    with pytest.raises(ValueError,match='runtime artifact identity'):
        verified_private_build(tmp_path,'fast_livo2')


def test_build_manifest_covers_every_locked_input_origin_including_copied_message_repo(tmp_path,monkeypatch):
    import bootstrap_backends as module
    lock={'repositories':{name:{'ref':'a'*40,'tree_sha256':'verified','patches':[]} for name in ('fast_livo2','livox-driver2')},
        'backends':{'fast_livo2':{'repositories':['fast_livo2','livox-driver2']}}}
    directory=tmp_path/'dependencies';directory.mkdir();(directory/'backends.lock.json').write_text(json.dumps(lock))
    for name in lock['repositories']:(tmp_path/'.deps'/name).mkdir(parents=True)
    prefix=tmp_path/'install';prefix.mkdir();binary=prefix/'node';binary.write_bytes(b'compiled')
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(module,'source_tree',lambda p:'verified')
    def output(args,**kwargs):
        if args[0]=='ldd':return ''
        if 'rev-parse' in args:return 'a'*40+'\n'
        return b''
    monkeypatch.setattr(module.subprocess,'check_output',output)
    interface=tmp_path/'localization/interfaces/livox_ros_driver2';interface.mkdir(parents=True)
    (interface/'CustomMsg.msg').write_text('source interface')
    from backend_provenance import compiled_platform_inputs
    module.manifest('fast_livo2',prefix,binary,['fast_livo2'],compiled_inputs=compiled_platform_inputs(tmp_path,'fast_livo2'))
    result=json.loads((prefix/'fast_livo2-build-manifest.json').read_text())
    assert set(result['repositories'])=={'fast_livo2','livox-driver2'}


def test_changed_compiled_platform_adapter_requires_a_new_private_build(tmp_path,monkeypatch):
    import backend_provenance as module
    monkeypatch.setattr(module,'ROOT',tmp_path,raising=False)
    adapter=tmp_path/'localization/visual_adapters/adapter.cpp';adapter.parent.mkdir(parents=True)
    adapter.write_bytes(b'original compiled adapter')
    binary=tmp_path/'install/node';binary.parent.mkdir();binary.write_bytes(b'unchanged executable')
    manifest={'binary':str(binary),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'runtime_artifacts':{str(binary):hashlib.sha256(binary.read_bytes()).hexdigest()},
        'compiled_platform_inputs':{'localization/visual_adapters/adapter.cpp':hashlib.sha256(adapter.read_bytes()).hexdigest()}}
    (binary.parent/'orb_slam3-build-manifest.json').write_text(json.dumps(manifest))
    assert module.verified_private_build(binary.parent,'orb_slam3')['binary']==str(binary)
    adapter.write_bytes(b'changed adapter without rebuilding')
    with pytest.raises(ValueError,match='compiled platform input'):
        module.verified_private_build(binary.parent,'orb_slam3')
