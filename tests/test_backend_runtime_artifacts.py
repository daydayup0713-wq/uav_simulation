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


def test_live_preflight_verifies_linked_libraries_as_well_as_the_executable(tmp_path):
    from backend_provenance import verified_private_build
    binary=tmp_path/'node';binary.write_bytes(b'fixed executable')
    library=tmp_path/'core.so';library.write_bytes(b'fixed core')
    manifest={'binary':str(binary),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
              'runtime_artifacts':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary,library)}}
    (tmp_path/'fast_livo2-build-manifest.json').write_text(json.dumps(manifest))
    assert verified_private_build(tmp_path,'fast_livo2')['binary']==str(binary)
    library.write_bytes(b'replaced core')
    with pytest.raises(ValueError,match='runtime artifact identity'):
        verified_private_build(tmp_path,'fast_livo2')
