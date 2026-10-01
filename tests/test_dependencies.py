import subprocess
import pytest
from bootstrap import checkout_repository, configure_agent_source

def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()

def cache(tmp_path):
    root = tmp_path / 'cache'
    root.mkdir()
    git(root, 'init', '-q')
    (root / 'model').write_text('clean model')
    git(root, 'add', '.')
    git(root, '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'initial')
    return root, git(root, 'rev-parse', 'HEAD')

def test_checkout_does_not_copy_dirty_cache(tmp_path):
    root, sha = cache(tmp_path)
    (root / 'model').write_text('local experimental changes')
    dest = tmp_path / 'dependency'
    checkout_repository({'url': str(root), 'local_cache': str(root), 'ref': sha}, dest)
    assert (dest / 'model').read_text() == 'clean model'
    assert (root / 'model').read_text() == 'local experimental changes'
    checkout_repository({'url': str(root), 'ref': sha}, dest)

def test_dirty_dependency_is_rejected(tmp_path):
    root, sha = cache(tmp_path)
    dest = tmp_path / 'dependency'
    checkout_repository({'url': str(root), 'ref': sha}, dest)
    (dest / 'model').write_text('changed')
    with pytest.raises(RuntimeError, match='dirty'):
        checkout_repository({'url': str(root), 'ref': sha}, dest)

def test_precreated_empty_dependency_directory_is_initialized(tmp_path):
    root, sha = cache(tmp_path)
    dest = tmp_path / 'empty'
    dest.mkdir()
    checkout_repository({'url': str(root), 'ref': sha}, dest)
    assert (dest / 'model').read_text() == 'clean model'

def test_agent_generated_source_pins_branches_without_mutating_checkout(tmp_path):
    source, dest = tmp_path/'source', tmp_path/'configured'
    source.mkdir()
    original = 'set(_fastdds_tag 2.14.x)\nset(UAGENT_P2P_CLIENT_TAG v2.4.3 CACHE STRING "tag")\n'
    (source/'CMakeLists.txt').write_text(original)
    (source/'cmake').mkdir()
    (source/'cmake/SuperBuild.cmake').write_text('find_package(fastcdr 2.2 EXACT QUIET)\nfind_package(foonathan_memory QUIET)\n')
    configure_agent_source(source, dest, {'_fastdds_tag': 'a'*40, 'UAGENT_P2P_CLIENT_TAG': 'b'*40})
    assert (source/'CMakeLists.txt').read_text() == original
    text = (dest/'CMakeLists.txt').read_text()
    assert '2.14.x' not in text and 'a'*40 in text and 'b'*40 in text
    superbuild = (dest/'cmake/SuperBuild.cmake').read_text()
    assert 'find_package(fastcdr' not in superbuild
    assert 'set(foonathan_memory_FOUND FALSE)' in superbuild
