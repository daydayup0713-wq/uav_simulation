import subprocess
import pytest
from bootstrap import checkout_repository

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
