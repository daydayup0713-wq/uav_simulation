from supervise import source_identity

def test_container_source_without_git_uses_supplied_revision(tmp_path):
    assert source_identity(tmp_path, {'LAB_REVISION': 'a'*40}) == ('a'*40, None)

def test_unversioned_source_archive_is_not_reported_as_clean_git(tmp_path):
    assert source_identity(tmp_path, {}) == ('source-archive', None)
