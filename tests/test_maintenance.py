import json
from pathlib import Path
import pytest

NOW=1791800000.


def project(tmp_path):
    (tmp_path/'dependencies').mkdir()
    (tmp_path/'dependencies/lock.json').write_text('{}')
    (tmp_path/'README.md').write_text('UAV Simulation Lab')
    (tmp_path/'.runtime').mkdir()
    return tmp_path


def run(root,index):
    path=root/'.runtime'/f'202601{index:02d}T000000Z-abcdef01'
    path.mkdir()
    (path/'manifest.json').write_text(json.dumps({'run_id':path.name,'supervisor_pid':9999999,
        'profile':'navigation','scene':'circle-eight','backend_selection':{'localization':'glim','planning':'fast_planner','localization_input_group':'generic'}}))
    (path/'closed').write_text('closed')
    (path/'bridge.log').write_text('FAILSAFE: controlled failure')
    (path/'data.bin').write_bytes(b'x'*1024)
    return path


def test_retention_keeps_references_current_and_last_three_failures(tmp_path):
    from maintenance import plan
    root=project(tmp_path);items=[run(root,i) for i in range(1,7)]
    (root/'.runtime/current-run').write_text(str(items[0]))
    doc=root/'docs/validation';doc.mkdir(parents=True)
    (doc/'report.json').write_text(json.dumps({'artifacts':[{'path':str(items[1]/'data.bin')}]}))
    result=plan(root,NOW)
    assert {row['path'] for row in result['candidates']}=={str(items[2].relative_to(root))}
    assert any('reference' in row['reason'] for row in result['protected'])


def test_apply_refuses_changed_files_and_new_references(tmp_path):
    from maintenance import plan,apply
    root=project(tmp_path);items=[run(root,i) for i in range(1,6)]
    proposal=plan(root,NOW)
    (items[0]/'data.bin').write_bytes(b'changed')
    p=root/'configs';p.mkdir();(p/'preserve.json').write_text(json.dumps({'path':str(items[1]/'data.bin')}))
    result=apply(root,proposal,NOW)
    assert not result['success'] and items[0].exists() and items[1].exists()
    assert len(result['refused'])==2


def test_symlinks_and_unknown_runtime_stay_protected(tmp_path):
    from maintenance import plan,apply
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    external=tmp_path/'outside';external.mkdir();(external/'keep').write_text('keep')
    (items[0]/'link').symlink_to(external,target_is_directory=True)
    unknown=root/'.runtime/unknown';unknown.mkdir()
    proposal=plan(root,NOW)
    assert not proposal['candidates']
    forged={**proposal,'candidates':[{'path':'../outside','identity':[]} ]}
    assert not apply(root,forged,NOW)['success']
    assert (external/'keep').read_text()=='keep' and unknown.exists()


def test_partial_removal_failure_is_reported_and_other_items_continue(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);items=[run(root,i) for i in range(1,6)]
    proposal=maintenance.plan(root,NOW);original=maintenance.shutil.rmtree
    def remove(path):
        if Path(path).name==items[0].name:
            (Path(path)/'data.bin').unlink()
            raise OSError('controlled partial filesystem failure')
        original(path)
    remove.avoids_symlink_attacks=original.avoids_symlink_attacks
    monkeypatch.setattr(maintenance.shutil,'rmtree',remove)
    result=maintenance.apply(root,proposal,NOW)
    assert not result['success'] and items[0].exists() and not items[1].exists()
    assert not (items[0]/'data.bin').exists() and (items[0]/'manifest.json').exists()
    assert result['removed'] and result['errors']


def test_events_alone_are_not_a_success_and_real_success_keeps_three(tmp_path):
    from maintenance import plan
    root=project(tmp_path);items=[run(root,i) for i in range(1,6)]
    for item in items:
        (item/'bridge.log').write_text('READY')
        (item/'events.jsonl').write_text('{}\n')
    assert not plan(root,NOW)['candidates']
    for item in items:(item/'acceptance.json').write_text('{"success":true}')
    assert len(plan(root,NOW)['candidates'])==2


def test_only_old_whitelisted_caches_are_candidates_and_active_run_blocks(tmp_path):
    import os
    from maintenance import plan
    root=project(tmp_path)
    cache=root/'.pytest_cache';cache.mkdir();(cache/'cache').write_text('rebuildable')
    os.utime(cache/'cache',(NOW-8*86400,NOW-8*86400));os.utime(cache,(NOW-8*86400,NOW-8*86400))
    assert any(row['path']=='.pytest_cache' for row in plan(root,NOW)['candidates'])
    current=run(root,1);(current/'ready').touch()
    assert not any(row['path']=='.pytest_cache' for row in plan(root,NOW)['candidates'])


def test_symlink_reference_and_lexical_parent_reference_protect_runs(tmp_path):
    from maintenance import plan
    root=project(tmp_path);items=[run(root,i) for i in range(1,6)]
    docs=root/'docs';docs.mkdir();(docs/'evidence.json').symlink_to(items[0]/'manifest.json')
    configs=root/'configs';configs.mkdir();(configs/'reference.json').write_text(json.dumps({'path':str(items[1]/'..'/items[1].name/'data.bin')}))
    assert not plan(root,NOW)['candidates']


def test_apply_cli_always_saves_audit(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);proposal=maintenance.plan(root,NOW)
    file=root/'plan.json';file.write_text(json.dumps(proposal))
    monkeypatch.setattr('sys.argv',['maintenance','apply','--root',str(root),'--plan',str(file)])
    assert maintenance.main()==0
    audit=list((root/'.runtime/maintenance').glob('*.json'))
    assert len(audit)==1 and json.loads(audit[0].read_text())['result']['removed']==[]


def test_running_process_identity_protects_closed_run(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    original=maintenance.active
    monkeypatch.setattr(maintenance,'active',lambda path,project:Path(path)==items[0] or original(path,project))
    assert not maintenance.plan(root,NOW)['candidates']


def test_only_owned_closed_source_checkout_can_be_removed(tmp_path):
    import os
    from maintenance import plan,apply
    root=project(tmp_path);temporary=root/'.runtime/policy-clean-checkout-20260101';temporary.mkdir()
    (temporary/'source.py').write_text('source')
    metadata=temporary/'maintenance-disposable.json'
    metadata.write_text(json.dumps({'schema':1,'kind':'source-checkout','root':str(root),'closed':True}))
    for p in [temporary,*temporary.iterdir()]:os.utime(p,(NOW-8*86400,NOW-8*86400))
    proposal=plan(root,NOW)
    assert any(item['path']==str(temporary.relative_to(root)) for item in proposal['candidates'])
    assert apply(root,proposal,NOW)['success'] and not temporary.exists()
    unknown=root/'.runtime/policy-clean-checkout-20260102';unknown.mkdir()
    assert not plan(root,NOW)['candidates']


def test_parent_directory_swap_never_follows_external_symlink(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    proposal=maintenance.plan(root,NOW)
    external=tmp_path/'external';external.mkdir();foreign=external/items[0].name;foreign.mkdir();(foreign/'keep').write_text('keep')
    original=maintenance.shutil.rmtree
    def swapped_remove(path):
        (root/'.runtime').rename(root/'.runtime-old')
        (root/'.runtime').symlink_to(external,target_is_directory=True)
        original(path)
    swapped_remove.avoids_symlink_attacks=original.avoids_symlink_attacks
    monkeypatch.setattr(maintenance.shutil,'rmtree',swapped_remove)
    maintenance.apply(root,proposal,NOW)
    assert (foreign/'keep').read_text()=='keep'


def test_native_passed_acceptance_field_retains_recent_three(tmp_path):
    from maintenance import plan
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    for item in items:
        (item/'bridge.log').write_text('READY')
        (item/'acceptance.json').write_text('{"passed":true}')
    assert len(plan(root,NOW)['candidates'])==1

@pytest.mark.parametrize('link_directory',[False,True])
def test_retained_report_container_transitively_protects_raw_run(tmp_path,link_directory):
    from maintenance import plan
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    reports=root/'.runtime/flight-acceptance';reports.mkdir()
    nested=reports/'nested';nested.mkdir()
    (nested/'acceptance.json').write_text(json.dumps({'run_id':items[0].name}))
    if link_directory:
        docs=root/'docs';docs.mkdir()
        (docs/'index.md').write_text('[acceptance](.runtime/flight-acceptance)')
    assert not plan(root,NOW)['candidates']


def test_recent_retained_run_protects_older_origin(tmp_path):
    from maintenance import plan
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    (items[-1]/'origin.json').write_text(json.dumps({'run_id':items[0].name}))
    assert not plan(root,NOW)['candidates']


def test_interrupted_removal_has_durable_intent_and_interrupted_audit(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    proposal=maintenance.plan(root,NOW);original=maintenance.remove_owned
    def interrupt(*args):
        audit=list((root/'.runtime/maintenance').glob('*.json'))
        assert audit and json.loads(audit[0].read_text())['pending']==str(items[0].relative_to(root))
        original(*args)
        raise KeyboardInterrupt()
    monkeypatch.setattr(maintenance,'remove_owned',interrupt)
    with pytest.raises(KeyboardInterrupt):maintenance.apply(root,proposal,NOW)
    assert not items[0].exists()
    record=json.loads(next((root/'.runtime/maintenance').glob('*.json')).read_text())
    assert record['state']=='interrupted' and record['pending']
    assert record['result']['success'] is False


def test_audit_storage_failure_prevents_any_deletion(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);items=[run(root,i) for i in range(1,5)]
    proposal=maintenance.plan(root,NOW)
    monkeypatch.setattr(maintenance.os,'fsync',lambda _:(_ for _ in ()).throw(OSError('disk unavailable')))
    with pytest.raises(OSError):maintenance.apply(root,proposal,NOW)
    assert all(item.exists() for item in items)


def test_intermittent_audit_write_failure_stops_remaining_candidates(tmp_path,monkeypatch):
    import maintenance
    root=project(tmp_path);items=[run(root,i) for i in range(1,7)]
    proposal=maintenance.plan(root,NOW);original=maintenance.os.fsync;calls=[0]
    def fail_intent(fd):
        calls[0]+=1
        if calls[0]==3:raise OSError('intent fsync failed')
        original(fd)
    monkeypatch.setattr(maintenance.os,'fsync',fail_intent)
    with pytest.raises(OSError):maintenance.apply(root,proposal,NOW)
    assert all(item.exists() for item in items)
