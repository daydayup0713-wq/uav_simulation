import importlib.util,json
from pathlib import Path


def test_cpu_fixture_snapshot_is_built_from_observations_with_explicit_scope(tmp_path):
    assert importlib.util.find_spec('ci_workbench')
    import ci_workbench
    snapshot,manifest=ci_workbench.prepare_map(tmp_path)
    from uav_lab_navigation.planner_comparison import load_snapshot
    collision,evidence=load_snapshot(snapshot)
    assert collision.point_clear([0,0,2]) and not collision.point_clear([1.5,0,2])
    assert not collision.point_clear([20,20,2])
    assert evidence['configuration']['clearance_m']==.25
    assert 'analytical' in json.loads(manifest.read_text())['scope']
    assert snapshot.is_relative_to(manifest.parent)


def test_cpu_replay_uses_its_own_domain_without_changing_flight_environment():
    import ci_workbench
    assert hasattr(ci_workbench,'replay_environment')
    original={'LAB_DOMAIN_ID':'42','UNRELATED':'kept'}
    result=ci_workbench.replay_environment(original)
    assert result['LAB_DOMAIN_ID']=='77' and original['LAB_DOMAIN_ID']=='42'
    assert result['UNRELATED']=='kept'
