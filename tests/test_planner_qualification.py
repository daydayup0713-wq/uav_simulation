"""Planner stage promotion requires the independently rechecked native evidence."""
import importlib
import importlib.util
from pathlib import Path
import json
import shutil
import pytest


def api():
    assert importlib.util.find_spec('uav_lab_navigation.planner_qualification'),'planner qualification missing'
    return importlib.import_module('uav_lab_navigation.planner_qualification')


def native_report(root):
    pointer=root/'.runtime/planner-comparison-current'
    if not pointer.exists():pytest.skip('native fixed-map report absent')
    return Path(pointer.read_text().strip())


def test_planner_replay_stage_rejects_summary_without_curve_artifacts(tmp_path):
    (tmp_path/'report.json').write_text(json.dumps({'passed':True,'rows':[]}))
    with pytest.raises(ValueError,match='evidence'):
        api().replay_checks(Path('.').resolve(),'ego',tmp_path)


def test_planner_replay_stage_is_rechecked_against_real_map_and_initial_derivatives():
    root=Path('.').resolve();directory=native_report(root)
    for backend in ('astar','ego','fast_planner','gcopter'):
        result=api().replay_checks(root,backend,directory)
        assert result['all_passed'] and result['full_curve_rechecked']
        assert result['flight_qualified'] is False


def test_claimed_curve_must_match_actual_upstream_position_solution(tmp_path):
    root=Path('.').resolve();source=native_report(root)
    directory=tmp_path/'copy';shutil.copytree(source,directory)
    for path in (directory/'ego').glob('*/pipeline-report.json'):
        data=json.loads(path.read_text())
        if data.get('source_result'):
            old=Path(data['source_result']);new=directory/old.relative_to(source)
            data['source_result']=str(new);path.write_text(json.dumps(data))
    path=directory/'ego/normal/pipeline-report.json';data=json.loads(path.read_text())
    raw=Path(data['source_result']);result=json.loads(raw.read_text())
    result['trajectory']['segments'][0]['coefficients'][0][0]+=.1
    raw.write_text(json.dumps(result))
    with pytest.raises(ValueError,match='upstream|solution'):
        api().replay_checks(root,'ego',directory)
