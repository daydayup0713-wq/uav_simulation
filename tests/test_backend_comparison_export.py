import json
from pathlib import Path
import pytest
from export_backend_comparison import summarize


def report(directory,backend,group,passed,posterior=False):
    directory.mkdir()
    data={'backend':backend,'input_group':group,'success':passed,'implementation_sha256':'old',
          'dataset_sha256':{'bag.db3':'a'*64},'quality':{'passed':passed,'metrics':{'ate_rmse_m':1.}},
          'scope':'offline replay; no flight qualification',
          'pose_graph':{'quality':{'passed':True},'scope':'keyframes only'} if posterior else None}
    path=directory/'report.json';path.write_text(json.dumps(data));return path


def test_export_groups_by_consumed_modalities_and_keeps_failed_frontend_separate(tmp_path):
    lidar=report(tmp_path/'lio','fast_lio2','livox_inertial',True)
    visual=report(tmp_path/'vins','vins_fusion','monocular_inertial',False,posterior=True)
    result=summarize([lidar,visual],{'fast_lio2':'new','vins_fusion':'new'})
    assert {g['input_group'] for g in result['groups']}=={'livox_inertial','monocular_inertial'}
    entry=next(g for g in result['groups'] if g['input_group']=='monocular_inertial')['cases'][0]['runs'][0]
    assert entry['success'] is False
    assert entry['pose_graph']['quality']['passed'] is True
    assert entry['current_implementation'] is False
    assert len(entry['source_report_sha256'])==64
    assert result['qualification']=='none; consult the artifact-backed backend registry'


def test_export_rejects_missing_group_and_duplicate_artifact(tmp_path):
    path=report(tmp_path/'bad','fast_lio2',None,False)
    with pytest.raises(ValueError,match='input group'):summarize([path],{})
    data=json.loads(path.read_text());data['input_group']='livox_inertial';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='duplicate'):summarize([path,path],{})
