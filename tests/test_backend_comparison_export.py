import json
from pathlib import Path
import pytest
from export_backend_comparison import summarize


def test_export_keeps_actual_rtk_batch_separate_from_global_topic_metrics(tmp_path):
    path=tmp_path/'report.json'
    batch={'completed':True,'quality':{'passed':True,'metrics':{'ate_rmse_m':.0138}},
        'pose_target':'GNSS antenna; separate from body control'}
    path.write_text(json.dumps({'backend':'fast_livo2_rtk','input_group':'livox_visual_inertial_rtk',
        'dataset_sha256':{'bag':'a'},'global_quality':None,'rtk_batch':batch}))
    row=summarize([path],{})['groups'][0]['cases'][0]['runs'][0]
    assert row['rtk_batch']==batch
    assert row['global_quality'] is None


def test_export_preserves_actual_delivery_and_rate_for_comparison(tmp_path):
    path=tmp_path/'report.json'
    path.write_text(json.dumps({'backend':'fast_livo2','input_group':'livox_visual_inertial',
        'dataset_sha256':{'bag':'a'},'playback_rate':.5,'requested_source_duration_s':200,
        'input_counts':{'/uav001/camera/image_raw':3000,'/uav001/camera/camera_info':6061}}))
    row=summarize([path],{})['groups'][0]['cases'][0]['runs'][0]
    assert row['playback_rate']==.5 and row['requested_source_duration_s']==200
    assert row['input_counts']['/uav001/camera/image_raw']==3000


def test_different_rates_and_source_windows_have_separate_comparison_cases(tmp_path):
    paths=[]
    for index,(rate,duration) in enumerate([(1.,200),(.5,200),(1.,100)]):
        path=tmp_path/(str(index)+'.json');paths.append(path)
        path.write_text(json.dumps({'backend':'fast_livo2','input_group':'livox_visual_inertial',
            'dataset_sha256':{'bag':'a'},'playback_rate':rate,'requested_source_duration_s':duration}))
    cases=summarize(paths,{})['groups'][0]['cases']
    assert len(cases)==3
    assert len({case['dataset_identity'] for case in cases})==1
    assert len({case['comparison_identity'] for case in cases})==3


def test_glim_export_labels_upstream_corrected_local_stream_accurately(tmp_path):
    path=tmp_path/'report.json'
    path.write_text(json.dumps({'backend':'glim','input_group':'sync','dataset_sha256':{'bag':'a'},
        'global_quality':{'metrics':{'ate_rmse_m':.05}},'capability_observations':{'global_pose_messages':12}}))
    row=summarize([path],{})['groups'][0]['cases'][0]['runs'][0]
    assert row['global_quality'] is None
    assert row['corrected_local_quality']['metrics']['ate_rmse_m']==.05


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
