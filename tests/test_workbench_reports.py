import hashlib,json
from workbench_reports import comparison


def test_reports_keep_input_and_dataset_groups_and_reject_changed_source(tmp_path):
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    reports=[]
    for name in ('a','b'):
        p=tmp_path/name;p.write_text('{"success":false}')
        reports.append({'backend':'glim','success':False,'source_report':str(p),
            'source_report_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'current_implementation':True,
            'quality':{'passed':False,'metrics':{'ate_rmse_m':.6}}})
    groups=[{'input_group':group,'cases':[{'dataset_identity':identity,'runs':[reports[i]]}]} for i,(group,identity) in enumerate([('sync','bag1'),('visual','bag2')])]
    (docs/'localization-comparison.json').write_text(json.dumps({'groups':groups}))
    result=comparison(tmp_path)
    assert [x['input_group'] for x in result['groups']]==['sync','visual']
    assert result['groups'][0]['cases'][0]['runs'][0]['source_verified']
    assert result['groups'][0]['cases'][0]['runs'][0]['success'] is False
    (tmp_path/'a').write_text('changed')
    assert not comparison(tmp_path)['groups'][0]['cases'][0]['runs'][0]['source_verified']


def test_missing_native_evidence_does_not_become_a_verified_success(tmp_path):
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    (docs/'localization-comparison.json').write_text(json.dumps({'groups':[{'input_group':'sync','cases':[{'dataset_identity':'fixed',
        'runs':[{'backend':'glim','success':True,'source_report':'/missing','source_report_sha256':'0'*64}]}]}]}))
    row=comparison(tmp_path)['groups'][0]['cases'][0]['runs'][0]
    assert row['success'] is True and not row['source_verified']


def test_metric_curves_match_source_times_and_never_read_arbitrary_paths(tmp_path):
    import pytest,workbench_reports
    run=tmp_path/'.runtime/flight';run.mkdir(parents=True)
    (run/'acceptance.json').write_text('{"passed":true}')
    (run/'tracking.samples.json').write_text(json.dumps({'references':[
        {'stamp':1.,'id':'a','position':[0,0,2],'velocity':[.5,0,0],'acceleration':[0,0,0]},
        {'stamp':1.1,'id':'a','position':[.05,0,2],'velocity':[.5,0,0],'acceleration':[0,0,0]}],
        'actual':[{'stamp':1.05,'position':[.125,0,2]},{'stamp':9.,'position':[10,0,2]}]}))
    assert hasattr(workbench_reports,'metrics')
    curve=workbench_reports.metrics(tmp_path,'flight')
    assert len(curve['points'])==1 and curve['points'][0]['error_m']==pytest.approx(.1)
    with pytest.raises(ValueError):workbench_reports.metrics(tmp_path,'../../etc')


def test_resource_phase_scope_and_posterior_remain_separate(tmp_path):
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    source=tmp_path/'report.json';source.write_text('{}')
    row={'backend':'fast_livo2_rtk','success':True,'source_report':str(source),
        'source_report_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'resource_summary':{'replay':{'mean_cpu_cores':1.2,'peak_combined_rss_bytes':1048576,'scope':'algorithm only'},
                            'posterior':{'mean_cpu_cores':2.,'peak_combined_rss_bytes':2097152,'scope':'post-process only'}},
        'quality':{'metrics':{'ate_rmse_m':.1}},'global_quality':{'metrics':{'ate_rmse_m':.02}},
        'capability_observations':{'loop_closure':'not observed','relocalization':'not verified'}}
    (docs/'localization-comparison.json').write_text(json.dumps({'groups':[{'input_group':'rtk','cases':[{'dataset_identity':'bag','runs':[row]}]}]}))
    result=comparison(tmp_path)['groups'][0]['cases'][0]['runs'][0]
    assert result['resource_measurements']==[{'phase':'replay','mean_cpu_cores':1.2,'peak_rss_mib':1.,'scope':'algorithm only'},
        {'phase':'posterior','mean_cpu_cores':2.,'peak_rss_mib':2.,'scope':'post-process only'}]
    assert result['quality']['metrics']['ate_rmse_m']==.1
    assert result['global_quality']['metrics']['ate_rmse_m']==.02


def test_flight_export_preserves_failed_native_result_and_source_changes(tmp_path):
    import workbench_reports
    runtime=tmp_path/'.runtime/native';runtime.mkdir(parents=True)
    acceptance={'passed':False,'reason':'curve unknown','runs':3,'run_id':'owned',
        'backend_pair':{'localization':'glim','planning':'ego','scene':'dense'}}
    (runtime/'acceptance.json').write_text(json.dumps(acceptance))
    (runtime/'tracking.json').write_text(json.dumps({'passed':True,'tracking_p95_m':.05}))
    assert hasattr(workbench_reports,'flight_comparison')
    report=workbench_reports.flight_comparison(tmp_path)
    row=report['rows'][0]
    assert row['passed'] is False and row['reason']=='curve unknown' and row['source_verified']
    assert row['tracking_p95_m']==.05
    assert row['qualified'] is False and row['planning']=='ego' and row['scene']=='dense'
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    (docs/'flight-comparison.json').write_text(json.dumps(report))
    (runtime/'acceptance.json').write_text('{}')
    assert not comparison(tmp_path)['closed_loop']['rows'][0]['source_verified']


def test_planning_success_rate_excludes_expected_rejection_cases(tmp_path):
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    (docs/'planner-comparison.json').write_text(json.dumps({'rows':[{'backend':'ego','cases':[
        {'purpose':'normal','success':False,'passed':False,'expected_success':True},
        {'purpose':'normal','success':True,'passed':True,'expected_success':True},
        {'purpose':'rejection','success':False,'passed':True,'expected_success':False}]}]}))
    row=comparison(tmp_path)['planning']['rows'][0]
    assert row['success_rate']==.5
    assert row['expected_outcome_rate']==2/3


def test_flight_derivatives_are_measured_analytic_maxima_not_a_handoff_count(tmp_path):
    import workbench_reports
    runtime=tmp_path/'.runtime/native';runtime.mkdir(parents=True)
    (runtime/'acceptance.json').write_text(json.dumps({'passed':True,'runs':3,'run_id':'owned',
        'backend_pair':{'localization':'glim','planning':'fast_planner'}}))
    (runtime/'geometry.json').write_text(json.dumps({'handoffs':138,'successful_runs':3,
        'accepted_curve_check':{'passed':True,'analytic_maxima':{'speed':.48,'acceleration':.36,'jerk':.62}}}))
    row=workbench_reports.flight_comparison(tmp_path)['rows'][0]
    assert row['analytic_maxima']=={'speed':.48,'acceleration':.36,'jerk':.62}
    assert row['successful_runs']==3 and row['handoffs']==138


def test_glim_corrected_local_pose_metric_is_not_reported_as_global_optimization(tmp_path):
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    source=tmp_path/'report.json';source.write_text('{}')
    run={'backend':'glim','success':True,'source_report':str(source),
        'source_report_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'global_quality':{'metrics':{'ate_rmse_m':.05}},'capability_observations':{'global_pose_messages':100}}
    (docs/'localization-comparison.json').write_text(json.dumps({'groups':[{'input_group':'sync','cases':[{'runs':[run]}]}]}))
    row=comparison(tmp_path)['groups'][0]['cases'][0]['runs'][0]
    assert row['global_quality'] is None
    assert row['corrected_local_quality']['metrics']['ate_rmse_m']==.05
    assert row['capability_observations']['corrected_local_pose_messages']==100


def test_comparison_recomputes_source_identity_after_catalog_change(tmp_path):
    docs=tmp_path/'docs/validation/l1';docs.mkdir(parents=True)
    source=tmp_path/'report.json';source.write_text('{}')
    row={'backend':'glim','implementation_sha256':'a'*64,'current_implementation':True,
        'source_report':str(source),'source_report_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    (docs/'localization-comparison.json').write_text(json.dumps({'groups':[{'input_group':'sync','cases':[{'runs':[row]}]}]}))
    config=tmp_path/'configs';config.mkdir();catalog=config/'backends.json'
    catalog.write_text(json.dumps({'backends':[{'id':'glim','implementation_sha256':'b'*64}]}))
    result=comparison(tmp_path)['groups'][0]['cases'][0]['runs'][0]
    assert result['current_implementation'] is False and result['source_verified'] is True
