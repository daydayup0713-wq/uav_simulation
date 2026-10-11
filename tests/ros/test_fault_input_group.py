import json
import pytest
pytest.importorskip('px4_msgs.msg')


def test_fault_suite_cannot_qualify_a_pair_measured_with_a_different_sensor_group(tmp_path,monkeypatch):
    import qualify_pair as module
    from fault_scenario import CASES
    from px4_msgs.msg import VehicleStatus
    monkeypatch.setattr(module,'identity_checks',lambda *args:{'glim':'a'*64,'astar':'b'*64})
    monkeypatch.setattr(module,'matches',lambda *args:True)
    monkeypatch.setattr(module,'stop_checks',lambda *args:[])  # Independent stop geometry is tested separately.
    directory=tmp_path/'faults'
    manifests=[]
    for case in CASES:
        run=tmp_path/'.runtime'/case;run.mkdir(parents=True)
        manifest={'backend_selection':{'localization':'glim','planning':'astar'},'control_implementation':{},
                  'calibration':{'lidar':{'kind':'generic'}}}
        path=run/'manifest.json';path.write_text(json.dumps(manifest));manifests.append(path)
        (run/'web-failure.json').write_text('{}')
        report={'case':case,'run_id':case,'passed':True,'final':{'armed':False,'landed':True},
            'motion_returncode':0 if case=='web_disconnect' else 1,
            'events':[{'armed':True,'nav_state':VehicleStatus.NAVIGATION_STATE_AUTO_LAND}],
            'commands':[{'arguments':['arm'],'returncode':1}],
            'references':[{'velocity':[0,0,0]}],'source_process_resumed':True,'browser':{'client_closed':True}}
        target=directory/case;target.mkdir(parents=True);(target/'fault.json').write_text(json.dumps(report))
    pair={'localization':'glim','planning':'astar','input_group':'synchronous_lidar_imu'}
    assert len(module.fault_checks(tmp_path,[directory],pair))==5
    changed=json.loads(manifests[0].read_text());changed['calibration']['lidar']['kind']='livox'
    manifests[0].write_text(json.dumps(changed))
    with pytest.raises(ValueError,match='input group'):
        module.fault_checks(tmp_path,[directory],pair)
