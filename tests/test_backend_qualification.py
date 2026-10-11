import pytest
from uav_lab_experiments.qualification import qualification_checks


def passing(scope):
    return {'success':True,'quality':{'passed':True},'invalid_poses':[],
            'player_exit_code':0,'replay_completion':{'completed':True,'mode':'eof','expected_count':100},
            'normalized_output_count':100,'normalizer_events':[{'signature':['READY','']}],
            'scope':scope,'debug_core':False,'input_counts':{'/uav001/lidar/points':100,'/uav001/imu/data':2000}}


def test_failed_quality_or_debug_run_cannot_promote_replay_stage():
    report=passing('offline replay; no realtime or flight qualification')
    assert qualification_checks('replay_passed',report)['all_passed']
    report['quality']['passed']=False
    with pytest.raises(ValueError,match='quality'):qualification_checks('replay_passed',report)
    report['quality']['passed']=True;report['debug_core']=True
    with pytest.raises(ValueError,match='diagnostic'):qualification_checks('replay_passed',report)


def test_realtime_stage_requires_actual_live_stream_without_latched_failure():
    report=passing('offline replay; no realtime or flight qualification')
    with pytest.raises(ValueError,match='live'):qualification_checks('realtime_passed',report)
    report['scope']='real-time localization from measured Livox/IMU/camera; no GNSS or truth input.'
    assert qualification_checks('realtime_passed',report)['all_passed']
    report['normalizer_events'].append({'signature':['FAILED','source stale']})
    with pytest.raises(ValueError,match='watchdog'):qualification_checks('realtime_passed',report)


@pytest.mark.parametrize(('exit_code','complete'),[(1,True),(0,False),(None,True)])
def test_partial_or_failed_player_cannot_qualify_an_accurate_received_prefix(exit_code,complete):
    report=passing('offline replay; no realtime or flight qualification')
    report.update(player_exit_code=exit_code,replay_completion={
        'completed':complete,'mode':'eof','intentional_duration_stop':False,
        'expected_count':1000,'observed_count':1000 if complete else 10})
    with pytest.raises(ValueError,match='replay.*complet|player'):
        qualification_checks('replay_passed',report)
