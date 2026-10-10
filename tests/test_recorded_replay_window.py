from types import SimpleNamespace as NS
import pytest
from replay_completion import recorded_window,replay_completion


def sensor_window(duration=0.):
    messages=[]
    for second in range(10,21):
        messages.extend([('/clock',NS(clock=NS(sec=second,nanosec=0))),
                         ('/uav001/lidar/points',NS(header=NS(stamp=NS(sec=second,nanosec=0))))])
    return recorded_window(messages,'fast_livo2',{'scan_end_offset_s':.1},duration)


def test_recording_supplies_full_denominator_even_when_observer_received_only_a_prefix():
    window=sensor_window()
    assert window['quality_expected_stamps']==[13.1,14.1,15.1,16.1,17.1,18.1,19.1,20.1]
    completion=replay_completion(window,[10.1,11.1,12.1],0)
    assert not completion['completed'] and completion['expected_count']==11


@pytest.mark.parametrize(('exit_code','intentional','wanted'),[(0,False,True),(1,False,False),
    (130,False,False),(130,True,True),(-2,True,True),(-9,True,False),(1,True,False)])
def test_requested_duration_distinguishes_owned_stop_from_player_failure(exit_code,intentional,wanted):
    window=sensor_window(5.)
    assert window['source_start_s']==10. and window['source_end_s']==15.
    assert window['quality_expected_stamps']==[13.1,14.1]
    completion=replay_completion(window,[10.1,11.1,12.1,13.1,14.1,15.1],exit_code,intentional)
    assert completion['completed'] is wanted


def test_full_recording_cannot_be_qualified_by_claiming_a_duration_stop():
    completion=replay_completion(sensor_window(),[20.1],130,True)
    assert not completion['completed'] and not completion['intentional_duration_stop']


def test_duration_window_excludes_a_scan_whose_measurement_end_is_after_the_cutoff():
    messages=[]
    for second in range(10,21):
        messages.extend([('/clock',NS(clock=NS(sec=second,nanosec=50_000_000))),
                         ('/uav001/lidar/points',NS(header=NS(stamp=NS(sec=second,nanosec=0))))])
    window=recorded_window(messages,'fast_livo2',{'scan_end_offset_s':.1},5.)
    assert window['source_end_s']==15.05
    assert window['expected_stamps'][-1]==14.1, 'scan ending at 15.1 is beyond the requested source window'
    assert replay_completion(window,[10.1,11.1,12.1,13.1,14.1],0,True)['completed']
