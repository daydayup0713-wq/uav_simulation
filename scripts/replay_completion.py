"""Recording-derived replay windows; received prefixes never set the denominator."""
import math


def recorded_window(messages,backend,calibration,duration=0.):
    primary='/uav001/camera/image_raw' if backend in ('orb_slam3','vins_fusion') else '/uav001/lidar/points'
    offset=0. if backend in ('glim','lio_sam','orb_slam3','vins_fusion') else calibration.get('scan_end_offset_s',.1)
    clocks=[];stamps=[]
    for topic,msg,*unused in messages:
        if topic=='/clock':clocks.append(msg.clock.sec+msg.clock.nanosec/1e9)
        elif topic==primary:stamps.append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9+offset)
    if not clocks or not stamps or not all(math.isfinite(t) for t in clocks+stamps):
        raise ValueError('recorded replay source window missing')
    if any(b<a for a,b in zip(clocks,clocks[1:])) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('recorded replay source timestamps regressed or duplicated')
    recording_end=max(clocks[-1],stamps[-1])
    end=min(recording_end,clocks[0]+duration) if duration else recording_end
    # Stamps already represent measurement end. Adding the scan duration here
    # would include data not yet available when the owned player is stopped.
    selected=[t for t in stamps if t<=end+1e-6]
    if not selected:raise ValueError('recorded replay source window empty')
    return {'primary_topic':primary,'source_start_s':clocks[0],'source_end_s':end,
            'requested_duration_s':duration or None,'expected_stamps':selected,
            'quality_expected_stamps':[t for t in selected if t>=stamps[0]+3.]}


def replay_completion(window,observed,exit_code,intentional_duration_stop=False):
    expected=window['expected_stamps']
    intentional=intentional_duration_stop is True and window['requested_duration_s'] is not None
    exit_ok=exit_code in (0,130,-2) if intentional else exit_code==0
    supported=[t for t in observed if math.isfinite(t) and expected[0]-1e-6<=t<=expected[-1]+1e-6]
    endpoint=bool(supported) and max(supported)>=expected[-1]-1e-6
    return {'completed':bool(exit_ok and endpoint),'mode':'duration' if intentional else 'eof',
            'intentional_duration_stop':intentional,'player_exit_code':exit_code,
            'expected_count':len(expected),'observed_count':len(supported),
            'expected_start_s':expected[0],'expected_end_s':expected[-1],
            'observed_end_s':max(supported) if supported else None,
            'denominator':'recorded primary sensor timestamps; explicit 3s initialization excluded from quality',
            'reason':'complete' if exit_ok and endpoint else 'player failure or recorded source window incomplete'}
