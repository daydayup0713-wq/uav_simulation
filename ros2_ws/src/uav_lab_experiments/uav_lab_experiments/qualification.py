"""Native experiment gates; starting a process cannot qualify an algorithm."""


def qualification_checks(stage,report):
    if stage not in ('replay_passed','realtime_passed'):raise ValueError('unsupported experiment qualification stage')
    if report.get('success') is not True or report.get('quality',{}).get('passed') is not True:
        raise ValueError('experiment or quality gate failed')
    if report.get('invalid_poses'):raise ValueError('invalid estimator poses')
    if report.get('debug_core'):raise ValueError('diagnostic run cannot qualify a backend')
    if stage=='replay_passed' and not report.get('scope','').startswith('offline replay;'):
        raise ValueError('recorded replay evidence required')
    if stage=='realtime_passed':
        if not report.get('scope','').startswith('real-time localization from measured'):
            raise ValueError('actual live sensor evidence required')
        if report.get('normalized_output_count',0)<20 or not report.get('input_counts'):
            raise ValueError('live normalized stream missing')
        events=report.get('normalizer_events',[])
        if not any(event['signature'][0]=='READY' for event in events) or any(event['signature'][0]=='FAILED' for event in events):
            raise ValueError('live localization watchdog failed or never ready')
    batch=report.get('rtk_batch')
    if batch is not None and not (batch.get('completed') is True and batch.get('quality',{}).get('passed') is True):
        raise ValueError('RTK posterior optimization gate failed')
    return {'all_passed':True,'quality':True,'valid_pose_contract':True,
            'execution':'live_sensor' if stage=='realtime_passed' else 'recorded_replay',
            'flight_qualified':False}
