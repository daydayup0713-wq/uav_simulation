#!/usr/bin/python3
"""Export measured replay reports by consumed inputs and exact dataset identity."""
import argparse
import hashlib
import json
from pathlib import Path


def measured_scopes(entry):
    if entry.get('backend')=='glim':
        # Pinned rviz_viewer::odometry_new_frame publishes local T_odom_imu
        # for both Odometry topics. Actual global correction is its private TF.
        if entry.get('global_quality') is not None:
            entry['corrected_local_quality']=entry['global_quality']
        entry['global_quality']=None
        capabilities=dict(entry.get('capability_observations') or {})
        if 'global_pose_messages' in capabilities:
            capabilities['corrected_local_pose_messages']=capabilities.pop('global_pose_messages')
        capabilities['global_metric_scope']='not measured; odom_corrected is local fixed-window output; global map display uses actual private TF'
        entry['capability_observations']=capabilities
    return entry


def summarize(paths,fingerprints):
    groups={};seen=set()
    for path in paths:
        path=Path(path).resolve()
        if path in seen:raise ValueError('duplicate report artifact')
        seen.add(path)
        raw=path.read_bytes();report=json.loads(raw)
        group=report.get('input_group')
        if not isinstance(group,str) or not group:raise ValueError('measured input group missing')
        dataset=report.get('dataset_sha256')
        if not isinstance(dataset,dict) or not dataset:raise ValueError('dataset artifact identity missing')
        identity=hashlib.sha256(json.dumps(dataset,sort_keys=True).encode()).hexdigest()
        rate=report.get('playback_rate');duration=report.get('requested_source_duration_s')
        context=hashlib.sha256(json.dumps({'dataset':identity,'rate':rate,'duration':duration},sort_keys=True).encode()).hexdigest()
        case=groups.setdefault(group,{}).setdefault(context,{'dataset_identity':identity,'comparison_identity':context,
            'dataset_sha256':dataset,'playback_rate':rate,'requested_source_duration_s':duration,'runs':[]})
        keys=('backend','success','error','quality','global_quality','pose_graph','rtk_batch','transport',
              'capability_observations','resource_summary','latency_source_s','scope',
              'normalized_output_count','normalizer_events','implementation_sha256',
              'input_counts','playback_rate','requested_source_duration_s','player_exit_code')
        entry={key:report.get(key) for key in keys}
        entry.update(run_id=path.parent.name,source_report=str(path),
            source_report_sha256=hashlib.sha256(raw).hexdigest(),
            current_implementation=report.get('implementation_sha256')==fingerprints.get(report.get('backend'))
                and report.get('implementation_sha256') is not None,
            binary_sha256=report.get('build',{}).get('binary_sha256'))
        measured_scopes(entry)
        case['runs'].append(entry)
    result={'schema':1,'qualification':'none; consult the artifact-backed backend registry',
            'groups':[{'input_group':group,'cases':list(cases.values())} for group,cases in sorted(groups.items())]}
    # Reject nonfinite numbers rather than emit invalid browser/report JSON.
    json.dumps(result,allow_nan=False)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--catalog',type=Path,default=Path('configs/backends.json'))
    args=parser.parse_args()
    if args.output.exists():parser.error('new output required; existing evidence is immutable')
    catalog=json.loads(args.catalog.read_text())
    fingerprints={row['id']:row.get('implementation_sha256') for row in catalog['backends']}
    result=summarize(args.reports,fingerprints)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output),'groups':len(result['groups']),
        'runs':sum(len(c['runs']) for g in result['groups'] for c in g['cases'])}))


if __name__=='__main__':main()
