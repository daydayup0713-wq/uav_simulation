#!/usr/bin/python3
"""Export measured replay reports by consumed inputs and exact dataset identity."""
import argparse
import hashlib
import json
from pathlib import Path


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
        case=groups.setdefault(group,{}).setdefault(identity,{'dataset_identity':identity,
            'dataset_sha256':dataset,'runs':[]})
        keys=('backend','success','error','quality','global_quality','pose_graph',
              'capability_observations','resource_summary','latency_source_s','scope',
              'normalized_output_count','normalizer_events','implementation_sha256')
        entry={key:report.get(key) for key in keys}
        entry.update(run_id=path.parent.name,source_report=str(path),
            source_report_sha256=hashlib.sha256(raw).hexdigest(),
            current_implementation=report.get('implementation_sha256')==fingerprints.get(report.get('backend'))
                and report.get('implementation_sha256') is not None,
            binary_sha256=report.get('build',{}).get('binary_sha256'))
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
