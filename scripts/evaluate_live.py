#!/usr/bin/python3
"""Evaluate already-produced LIO separately from the independent truth bag."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ('uav_lab_tools','uav_lab_localization'):
    sys.path.insert(0,str(ROOT/'ros2_ws/src'/package))
from uav_lab_localization.benchmark import read_dataset, TRUTH, stamp
from uav_lab_localization.evaluation import evaluate
from uav_lab_tools.datasets import load_dataset, file_hash

parser=argparse.ArgumentParser()
parser.add_argument('dataset');parser.add_argument('trace');parser.add_argument('--output',required=True)
args=parser.parse_args()
meta=load_dataset(args.dataset)
truth=[]
for _, msg, _, _ in read_dataset(args.dataset,[TRUTH]):
    p,q=msg.pose.pose.position,msg.pose.pose.orientation
    truth.append([stamp(msg)/1e9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
truth=np.asarray(truth)
rows=[json.loads(line) for line in Path(args.trace).read_text().splitlines()]
rejected=[r for r in rows if r.get('event')=='scan_rejected']
rows=[r for r in rows if 'position' in r and truth[0,0]<=r['stamp']<=truth[-1,0]]
estimate=np.asarray([[r['stamp'],*r['position'],*r['quaternion']] for r in rows])
expected=np.arange(np.ceil(truth[0,0]*10)/10,truth[-1,0]+1e-6,.1)
report=evaluate(estimate,truth,expected)
report.update(source_age_p95_s=float(np.quantile([r['source_age_s'] for r in rows],.95)),
              rejected_scans=len(rejected),
              source_age_max_s=float(max(r['source_age_s'] for r in rows)),
              ready_fraction=float(np.mean([r['ready'] for r in rows])),
              input_sha256=meta['bag_sha256'],trace_sha256=file_hash(args.trace))
report['passed']=(report['ate_rmse_m']<=.3 and report['attitude_rmse_deg']<=5 and report['coverage']>=.95 and report['ready_fraction']==1)
Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
