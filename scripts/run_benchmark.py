#!/usr/bin/python3
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in ('uav_lab_tools', 'uav_lab_localization'):
    sys.path.insert(0, str(ROOT/'ros2_ws/src'/package))
from uav_lab_localization.benchmark import run_benchmark

parser = argparse.ArgumentParser()
parser.add_argument('dataset')
parser.add_argument('--output', required=True)
parser.add_argument('--domain', type=int, default=77)
parser.add_argument('--skip-learning', action='store_true')
args = parser.parse_args()
result = run_benchmark(ROOT, args.dataset, args.output, args.domain, not args.skip_learning)
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['passed'] else 1)
