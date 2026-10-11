#!/usr/bin/python3
"""Compare four real planner pipelines sequentially on an archived observed map."""
import argparse,json
from pathlib import Path
from uav_lab_navigation.planner_comparison import benchmark


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot',type=Path);parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--timeout',type=float,default=8.)
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    result=benchmark(root,args.snapshot,args.output,args.manifest,args.timeout)
    print(json.dumps({'passed':result['passed'],'same_map':result['same_map'],'report':str(args.output/'report.json')},indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
