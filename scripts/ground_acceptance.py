#!/usr/bin/python3
"""Passive ground health and process-resource sampling; sends no flight commands."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

COMPONENTS=('flight','localization','navigation')


def verdict(elapsed,duration,states,failures):
    return {'success':elapsed>=duration and not failures and
        all(states.get(name)=='READY' for name in COMPONENTS),
        'elapsed_s':elapsed,'required_duration_s':duration,'final_states':states,'failures':failures}


def diagnostic_level(value):
    return int.from_bytes(value,'little') if isinstance(value,bytes) else int(value)


def resources(run):
    try:groups=set(json.loads((run/'processes.json').read_text()).values())
    except (OSError,ValueError):return []
    rows=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            raw=(path/'stat').read_text();fields=raw[raw.rfind(')')+2:].split()
            if int(fields[2]) not in groups:continue
            row={'pid':int(path.name),'name':raw[raw.find('(')+1:raw.rfind(')')],
                'cpu_s':(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
                'rss_bytes':int(fields[21])*os.sysconf('SC_PAGE_SIZE')}
            try:row['io']={k:int(v) for k,v in (line.split(':') for line in (path/'io').read_text().splitlines())}
            except (OSError,ValueError):row['io']={}
            rows.append(row)
        except (OSError,ValueError,IndexError):continue
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=1800)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 0<args.duration<=10800:parser.error('positive bounded duration required')
    run=Path(os.environ['LAB_RUN_DIR'])
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    rclpy.init();node=rclpy.create_node('passive_ground_acceptance')
    states={};received={};failures=[];stopping=[False]
    begun=time.monotonic()
    def diagnose(message):
        for status in message.status:
            name=status.name.removeprefix('uav001/')
            if name not in COMPONENTS:continue
            states[name]=status.message;received[name]=time.monotonic()
            values={v.key:v.value for v in status.values}
            reason=None
            if diagnostic_level(status.level)>=2 or status.message in ('FAILSAFE','FAILED'):reason=name+': '+status.message+' '+values.get('reason','')
            if name=='flight' and values.get('armed')=='True':reason='ground acceptance requires unarmed vehicle'
            if reason and reason not in failures:failures.append(reason)
    for topic in ('/uav001/diagnostics','/uav001/localization/diagnostics','/uav001/navigation/diagnostics'):
        node.create_subscription(DiagnosticArray,topic,diagnose,10)
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    sampled=0.;trace=args.output.with_suffix('.jsonl')
    try:
        with trace.open('x') as stream:
            while not stopping[0] and time.monotonic()-begun<args.duration:
                rclpy.spin_once(node,timeout_sec=.1)
                now=time.monotonic()
                if now-sampled<1:continue
                sampled=now
                if now-begun>5:
                    for name in COMPONENTS:
                        if now-received.get(name,begun)>2:
                            reason=name+': diagnostic stream stale'
                            if reason not in failures:failures.append(reason)
                stream.write(json.dumps({'elapsed_s':now-begun,'states':states,'failures':failures,
                    'processes':resources(run)})+'\n');stream.flush()
    except (Exception,KeyboardInterrupt) as error:
        failures.append(type(error).__name__+': '+str(error))
    finally:node.destroy_node();rclpy.shutdown()
    result={**verdict(time.monotonic()-begun,args.duration,states,failures),'run_id':run.name,
        'scope':'wall-clock ground run; passive observation, no commands','trace':str(trace)}
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result));return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
