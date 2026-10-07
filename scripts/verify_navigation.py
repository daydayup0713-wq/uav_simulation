#!/usr/bin/python3
"""Independent truth-based evaluator; scene geometry never enters navigation nodes."""
import argparse
import json
import time
import signal
from pathlib import Path
import numpy as np


def clearance(position,lower,upper,halfsize):
    p,lo,hi,h=[np.asarray(v,dtype=float) for v in (position,lower,upper,halfsize)]
    gap=np.maximum(lo-(p+h),(p-h)-hi)
    return float(np.linalg.norm(np.maximum(gap,0))) if np.any(gap>0) else float(gap.max())


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--duration',type=float,default=240.)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry,Path as NavPath
    from diagnostic_msgs.msg import DiagnosticArray
    rclpy.init();node=rclpy.create_node('independent_navigation_evaluator');samples=[];paths=[];phase=['UNKNOWN']
    def state(msg):
        for status in msg.status:
            if status.name=='uav001/flight':phase[0]=status.message
    def truth(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        samples.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,'position':[p.x,p.y,p.z],
            'quaternion':[q.x,q.y,q.z,q.w],'phase':phase[0]})
    def route(msg):paths.append([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
    node.create_subscription(Odometry,'/uav001/ground_truth/odometry',truth,qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray,'/uav001/diagnostics',state,10)
    node.create_subscription(NavPath,'/uav001/navigation/path',route,10)
    stopping=[False]
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    try:
        end=time.monotonic()+args.duration
        while not stopping[0] and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    finally:node.destroy_node();rclpy.shutdown()
    airborne=[s for s in samples if s['position'][2]>1. and s['phase'] in ('MOVING','HOLDING')]
    net=[clearance(s['position'],[1.3,-1,0],[1.7,1,4],[.4,.4,.3]) for s in airborne]
    speeds=[]
    for a,b in zip(samples,samples[1:]):
        dt=b['stamp']-a['stamp']
        if 0<dt<.2 and a['phase']=='MOVING' and b['phase']=='MOVING':speeds.append(float(np.linalg.norm(np.asarray(b['position'])-a['position'])/dt))
    report={'source':'Gazebo ground truth, physical navigation divider; evaluator only','samples':len(samples),'airborne_samples':len(airborne),
        'body_halfsize_m':[.4,.4,.3],'min_body_clearance_m':min(net) if net else None,
        'max_measured_speed_mps':max(speeds) if speeds else None,'paths':paths,
        'passed':bool(net and min(net)>.25 and paths),'speed_scope':'measured response; .5m/s and .5m/s² bounds apply to commanded reference'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    args.output.with_suffix('.samples.json').write_text(json.dumps(samples)+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
