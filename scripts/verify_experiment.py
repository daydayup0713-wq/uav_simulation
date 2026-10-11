#!/usr/bin/python3
"""Observe complex continuous navigation independently of localization/planning."""
import argparse,json,signal,time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True);parser.add_argument('--runs',type=int,default=1)
    parser.add_argument('--duration',type=float,default=3600.);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from uav_lab_interfaces.msg import TrajectoryReference
    rclpy.init();node=rclpy.create_node('independent_complex_evaluator')
    samples=[];references=[];phase=['UNKNOWN'];stopping=[False]
    def state(message):
        for status in message.status:
            if status.name=='uav001/flight':phase[0]=status.message
    def truth(message):
        p,q=message.pose.pose.position,message.pose.pose.orientation
        samples.append({'stamp':message.header.stamp.sec+message.header.stamp.nanosec/1e9,
            'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],'phase':phase[0]})
    def reference(message):
        if phase[0]=='MOVING' and message.trajectory_id and message.trajectory_id!='operator-stop':
            references.append({'stamp':message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                'id':message.trajectory_id,'position':list(message.position)})
    node.create_subscription(Odometry,'/uav001/ground_truth/odometry',truth,qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray,'/uav001/diagnostics',state,10)
    node.create_subscription(TrajectoryReference,'/uav001/trajectory/reference',reference,10)
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    try:
        deadline=time.monotonic()+args.duration
        while not stopping[0] and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    finally:node.destroy_node();rclpy.shutdown()
    from experiment_evaluation import evaluate
    directory=args.run/'configuration/scene'
    events=[json.loads(line) for line in (args.run/'navigation.jsonl').read_text().splitlines()]
    report=evaluate(json.loads((directory/'sensor-geometry.json').read_text()),
                    json.loads((directory/'route.json').read_text()),samples,references,events,args.runs)
    from handoff_evidence import recheck
    report['accepted_curve_check']=recheck([json.loads(line) for line in (args.run/'events.jsonl').read_text().splitlines()])
    if not report['accepted_curve_check']['passed']:
        report['passed']=False;report['failed_checks'].append('accepted_curve_check')
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    args.output.with_suffix('.samples.json').write_text(json.dumps({'truth':samples,'references':references})+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
