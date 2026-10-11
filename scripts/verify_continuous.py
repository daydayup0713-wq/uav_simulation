#!/usr/bin/python3
"""Independent observed reference/tracking/rate acceptance; no algorithm inputs."""
import argparse
import json
from pathlib import Path
import signal
import time
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration',type=float,default=600.)
    args = parser.parse_args()
    import rclpy
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
    from uav_lab_interfaces.msg import TrajectoryReference
    rclpy.init()
    node = rclpy.create_node('independent_continuous_evaluator')
    references, actual, heartbeats, setpoints = [], [], [], []
    moving = [False]
    def phase(msg):
        for status in msg.status:
            if status.name == 'uav001/flight':
                moving[0] = status.message == 'MOVING'
    def reference(msg):
        if moving[0] and msg.trajectory_id and msg.trajectory_id != 'operator-stop':
            references.append({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'wall': time.monotonic(), 'id': msg.trajectory_id, 'position': list(msg.position),
                'velocity': list(msg.velocity), 'acceleration': list(msg.acceleration)})
    def odometry(msg):
        if moving[0]:
            p = msg.pose.pose.position
            actual.append({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                           'position': [p.x, p.y, p.z]})
    node.create_subscription(TrajectoryReference, '/uav001/trajectory/reference', reference, 10)
    node.create_subscription(Odometry, '/uav001/odometry', odometry, 10)
    node.create_subscription(DiagnosticArray, '/uav001/diagnostics', phase, 10)
    node.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode',
                             lambda m: heartbeats.append(time.monotonic()) if moving[0] else None, 10)
    node.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                             lambda m: setpoints.append(time.monotonic()) if moving[0] else None, 10)
    stopping = [False]
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
    try:
        end = time.monotonic() + args.duration
        while not stopping[0] and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node(); rclpy.shutdown()
    from uav_lab_experiments.evaluation import evaluate_continuous
    report = evaluate_continuous(references, actual, heartbeats, setpoints)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    args.output.with_suffix('.samples.json').write_text(json.dumps({'references': references, 'actual': actual,
        'heartbeats_wall': heartbeats, 'setpoints_wall': setpoints}) + '\n')
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
