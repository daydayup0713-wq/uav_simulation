#!/usr/bin/python3
"""Observe PX4 independently while stopping only the current lab's bridge group."""
import json
import os
from pathlib import Path
import signal
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleStatus, VehicleLandDetected

def main():
    root = Path(os.environ['LAB_ROOT'])
    run = Path((root/'.runtime/current-run').read_text().strip())
    if run.parent != root/'.runtime' or not (run/'ready').exists():
        raise RuntimeError('current run is not a ready lab-owned runtime')
    pids = json.loads((run/'processes.json').read_text())
    bridge_pid = pids['bridge']
    # Avoid acting on stale PID files after a completed run.
    command = Path(f'/proc/{bridge_pid}/cmdline').read_bytes()
    if b'uav_lab_bridge' not in command or os.getpgid(bridge_pid) != bridge_pid:
        raise RuntimeError('bridge PID identity mismatch')
    rclpy.init()
    node = rclpy.create_node('fault_observer')
    status = {'armed': None, 'offboard': None, 'landed': None}
    received = {}
    events = []
    def state(msg):
        status.update(armed=msg.arming_state==VehicleStatus.ARMING_STATE_ARMED,
                      offboard=msg.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        received['status'] = time.monotonic()
        events.append({'time': time.monotonic(), 'nav_state': msg.nav_state, **status})
    def land(msg):
        status['landed'] = bool(msg.landed)
        received['land'] = time.monotonic()
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', state, qos_profile_sensor_data)
    node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', land, qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+5
        while (None in status.values()) and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not status['armed'] or not status['offboard'] or status['landed']:
            raise RuntimeError('fault test requires airborne armed Offboard vehicle')
        started = time.monotonic()
        os.killpg(bridge_pid, signal.SIGTERM)
        deadline = started+50
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            fresh = all(time.monotonic()-received.get(k,0)<1 for k in ('status','land'))
            if fresh and status['landed'] and not status['armed'] and not status['offboard']:
                report = {'passed': True, 'failure': 'bridge_exit', 'elapsed_s': time.monotonic()-started,
                          'final': status, 'events': events}
                (run/'fault-acceptance.json').write_text(json.dumps(report, indent=2)+'\n')
                print(json.dumps({k:v for k,v in report.items() if k!='events'}), flush=True)
                return 0
        raise RuntimeError('PX4 did not land/disarm after bridge exit within 50s')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    raise SystemExit(main())
