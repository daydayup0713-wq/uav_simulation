#!/usr/bin/python3
"""Observe PX4 independently while stopping only the current lab's bridge group."""
import json
import os
from pathlib import Path
import signal
import time
import argparse


def component_identity(component, command):
    tokens = command.split(b'\0')
    package, executable = {'bridge': ('uav_lab_bridge','bridge'),
                           'lio': ('glim_ros','glim_rosnode'),
                           'localization': ('uav_lab_localization','localization')}[component]
    return any(tokens[i:i+3] == [b'run', package.encode(), executable.encode()] and
               i > 0 and Path(tokens[i-1].decode()).name == 'ros2' for i in range(len(tokens)))

def landing_completed(status, fresh, landing_observed):
    # PX4 can restore the previous nav_state after disarming. Observe the
    # airborne landing transition, then require actual landed + disarmed.
    return fresh and landing_observed and status['landed'] and not status['armed']

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--component', choices=('bridge','lio','localization'), default='bridge')
    options = parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected
    root = Path(os.environ['LAB_ROOT'])
    run = Path((root/'.runtime/current-run').read_text().strip())
    if run.parent != root/'.runtime' or not (run/'ready').exists():
        raise RuntimeError('current run is not a ready lab-owned runtime')
    pids = json.loads((run/'processes.json').read_text())
    bridge_pid = pids[options.component]
    # Avoid acting on stale PID files after a completed run.
    command = Path(f'/proc/{bridge_pid}/cmdline').read_bytes()
    if not component_identity(options.component, command) or os.getpgid(bridge_pid) != bridge_pid:
        raise RuntimeError('fault component PID identity mismatch')
    rclpy.init()
    node = rclpy.create_node('fault_observer')
    status = {'armed': None, 'offboard': None, 'landed': None}
    received = {}
    events = []
    started = None
    observed = {'landing_while_armed': False}
    def state(msg):
        status.update(armed=msg.arming_state==VehicleStatus.ARMING_STATE_ARMED,
                      offboard=msg.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        received['status'] = time.monotonic()
        events.append({'time': time.monotonic(), 'nav_state': msg.nav_state, **status})
        if started is not None and status['armed'] and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            observed['landing_while_armed'] = True
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
            if landing_completed(status, fresh, observed['landing_while_armed']):
                report = {'passed': True, 'failure': options.component+'_exit', 'elapsed_s': time.monotonic()-started,
                          'final': status, 'failsafe_landing_observed': True, 'events': events}
                (run/'fault-acceptance.json').write_text(json.dumps(report, indent=2)+'\n')
                print(json.dumps({k:v for k,v in report.items() if k!='events'}), flush=True)
                return 0
        (run/'fault-acceptance.json').write_text(json.dumps({'passed': False, 'failure': options.component+'_exit',
            'final': status, 'observed': observed, 'events': events}, indent=2)+'\n')
        raise RuntimeError('PX4 failsafe landing/disarm not independently confirmed within 50s')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    raise SystemExit(main())
