#!/usr/bin/python3
"""Execute the archived route on an explicitly armed hovering vehicle."""
import argparse,json
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from uav_lab_navigation.cli import Navigator,pose


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('route',type=Path);args=parser.parse_args()
    data=json.loads(args.route.read_text())
    if data.get('frame')!='odom' or data.get('dwell'):raise ValueError('continuous odom route required')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('web_route_operator');nav=Navigator(node,3300.)
    try:
        state=nav.operator.status()
        if state.get('armed')!='True' or state.get('landed')!='False' or state.get('phase')!='HOLDING':
            raise RuntimeError('explicit armed airborne hover required')
        result=nav.route([pose(point) for point in data['controls']]);print(json.dumps(result),flush=True)
        return 0 if result['success'] else 1
    except (RuntimeError,ValueError,KeyboardInterrupt) as error:
        print(json.dumps({'success':False,'reason':str(error) or 'route canceled'}),flush=True);return 1
    finally:nav.destroy();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
