#!/usr/bin/python3
"""Send one ENU goal, including yaw, through the platform navigation action."""
import argparse
import json
import math

import rclpy
from rclpy.signals import SignalHandlerOptions
from uav_lab_navigation.cli import Navigator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', nargs=3, type=float)
    parser.add_argument('--yaw', type=float, default=0.)
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    node = nav = None
    try:
        if not all(math.isfinite(value) for value in [*args.target, args.yaw, args.timeout]) or args.timeout <= 0:
            raise ValueError('finite goal and positive finite timeout required')
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('web_goto_operator')
        nav = Navigator(node, args.timeout)
        result = nav.goto([*args.target, args.yaw])
        print(json.dumps(result, allow_nan=False), flush=True)
        return 0 if result['success'] else 1
    except (RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(json.dumps({'success': False, 'reason': str(error) or 'navigation canceled'}), flush=True)
        return 1
    finally:
        if nav: nav.destroy()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
