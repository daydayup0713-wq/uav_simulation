"""Capability/evidence query and bridge-owned ground configuration."""
import argparse
import json
import os
from pathlib import Path
import time

from .registry import Registry


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    listing = sub.add_parser('list')
    listing.add_argument('--role', choices=['localization', 'planning'])
    listing.add_argument('--group')
    capability = sub.add_parser('capabilities')
    capability.add_argument('backend')
    capability.add_argument('--group')
    selection = sub.add_parser('select')
    selection.add_argument('--localization', required=True)
    selection.add_argument('--planning', required=True)
    selection.add_argument('--group', required=True)
    route = sub.add_parser('route')
    route.add_argument('file', type=Path)
    route.add_argument('--timeout', type=float, default=600.)
    args = parser.parse_args(argv)
    root = Path(os.environ['LAB_ROOT'])
    try:
        registry = Registry(root / 'configs/backends.json', root / '.runtime/backend-evidence')
        if args.command == 'list':
            result = registry.list(args.role, args.group)
        elif args.command == 'capabilities':
            result = registry.describe(args.backend, args.group)
        elif args.command == 'route':
            from .flight import execute_route
            result = execute_route(args.file, args.timeout)
        else:
            import rclpy
            from uav_lab_interfaces.srv import SelectBackends
            rclpy.init()
            node = rclpy.create_node('experiment_operator')
            try:
                client = node.create_client(SelectBackends, '/uav001/experiments/select_backends')
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError('ground selection service unavailable; start lab first')
                request = SelectBackends.Request(localization=args.localization, planning=args.planning,
                                                input_group=args.group)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or future.exception():
                    raise RuntimeError('ground selection confirmation unavailable')
                response = future.result()
                if not response.success:
                    raise RuntimeError(response.reason)
                result = {'success': True, 'reason': response.reason}
            finally:
                node.destroy_node()
                rclpy.shutdown()
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (ValueError, KeyError, RuntimeError, OSError) as exc:
        print(json.dumps({'success': False, 'reason': str(exc)}))
        return 1
