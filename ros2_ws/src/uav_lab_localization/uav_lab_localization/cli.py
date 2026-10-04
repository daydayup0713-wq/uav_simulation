import argparse
import json
import os
from pathlib import Path
import time
import subprocess


def localization_status(timeout=5):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    rclpy.init()
    node = rclpy.create_node('localization_status')
    result = {}
    def observe(msg):
        for status in msg.status:
            if status.name == 'uav001/localization':
                result.update({v.key: v.value for v in status.values})
                result['state'] = status.message
    node.create_subscription(DiagnosticArray, '/uav001/localization/diagnostics', observe, 10)
    try:
        end = time.monotonic()+timeout
        while time.monotonic()<end and not result:
            rclpy.spin_once(node, timeout_sec=.1)
        if not result:
            return {'ready': 'False', 'state': 'UNAVAILABLE', 'reason': 'localization diagnostics unavailable'}
        return result
    finally:
        node.destroy_node(); rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='CPU LIO/SLAM laboratory')
    commands = parser.add_subparsers(dest='command', required=True)
    status = commands.add_parser('status'); status.add_argument('--timeout', type=float, default=5)
    bench = commands.add_parser('benchmark'); bench.add_argument('dataset'); bench.add_argument('--output', required=True)
    bench.add_argument('--domain', type=int, default=77); bench.add_argument('--skip-learning', action='store_true')
    save = commands.add_parser('map-save'); save.add_argument('benchmark'); save.add_argument('destination')
    load = commands.add_parser('map-load'); load.add_argument('directory')
    locate = commands.add_parser('relocalize')
    locate.add_argument('--initial', nargs=6, type=float, required=True, metavar=('X','Y','Z','ROLL','PITCH','YAW'))
    opts = parser.parse_args(args)
    root = Path(os.environ.get('LAB_ROOT', Path.cwd()))
    try:
        if opts.command == 'status':
            result = localization_status(opts.timeout); passed = result.get('ready') == 'True'
        elif opts.command == 'benchmark':
            from .benchmark import run_benchmark
            result = run_benchmark(root, opts.dataset, opts.output, opts.domain, not opts.skip_learning)
            passed = result['passed']
        elif opts.command == 'map-save':
            from .maps import save_native_map
            result = save_native_map(root, opts.benchmark, opts.destination); passed = True
        else:
            import rclpy
            from scipy.spatial.transform import Rotation
            from uav_lab_interfaces.srv import LoadMap, Relocalize
            rclpy.init(); node = rclpy.create_node('map_command')
            try:
                srv = LoadMap if opts.command == 'map-load' else Relocalize
                service = '/uav001/localization/'+('load_map' if opts.command == 'map-load' else 'relocalize')
                client = node.create_client(srv, service)
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError('map service unavailable')
                request = srv.Request()
                if opts.command == 'map-load':
                    request.directory = str(Path(opts.directory).resolve())
                else:
                    request.initial_body_pose.header.frame_id = 'map'
                    p = request.initial_body_pose.pose.position
                    p.x, p.y, p.z = opts.initial[:3]
                    q = request.initial_body_pose.pose.orientation
                    q.x, q.y, q.z, q.w = Rotation.from_euler('xyz', opts.initial[3:]).as_quat()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                if not future.done() or future.result() is None:
                    raise RuntimeError('map operation timeout')
                response = future.result(); passed = response.success
                result = {'success': passed, 'reason': response.reason}
                if opts.command == 'relocalize':
                    t = response.map_to_odom.transform
                    result.update(inlier_fraction=response.inlier_fraction, rmse_m=response.rmse_m,
                                  map_to_odom={'xyz': [t.translation.x,t.translation.y,t.translation.z],
                                               'xyzw': [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]})
            finally:
                node.destroy_node(); rclpy.shutdown()
        print(json.dumps(result, indent=2))
        return 0 if passed else 1
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        print(json.dumps({'success': False, 'reason': str(error)})); return 1
