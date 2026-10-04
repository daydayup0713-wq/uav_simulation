#!/usr/bin/python3
"""Build locked CPU localization libraries in this project's private prefix."""
import argparse
import json
import os
from pathlib import Path

from bootstrap import checkout_repository, run, git
from slam_config import prepare_slam

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--sources-only', action='store_true')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    lock = json.loads((ROOT / 'dependencies/lock.json').read_text())['localization']
    for name, spec in lock['repositories'].items():
        checkout_repository(spec, ROOT / '.deps' / name)
    prepare_slam(ROOT, ROOT / 'configs/slam')
    if args.sources_only:
        return 0
    prefix = ROOT / '.deps/slam/install'
    environment = dict(os.environ)
    environment['CMAKE_PREFIX_PATH'] = str(prefix) + ':' + environment.get('CMAKE_PREFIX_PATH', '')
    for name in ('gtsam_points', 'glim', 'glim_ros2'):
        build = ROOT / '.deps/slam/build-v1.1.0' / name
        run(['cmake', '-S', ROOT / '.deps' / name, '-B', build,
             '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX=' + str(prefix),
             '-DBUILD_WITH_CUDA=OFF', '-DBUILD_WITH_VIEWER=OFF',
             '-DBUILD_WITH_OPENCV=OFF', '-DBUILD_WITH_CV_BRIDGE=OFF',
             '-DBUILD_WITH_MARCH_NATIVE=OFF', '-DBUILD_WITH_TBB=ON', '-DBUILD_TESTING=OFF'],
            env=environment)
        run(['cmake', '--build', build, '-j', str(args.jobs)], env=environment)
        run(['cmake', '--install', build], env=environment)
    (prefix / 'build-manifest.json').write_text(json.dumps({
        'repositories': {name: git(ROOT / '.deps' / name, 'rev-parse', 'HEAD')
                         for name in lock['repositories']},
        'build': lock['build'], 'gtsam': lock['gtsam']}, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
