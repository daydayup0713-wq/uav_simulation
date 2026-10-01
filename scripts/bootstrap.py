#!/usr/bin/python3
"""Check out clean pinned dependencies; never copy cache working-tree edits."""
import argparse
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def run(args, **kwargs):
    subprocess.run([str(a) for a in args], check=True, **kwargs)

def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()

def checkout_repository(spec, dest):
    dest = Path(dest)
    cache = Path(spec.get('local_cache', '/nonexistent'))
    source = str(cache) if (cache / '.git').exists() else spec['url']
    if not (dest / '.git').exists():
        dest.parent.mkdir(parents=True, exist_ok=True)
        run(['git', 'clone', '--no-checkout', source, dest])
        run(['git', '-C', dest, 'checkout', '--detach', spec['ref']])
    expected = git(dest, 'rev-parse', spec['ref'] + '^{commit}')
    if git(dest, 'rev-parse', 'HEAD') != expected:
        raise RuntimeError(f'wrong revision in {dest}; expected {expected}')
    if git(dest, 'status', '--porcelain', '--untracked-files=no'):
        raise RuntimeError(f'dirty dependency: {dest}')
    modules = dest / '.gitmodules'
    if modules.exists():
        result = subprocess.run(['git', '-C', str(dest), 'config', '-f', '.gitmodules', '--get-regexp', r'submodule\..*\.path'], text=True, capture_output=True)
        if result.returncode not in (0, 1):
            raise RuntimeError(result.stderr)
        entries = result.stdout.splitlines()
        for entry in entries:
            key, relative = entry.split(None, 1)
            local_module = cache / relative
            module_url = git(dest, 'config', '-f', '.gitmodules', '--get', key[:-4] + 'url')
            module_sha = git(dest, 'ls-tree', 'HEAD', relative).split()[2]
            checkout_repository({'url': module_url, 'ref': module_sha,
                                 'local_cache': str(local_module)}, dest / relative)
        # Mark recursively cloned repositories as initialized submodules.
        run(['git', '-C', dest, 'submodule', 'init'])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', choices=['sources', 'agent', 'px4', 'all'], default='all')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    lock = json.loads((ROOT / 'dependencies/lock.json').read_text())
    repos = lock['repositories']
    if args.only in ('sources', 'px4', 'all'):
        checkout_repository(repos['px4'], ROOT / '.deps/px4')
        checkout_repository(repos['px4_msgs'], ROOT / 'ros2_ws/src/px4_msgs')
    if args.only in ('agent', 'all'):
        source = ROOT / '.deps/agent'
        checkout_repository(repos['agent'], source)
        run(['cmake', '-S', source, '-B', ROOT / '.deps/agent-build',
             '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX=' + str(ROOT / '.deps/agent-install'),
             '-DUAGENT_BUILD_EXECUTABLE=ON', '-DUAGENT_BUILD_TESTS=OFF'])
        run(['cmake', '--build', ROOT / '.deps/agent-build', '-j', args.jobs])
        run(['cmake', '--install', ROOT / '.deps/agent-build'])
    if args.only in ('px4', 'all'):
        environment = dict(os.environ)
        environment.update(CCACHE_DIR=str(ROOT / '.deps/ccache'), CCACHE_TEMPDIR=str(ROOT / '.deps/ccache-tmp'))
        (ROOT / '.deps/ccache-tmp').mkdir(exist_ok=True)
        run(['make', 'j='+str(args.jobs), 'px4_sitl_default'], cwd=ROOT / '.deps/px4', env=environment)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
