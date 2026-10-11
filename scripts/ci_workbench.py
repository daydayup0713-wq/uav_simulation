#!/usr/bin/python3
"""CPU-only real-core fixtures; analytical observations never qualify flight."""
import argparse,json,os,subprocess
from pathlib import Path
import numpy as np


def prepare_map(destination):
    from uav_lab_navigation.fixture import write_fixture
    from uav_lab_navigation.benchmark import benchmark
    from uav_lab_navigation.occupancy import VoxelMap
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    write_fixture(destination/'observations')
    report=benchmark(destination/'observations',destination/'mapping')
    config=report['configuration'];grid=VoxelMap(config['resolution_m'],config['lower'],config['upper'])
    with np.load(destination/'mapping/grid.npz',allow_pickle=False) as data:
        grid.score=data['score'].copy();grid.seen=data['seen'].copy();grid.version=report['map_version']
    envelope=np.asarray(config['body_halfsize_m'])+config['clearance_m'];collision=grid.snapshot(envelope)
    snapshot=destination/'observed-inflated.npz'
    np.savez_compressed(snapshot,free=collision.free,lower=collision.lower,resolution=collision.resolution,
        map_version=collision.version,source_stamp=0.,envelope=envelope,configuration=json.dumps(config))
    manifest=destination/'manifest.json'
    manifest.write_text(json.dumps({'run_id':'ci-analytical-observations','scope':'analytical observation fixture; no rendered sensors or PX4 flight',
        'observation_sha256':report['input_sha256'],'configuration':config},indent=2)+'\n')
    return snapshot,manifest


def replay_environment(environment):return {**environment,'LAB_DOMAIN_ID':'77'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    if args.output.exists():parser.error('exclusive evidence destination required')
    from uav_lab_navigation.native_provenance import verified_build
    from backend_provenance import verified_private_build
    for name in ('ego','fast_planner','gcopter'):verified_build(root,name)
    verified_private_build(root/'.deps/backends/glim/install','glim')
    args.output.mkdir(parents=True)
    def run(command,env=None):subprocess.run([str(x) for x in command],cwd=root,check=True,env=env)
    run([root/'scripts/env.sh','env','PYTEST_DISABLE_PLUGIN_AUTOLOAD=1','/usr/bin/python3','-m','pytest','-q','tests/test_native_planners.py'])
    snapshot,manifest=prepare_map(args.output/'map-input')
    run([root/'scripts/env.sh','/usr/bin/python3',root/'scripts/benchmark_planners.py',snapshot,'--manifest',manifest,
        '--output',args.output/'planning','--timeout','20'])
    run([root/'scripts/env.sh','/usr/bin/python3',root/'scripts/make_lio_fixture.py',args.output/'lio-fixture'])
    run([root/'scripts/env.sh','/usr/bin/python3',root/'scripts/replay_backend.py','--backend','glim',
        '--dataset',args.output/'lio-fixture','--output',args.output/'localization'],env=replay_environment(os.environ))
    print(json.dumps({'success':True,'scope':'CPU real cores and analytical fixtures; no flight qualification','output':str(args.output)}))


if __name__=='__main__':main()
