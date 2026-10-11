#!/usr/bin/python3
"""Build real locked planner cores in private prefixes; no fake-drone simulator."""
import argparse,json,os,re,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/uav_lab_navigation'))
from uav_lab_navigation.native_provenance import digest,fingerprint,implementation_files
from bootstrap_backends import prepare_sources,verify_sources,source_tree
ROOT=Path(__file__).resolve().parents[1]


def run(command,env):subprocess.run([str(x) for x in command],env=env,check=True)


def build(root,backend,jobs=1):
    root=Path(root);lock=json.loads((root/'dependencies/planners.lock.json').read_text())
    names=lock['backends'][backend]['repositories'];prepare_sources(root,lock,names)
    prefix=root/'.deps/planners'/backend/'install'
    env={**os.environ,'CMAKE_BUILD_PARALLEL_LEVEL':str(jobs),'OMP_NUM_THREADS':'1'}
    if backend=='fast_planner':
        nlopt=root/'.deps/planners/nlopt/install';directory=root/'.deps/planners/build/nlopt'
        run(['cmake','-S',root/'.deps/nlopt','-B',directory,'-DCMAKE_BUILD_TYPE=Release',
            '-DCMAKE_INSTALL_PREFIX='+str(nlopt),'-DNLOPT_PYTHON=OFF','-DNLOPT_OCTAVE=OFF',
            '-DNLOPT_GUILE=OFF','-DNLOPT_SWIG=OFF','-DNLOPT_TESTS=OFF','-DNLOPT_FORTRAN=OFF'],env)
        run(['cmake','--build',directory,'-j',jobs],env);run(['cmake','--install',directory],env)
        env['CMAKE_PREFIX_PATH']=str(nlopt)+':'+env.get('CMAKE_PREFIX_PATH','')
        env['LD_LIBRARY_PATH']=str(nlopt/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    directory=root/'.deps/planners/build'/backend
    run(['cmake','-S',root/'navigation/native','-B',directory,'-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_INSTALL_PREFIX='+str(prefix),'-DBACKEND='+backend,'-DUPSTREAM='+str(root/'.deps'/backend)],env)
    run(['cmake','--build',directory,'-j',jobs],env);run(['cmake','--install',directory],env)
    return write_manifest(root,backend,env)


def write_manifest(root,backend,env=None):
    root=Path(root);prefix=root/'.deps/planners'/backend/'install';binary=prefix/'bin/planner_core'
    lock=json.loads((root/'dependencies/planners.lock.json').read_text());names=lock['backends'][backend]['repositories']
    verify_sources(root,lock,names)
    env=env or dict(os.environ)
    if backend=='fast_planner':env={**env,'LD_LIBRARY_PATH':str(root/'.deps/planners/nlopt/install/lib')+':'+env.get('LD_LIBRARY_PATH','')}
    listing=subprocess.check_output(['ldd',str(binary)],text=True,env=env)
    if '=> not found' in listing:raise ValueError('planner linked dependency missing: '+listing)
    artifacts={str(binary):digest(binary)}
    for line in listing.splitlines():
        match=re.search(r'=> (/[^ ]+)',line)
        if match:artifacts[str(Path(match[1]).resolve())]=digest(Path(match[1]).resolve())
    data={'schema':1,'backend':backend,'source_ref':lock['backends'][backend]['source_ref'],
          'binary':str(binary),'binary_sha256':digest(binary),'runtime_artifacts':artifacts,
          'implementation_sha256':fingerprint(root,backend),
          'source_files':{str(p):digest(p) for p in implementation_files(root)},
          'repositories':{name:lock['repositories'][name] for name in names},
          'scope':'native upstream optimizer and platform map/ROS2 transport; no flight qualification'}
    path=prefix/'build-manifest.json';path.write_text(json.dumps(data,indent=2)+'\n');print('BUILT',path)
    return data


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',choices=['ego','fast_planner','gcopter'],required=True)
    p.add_argument('--jobs',type=int,default=1);a=p.parse_args()
    if a.jobs not in (1,2):p.error('one or two jobs required')
    build(ROOT,a.backend,a.jobs)


if __name__=='__main__':main()
