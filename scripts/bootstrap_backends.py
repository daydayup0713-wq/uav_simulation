#!/usr/bin/python3
"""Build pinned localization backends into independent private prefixes."""
import argparse,hashlib,json,os,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def run(args,env=None,cwd=None):
    subprocess.run([str(x) for x in args],env=env,cwd=cwd,check=True)

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def source_tree(root):
    paths=subprocess.check_output(['git','-C',str(root),'ls-files','-co','--exclude-standard','-z']).decode().split('\0')
    value=hashlib.sha256()
    for name in sorted(set(paths)):
        path=root/name
        if not name or not path.is_file() or path.suffix in ('.so','.a','.o','.pyc'):continue
        value.update(name.encode()+b'\0');value.update(path.read_bytes())
    return value.hexdigest()

def verify_sources(root,lock,names):
    for name in names:
        spec=lock['repositories'][name];path=Path(root)/'.deps'/name
        if not path.is_dir():raise ValueError('locked source missing: '+name)
        ref=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
        if ref!=spec['ref'] or source_tree(path)!=spec['tree_sha256']:
            raise ValueError('locked source identity changed: '+name)
        for patch in spec['patches']:
            if sha(Path(root)/patch['path'])!=patch['sha256']:raise ValueError('locked patch identity changed: '+name)
        for relative,expected in spec.get('submodules',{}).items():
            child=path/relative
            actual=subprocess.check_output(['git','-C',str(child),'rev-parse','HEAD'],text=True).strip()
            if actual!=expected or subprocess.check_output(['git','-C',str(child),'status','--porcelain']):
                raise ValueError('locked submodule identity changed: '+name)

def prepare_sources(root,lock,names):
    from bootstrap import checkout_repository
    for name in names:
        spec=lock['repositories'][name];path=Path(root)/'.deps'/name
        if not path.is_dir():checkout_repository(spec,path)
        if source_tree(path)==spec['tree_sha256']:continue
        status=subprocess.check_output(['git','-C',str(path),'status','--porcelain','--untracked-files=no'])
        if status:raise ValueError('unexpected source edits; cannot apply locked patches: '+name)
        for patch in spec['patches']:
            file=Path(root)/patch['path']
            if sha(file)!=patch['sha256']:raise ValueError('locked patch identity changed: '+name)
            run(['git','-C',path,'apply',file])
    verify_sources(root,lock,names)
def native(name,source,prefix,jobs,arguments=(),target=None):
    build=ROOT/'.deps/backends/build'/name
    run(['cmake','-S',source,'-B',build,'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(prefix),*arguments])
    run(['cmake','--build',build,'-j',jobs,*(['--target',target] if target else [])])
    run(['cmake','--install',build])

def colcon(name,sources,prefix,jobs):
    base=ROOT/'.deps/backends'/name;base.mkdir(parents=True,exist_ok=True)
    env={**os.environ,'CMAKE_BUILD_PARALLEL_LEVEL':str(jobs),'MAKEFLAGS':'-j'+str(jobs),
         'CMAKE_PREFIX_PATH':str(prefix)+':'+os.environ.get('CMAKE_PREFIX_PATH','')}
    run(['colcon','build','--base-paths',*sources,'--build-base',base/'build','--install-base',prefix,
         '--merge-install','--parallel-workers','1','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],env,cwd=base)

def manifest(backend,prefix,binary,repositories,jobs=1):
    output=prefix/(backend+'-build-manifest.json')
    output.write_text(json.dumps({'schema':1,'backend':backend,'binary':str(binary),'binary_sha256':sha(binary),
        'repositories':{name:{'ref':subprocess.check_output(['git','-C',str(ROOT/'.deps'/name),'rev-parse','HEAD'],text=True).strip(),
            'tree_sha256':source_tree(ROOT/'.deps'/name),
            'diff_sha256':hashlib.sha256(subprocess.check_output(['git','-C',str(ROOT/'.deps'/name),'diff','--binary'])).hexdigest()} for name in repositories},
        'build':{'jobs':jobs,'python':'system3.10','ros':'humble','install':'private'}},indent=2)+'\n')
    print('BUILT '+str(output),flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--backend',choices=['fast_lio2','fast_livo2','fast_livo2_rtk'],required=True)
    parser.add_argument('--jobs',type=int,default=1);args=parser.parse_args()
    if not 1<=args.jobs<=2:parser.error('one or two jobs on this machine')
    lock=json.loads((ROOT/'dependencies/backends.lock.json').read_text())
    prepare_sources(ROOT,lock,lock['backends'][args.backend]['repositories'])
    common=ROOT/'.deps/backends/common/install'
    colcon('common',[ROOT/'localization/interfaces/livox_ros_driver2'],common,args.jobs)
    os.environ['CMAKE_PREFIX_PATH']=str(common)+':'+os.environ.get('CMAKE_PREFIX_PATH','')
    if args.backend=='fast_lio2':
        prefix=ROOT/'.deps/backends/fast_lio2/install'
        colcon('fast_lio2',[ROOT/'.deps/fast_lio2'],prefix,args.jobs)
        manifest(args.backend,prefix,prefix/'lib/fast_lio/fastlio_mapping',['fast_lio2','livox-driver2'],args.jobs);return
    prefix=ROOT/'.deps/backends'/args.backend/'install'
    if args.backend=='fast_livo2':
        native('sophus-modern',ROOT/'.deps/sophus-modern',prefix,args.jobs,['-DBUILD_SOPHUS_TESTS=OFF','-DBUILD_SOPHUS_EXAMPLES=OFF'])
        os.environ['CMAKE_PREFIX_PATH']=str(prefix)+':'+os.environ['CMAKE_PREFIX_PATH']
        native('vikit-modern',ROOT/'.deps/vikit-modern/vikit_common',prefix,args.jobs,target='vikit_common')
        colcon(args.backend,[ROOT/'.deps/vikit-modern/vikit_ros',ROOT/'.deps/fast_livo2'],prefix,args.jobs)
        manifest(args.backend,prefix,prefix/'lib/fast_livo2/fastlivo_mapping',['fast_livo2','sophus-modern','vikit-modern','intel-robotics'],args.jobs);return
    native('sophus-legacy',ROOT/'.deps/sophus-legacy',prefix,args.jobs,target='Sophus')
    config=prefix/'lib/cmake/Sophus';config.mkdir(parents=True,exist_ok=True)
    (config/'SophusConfig.cmake').write_text('set(Sophus_INCLUDE_DIRS "'+str(prefix/'include')+'")\nset(Sophus_LIBRARIES "'+str(prefix/'lib/libSophus.so')+'")\nif(NOT TARGET Sophus::Sophus)\nadd_library(Sophus::Sophus INTERFACE IMPORTED)\nset_target_properties(Sophus::Sophus PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "'+str(prefix/'include')+'" INTERFACE_LINK_LIBRARIES "'+str(prefix/'lib/libSophus.so')+'")\nendif()\n')
    native('geographiclib',ROOT/'.deps/geographiclib',prefix,args.jobs,['-DBUILD_SHARED_LIBS=ON','-DBUILD_DOCUMENTATION=OFF'])
    os.environ['CMAKE_PREFIX_PATH']=str(prefix)+':'+os.environ['CMAKE_PREFIX_PATH']
    source=ROOT/'.deps/fast_livo2_rtk_ros2'
    colcon(args.backend,[source/'src',source/'thirdparty'],prefix,args.jobs)
    manifest(args.backend,prefix,prefix/'lib/fast_livo/fastlivo_mapping',['fast_livo2_rtk_ros2','sophus-legacy','geographiclib'],args.jobs)

if __name__=='__main__':main()
