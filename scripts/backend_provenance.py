"""Bind qualification to the relevant pinned sources and platform implementation."""
import hashlib,json
from pathlib import Path
from uav_lab_tools.datasets import file_hash
ROOT=Path(__file__).resolve().parents[1]

ADAPTER_FILES=[
    'scripts/bootstrap_backends.py','scripts/backend_env.sh','scripts/backend_configs.py','scripts/replay_backend.py',
    'scripts/backend_provenance.py','scripts/backend_launch.py','scripts/live_backend.py',
    'scripts/register_backend.py','scripts/replay_completion.py',
    'scripts/vins_map.py',
    'configs/fastdds-local.xml',
    'localization/interfaces/livox_ros_driver2/CMakeLists.txt','localization/interfaces/livox_ros_driver2/package.xml',
    'localization/interfaces/livox_ros_driver2/msg/CustomMsg.msg','localization/interfaces/livox_ros_driver2/msg/CustomPoint.msg',
    *['ros2_ws/src/uav_lab_experiments/uav_lab_experiments/'+name+'.py' for name in
      ('algorithm_inputs','algorithm_sensor_node','backend_contract','backend_node','localization_quality',
       'localization_frames','trajectory_evaluation','benchmark_report','qualification')]]
ADAPTER_FILES += ['ros2_ws/src/uav_lab_experiments/uav_lab_experiments/visual_contract.py',
                  'ros2_ws/src/uav_lab_experiments/uav_lab_experiments/loop_evidence.py',
                  *[str(path.relative_to(Path(__file__).resolve().parents[1])) for path in
                    sorted((Path(__file__).resolve().parents[1]/'localization/visual_adapters').rglob('*')) if path.is_file()]]


def verify_runtime_artifacts(manifest):
    inventory=manifest.get('runtime_artifacts')
    if not isinstance(inventory,dict) or not inventory:
        raise ValueError('runtime artifact inventory missing')
    for name,expected in inventory.items():
        path=Path(name)
        if not path.is_file() or file_hash(path)!=expected:
            raise ValueError('runtime artifact identity changed: '+name)


def compiled_platform_inputs(root,backend):
    directories=[]
    if backend in ('orb_slam3','vins_fusion'):
        directories.append('localization/visual_adapters')
    if backend in ('fast_lio2','fast_livo2','fast_livo2_rtk'):
        directories.append('localization/interfaces/livox_ros_driver2')
    root=Path(root)
    inventory={}
    for name in directories:
        directory=root/name
        if not directory.is_dir():raise ValueError('compiled platform input directory missing: '+name)
        for path in sorted(directory.rglob('*')):
            if path.is_file():inventory[str(path.relative_to(root))]=file_hash(path)
    return inventory


def verify_compiled_platform_inputs(manifest,backend,root=None):
    if manifest.get('compiled_platform_inputs')!=compiled_platform_inputs(root or ROOT,backend):
        raise ValueError('compiled platform input identity changed; rebuild backend')


def verified_private_build(prefix,backend,root=None):
    prefix=Path(prefix).resolve()
    manifest=json.loads((prefix/(backend+'-build-manifest.json')).read_text())
    verify_runtime_artifacts(manifest)
    verify_compiled_platform_inputs(manifest,backend,root)
    binary=Path(manifest['binary']).resolve()
    if not binary.is_relative_to(prefix) or file_hash(binary)!=manifest['binary_sha256']:
        raise ValueError('private binary identity changed')
    return manifest


def verify_measured_build(current,measured):
    # Build job count is not execution identity. Everything loaded or compiled is.
    fields=('binary','binary_sha256','runtime_artifacts','repositories','compiled_platform_inputs')
    if any(field not in measured or measured[field]!=current.get(field) for field in fields):
        raise ValueError('measured build differs from current verified private build; rerun experiment')
    verify_runtime_artifacts(measured)


def implementation_fingerprint(root,lock,backend,files=ADAPTER_FILES):
    root=Path(root)
    relevant={name:lock['repositories'][name] for name in lock['backends'][backend]['repositories']}
    snapshot={'backend':backend,'repositories':relevant,'platform_files':{name:file_hash(root/name) for name in files}}
    return hashlib.sha256(json.dumps(snapshot,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def save_provenance(root,lock,backend,output,argv):
    import shutil
    root,output=Path(root),Path(output)
    directory=output/'source-snapshot';directory.mkdir()
    for name in ADAPTER_FILES:
        target=directory/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/name,target)
    (output/'dependencies.lock.json').write_text(json.dumps(lock,indent=2)+'\n')
    snapshot={'implementation_sha256':implementation_fingerprint(root,lock,backend),
              'platform_files':{name:file_hash(root/name) for name in ADAPTER_FILES},'argv':argv}
    (output/'provenance.json').write_text(json.dumps(snapshot,indent=2)+'\n')
    return snapshot
