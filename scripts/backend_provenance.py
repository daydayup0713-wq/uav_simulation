"""Bind qualification to the relevant pinned sources and platform implementation."""
import hashlib,json
from pathlib import Path
from uav_lab_tools.datasets import file_hash

ADAPTER_FILES=[
    'scripts/bootstrap_backends.py','scripts/backend_env.sh','scripts/backend_configs.py','scripts/replay_backend.py',
    'scripts/backend_provenance.py','scripts/backend_launch.py','scripts/live_backend.py',
    'localization/interfaces/livox_ros_driver2/CMakeLists.txt','localization/interfaces/livox_ros_driver2/package.xml',
    'localization/interfaces/livox_ros_driver2/msg/CustomMsg.msg','localization/interfaces/livox_ros_driver2/msg/CustomPoint.msg',
    *['ros2_ws/src/uav_lab_experiments/uav_lab_experiments/'+name+'.py' for name in
      ('algorithm_inputs','algorithm_sensor_node','backend_contract','backend_node','localization_quality',
       'localization_frames','trajectory_evaluation','benchmark_report')]]


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
