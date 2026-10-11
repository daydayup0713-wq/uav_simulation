"""Verify source-associated planner binaries before accepting their output."""
import hashlib
import json
from pathlib import Path


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def implementation_files(root):
    root=Path(root)
    return [root/'dependencies/planners.lock.json',
            root/'scripts/bootstrap_planners.py',
            *[p for p in sorted((root/'navigation/native').rglob('*')) if p.is_file()],
            *[root/'ros2_ws/src/uav_lab_navigation/uav_lab_navigation'/name for name in
              ('planner_backends.py','native_provenance.py','planner.py','occupancy.py')],
            root/'ros2_ws/src/uav_lab_bridge/uav_lab_bridge/continuous_trajectory.py']


def fingerprint(root,backend):
    root=Path(root)
    value={'backend':backend,'files':{str(p.relative_to(root)):digest(p) for p in implementation_files(root)}}
    return hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()


def verified_build(root,backend):
    root=Path(root).resolve();prefix=root/'.deps/planners'/backend/'install'
    manifest=json.loads((prefix/'build-manifest.json').read_text())
    if manifest.get('backend')!=backend or manifest.get('implementation_sha256')!=fingerprint(root,backend):
        raise ValueError('planner implementation changed; rebuild and reverify')
    inventory=manifest.get('runtime_artifacts')
    if not isinstance(inventory,dict) or not inventory:raise ValueError('planner runtime inventory missing')
    for path,sha in inventory.items():
        if not Path(path).is_file() or digest(path)!=sha:raise ValueError('planner runtime artifact changed: '+path)
    binary=Path(manifest['binary']).resolve()
    if not binary.is_relative_to(prefix) or digest(binary)!=manifest['binary_sha256']:
        raise ValueError('planner binary identity changed')
    return manifest
