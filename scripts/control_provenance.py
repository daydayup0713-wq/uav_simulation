"""Archive and bind the exact control/experiment implementation in each native run."""
import hashlib,json
from pathlib import Path


def control_files(root):
    root=Path(root)
    names=['scripts/supervise.py','scripts/sensor_model.py','scripts/experiment_scenarios.py',
        'scripts/experiment_configuration.py','scripts/rendering_configuration.py','scripts/control_provenance.py','scripts/env.sh','scripts/start_lab.sh',
        'configs/navigation.json','configs/navigation-sensors.json','configs/navigation-corridor-sensors.json','configs/sensors.json',
        'configs/sensors-livox.json','configs/sensors-livox-rtk.json','configs/sensors-mechanical.json',
        'configs/fastdds-local.xml','scripts/qualify_pair.py',
        'configs/px4-start.sh','dependencies/lock.json','dependencies/backends.lock.json',
        'dependencies/planners.lock.json',
        'ros2_ws/src/uav_lab_experiments/uav_lab_experiments/registry.py']
    files=[root/name for name in names]
    for package in ('uav_lab_bridge','uav_lab_navigation'):
        files.extend((root/'ros2_ws/src'/package/package).glob('*.py'))
    interface=root/'ros2_ws/src/uav_lab_interfaces'
    files.extend([interface/'CMakeLists.txt',interface/'package.xml'])
    for kind in ('action','msg','srv'):files.extend((interface/kind).glob('*.'+kind))
    return sorted(files)


def inventory(root,files=None):
    root=Path(root)
    return {str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (control_files(root) if files is None else files)}


def snapshot(root,output,files=None):
    root,output=Path(root),Path(output);values=inventory(root,files)
    output.mkdir(parents=True,exist_ok=False)
    for name in values:
        target=output/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((root/name).read_bytes())
    report={'schema':1,'files':values,'sha256':hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest()}
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def matches(root,report,files=None):
    try:
        actual=inventory(root,files)
        return report['schema']==1 and report['files']==actual and report['sha256']==hashlib.sha256(json.dumps(actual,sort_keys=True).encode()).hexdigest()
    except (KeyError,ValueError,OSError):return False


def qualified_pair(root,localization,planning,group):
    from uav_lab_experiments.registry import digest
    wanted={'localization':localization,'planning':planning,'input_group':group}
    try:
        catalog=json.loads((Path(root)/'configs/backends.json').read_text())
        implementations={item['id']:item['implementation_sha256'] for item in catalog['backends']
            if item['id'] in (localization,planning)}
        if set(implementations)!={localization,planning} or not all(
            isinstance(value,str) and len(value)==64 for value in implementations.values()):return None
    except (OSError,ValueError,KeyError,TypeError):return None
    for path in (Path(root)/'.runtime/pair-evidence').glob('*.json'):
        try:
            report=json.loads(path.read_text())
            if (report['schema']==1 and report['success'] is True and report['pair']==wanted
                    and report.get('backend_implementations')==implementations
                    and matches(root,report['control_implementation']) and report['artifacts']
                    and all(digest(a['path'])==a['sha256'] for a in report['artifacts'])):
                return str(path)
        except (OSError,ValueError,KeyError,TypeError):continue
    return None
