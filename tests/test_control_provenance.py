import importlib,importlib.util,json
from pathlib import Path


def api():
    assert importlib.util.find_spec('control_provenance'),'closed-loop control source binding missing'
    return importlib.import_module('control_provenance')


def test_source_snapshot_is_verifiable_and_edits_invalidate_current_qualification(tmp_path):
    source=tmp_path/'controller.py';source.write_text('bounded controller')
    root=tmp_path/'repo';root.mkdir();target=root/'controller.py';target.write_bytes(source.read_bytes())
    output=root/'configuration/control-source'
    report=api().snapshot(root,output,files=[target])
    assert api().matches(root,report,files=[target])
    target.write_text('modified controller')
    assert not api().matches(root,report,files=[target])
    assert (output/'controller.py').read_text()=='bounded controller'


def test_pair_qualification_is_specific_and_artifacts_cannot_change(tmp_path,monkeypatch):
    module=api();root=tmp_path;source=root/'controller.py';source.write_text('control')
    monkeypatch.setattr(module,'control_files',lambda r:[source])
    proof=root/'accepted.json';proof.write_text('independent native evidence')
    from uav_lab_experiments.registry import digest
    implementation=module.snapshot(root,root/'snapshot')
    directory=root/'.runtime/pair-evidence';directory.mkdir(parents=True)
    catalog=root/'configs';catalog.mkdir()
    values={'schema':1,'backends':[{'id':'glim','implementation_sha256':'a'*64},{'id':'ego','implementation_sha256':'b'*64}]}
    (catalog/'backends.json').write_text(json.dumps(values))
    report={'schema':1,'success':True,'pair':{'localization':'glim','planning':'ego','input_group':'sync'},
        'control_implementation':implementation,'backend_implementations':{'glim':'a'*64,'ego':'b'*64},
        'artifacts':[{'path':str(proof),'sha256':digest(proof)}]}
    (directory/'normal.json').write_text(json.dumps(report))
    assert module.qualified_pair(root,'glim','ego','sync')
    assert not module.qualified_pair(root,'glim','gcopter','sync')
    values['backends'][0]['implementation_sha256']='c'*64
    (catalog/'backends.json').write_text(json.dumps(values))
    assert not module.qualified_pair(root,'glim','ego','sync')
    values['backends'][0]['implementation_sha256']='a'*64
    (catalog/'backends.json').write_text(json.dumps(values))
    proof.write_text('changed evidence')
    assert not module.qualified_pair(root,'glim','ego','sync')


def test_control_source_inventory_includes_the_consumed_sensor_contract():
    root=Path(__file__).resolve().parents[1]
    names={str(p.relative_to(root)) for p in api().control_files(root)}
    assert 'configs/navigation-sensors.json' in names
    assert 'configs/sensors-livox-rtk.json' in names
def test_control_identity_includes_stop_map_wire_contract():
    import control_provenance
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    names={str(path.relative_to(root)) for path in control_provenance.control_files(root)}
    assert 'ros2_ws/src/uav_lab_interfaces/msg/CollisionSnapshot.msg' in names
    assert 'ros2_ws/src/uav_lab_interfaces/CMakeLists.txt' in names


def test_large_sensor_dds_contract_is_bound_to_algorithm_and_control_identity():
    import backend_provenance,control_provenance
    root=Path(__file__).resolve().parents[1]
    assert 'configs/fastdds-local.xml' in backend_provenance.ADAPTER_FILES
    assert root/'configs/fastdds-local.xml' in control_provenance.control_files(root)


def test_qualification_policy_is_part_of_current_control_source_identity():
    root=Path(__file__).resolve().parents[1]
    assert root/'ros2_ws/src/uav_lab_experiments/uav_lab_experiments/registry.py' in api().control_files(root)
