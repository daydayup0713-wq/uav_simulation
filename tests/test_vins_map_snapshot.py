import hashlib
from pathlib import Path
import numpy as np
import pytest

def sample_graph(tmp_path):
    prefix=str(tmp_path/'historic-prefix')
    rows=np.zeros((2,26));rows[:,0]=[0,1];rows[:,1]=[10,11];rows[:,16]=-1;rows[:,8]=1;rows[:,12]=1
    graph=Path(prefix+'pose_graph.txt');np.savetxt(graph,rows)
    for index in range(2):
        Path(prefix+str(index)+'_briefdes.dat').write_bytes(b'original descriptors')
        Path(prefix+str(index)+'_keypoints.txt').write_text('1 2 3 4\n')
    return graph

def test_vins_relocalization_loads_an_owned_snapshot_without_overwriting_evidence(tmp_path):
    from vins_map import snapshot_vins_map
    graph=sample_graph(tmp_path);before=graph.read_bytes()
    result=snapshot_vins_map(graph,tmp_path/'owned-map')
    assert result['source_keyframes']==2
    assert (tmp_path/'owned-map'/'pose_graph.txt').read_bytes()==before
    assert (tmp_path/'owned-map'/'0_briefdes.dat').read_bytes()==b'original descriptors'
    assert graph.read_bytes()==before
    assert result['source_artifacts'][str(graph)]==hashlib.sha256(before).hexdigest()

def test_missing_vins_descriptor_is_rejected_before_native_map_loading(tmp_path):
    from vins_map import snapshot_vins_map
    graph=sample_graph(tmp_path)
    Path(str(graph).removesuffix('pose_graph.txt')+'1_keypoints.txt').unlink()
    with pytest.raises(ValueError,match='keyframe artifact'):
        snapshot_vins_map(graph,tmp_path/'owned-map')
