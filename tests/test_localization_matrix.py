import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from accept_localization_matrix import run_matrix


def test_failed_algorithm_is_preserved_and_does_not_skip_the_next_comparison(tmp_path):
    calls=[]
    def replay(command,**kwargs):
        name=command[1];calls.append(name)
        directory=Path(command[command.index('--output')+1]);directory.mkdir()
        (directory/'report.json').write_text(json.dumps({'backend':name,'success':name=='glim',
            'input_group':'synchronous_lidar_imu','dataset_sha256':{'bag.db3':'a'*64},
            'quality':{'passed':name=='glim'},'implementation_sha256':'v1'}))
        return SimpleNamespace(returncode=0 if name=='glim' else 1)
    jobs=[{'backend':'fast_lio2','dataset':'input'},{'backend':'glim','dataset':'input'}]
    result=run_matrix(jobs,tmp_path/'matrix',{'fast_lio2':'v1','glim':'v1'},replay)
    assert calls==['fast_lio2','glim']
    assert result['complete'] and result['quality_passed']==1 and result['quality_failed']==1
    runs=result['comparison']['groups'][0]['cases'][0]['runs']
    assert {r['backend']:r['success'] for r in runs}=={'fast_lio2':False,'glim':True}
    assert result['qualification']=='none'
    with pytest.raises(ValueError,match='already exists'):
        run_matrix(jobs,tmp_path/'matrix',{},replay)
