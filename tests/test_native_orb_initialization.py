"""Exercise the real upstream no-keyframe boundary, including orderly shutdown."""
import os
from pathlib import Path
import subprocess
import pytest


def test_native_orb_can_report_uninitialized_state_before_any_keyframe(tmp_path):
    from backend_configs import write_config
    import json
    root=Path('.').resolve();prefix=root/'.deps/backends/orb_slam3/install'
    binary=prefix/'lib/orb_slam3/orb_slam3_node'
    if not binary.exists():pytest.skip('native ORB build required')
    config=tmp_path/'configuration';write_config('orb_slam3',json.loads((root/'configs/sensors-livox.json').read_text()),config,'probe')
    env=dict(os.environ,ROS_DOMAIN_ID='79',ROS_LOG_DIR=str(tmp_path/'ros-logs'),
             LD_LIBRARY_PATH=str(prefix/'lib')+':'+os.environ.get('LD_LIBRARY_PATH',''))
    result=subprocess.run([str(binary),str(root/'.deps/orb_slam3/Vocabulary/ORBvoc.txt'),
                           str(config/'algorithm.yaml'),str(tmp_path),'--probe-initialization'],
                          env=env,capture_output=True,text=True,timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'metric_ready=false' in result.stdout
