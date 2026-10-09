"""A global rigid correction must not jump the continuous control frame."""
from pathlib import Path
import subprocess
import pytest

def test_orb_control_pose_survives_global_correction_but_latches_map_reset_or_gap():
    directory=Path('.deps/backends/orb_slam3/install/lib/orb_slam3')
    if not (directory/'orb_slam3_node').exists():pytest.skip('private ORB core build absent')
    probe=directory/'orb_pose_probe'
    assert probe.is_file(),'build the native continuity probe'
    result=subprocess.run([str(probe.resolve())],capture_output=True,text=True,timeout=5)
    assert result.returncode==0,result.stdout+result.stderr
