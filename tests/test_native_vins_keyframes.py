"""Exercise frame/time/feature-ID handoff using the compiled upstream core."""
import subprocess
from pathlib import Path
import pytest

def test_native_vins_keyframe_transport_preserves_estimator_contract():
    binary=Path('.deps/backends/vins_fusion/install/lib/vins_fusion/vins_hook_probe')
    if not Path('.deps/backends/vins_fusion/install/lib/vins_fusion/vins_fusion_node').exists():
        pytest.skip('private upstream VINS build absent')
    assert binary.is_file(), 'build the native estimator transport probe'
    result=subprocess.run([str(binary.resolve())],capture_output=True,text=True,timeout=10)
    assert result.returncode==0, result.stdout+result.stderr
