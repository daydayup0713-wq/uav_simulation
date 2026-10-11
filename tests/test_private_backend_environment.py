import json,subprocess
from pathlib import Path
import pytest

@pytest.mark.skipif(not Path('/opt/ros/humble/setup.bash').is_file() or
    not (Path(__file__).resolve().parents[1]/'ros2_ws/install/setup.bash').is_file(),
    reason='native ROS Humble underlay and built platform overlay required')
def test_private_backend_excludes_legacy_slam_underlay_and_resolves_its_own_core():
    result=subprocess.run(['./scripts/backend_env.sh','glim','/usr/bin/python3','-c',
        'import json,os;print(json.dumps({k:os.environ.get(k,"") for k in ("LD_LIBRARY_PATH","AMENT_PREFIX_PATH","CMAKE_PREFIX_PATH")}))'],
        capture_output=True,text=True,check=True)
    values=json.loads(result.stdout)
    legacy=str(Path('.deps/slam/install').resolve())
    for value in values.values():assert all(not entry.startswith(legacy) for entry in value.split(':'))
    assert values['LD_LIBRARY_PATH'].split(':')[0]==str(Path('.deps/backends/glim/install/lib').resolve())
