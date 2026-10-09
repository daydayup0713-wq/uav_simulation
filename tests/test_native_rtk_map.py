"""Exercise the actual RTK export boundary with system PCL, including empty frames."""
import shutil,subprocess
from pathlib import Path
import pytest


def test_rtk_map_export_preserves_pose_only_frames_without_pcl_division_by_zero(tmp_path):
    include=Path('.deps/fast_livo2_rtk_ros2/src/fast_livo/include').resolve()
    if not (include/'platform_cloud_transform.hpp').is_file() or not Path('/usr/include/pcl-1.12').is_dir():
        pytest.skip('build pinned RTK backend and install PCL first')
    binary=tmp_path/'rtk-map-export'
    subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/pcl-1.12','-I/usr/include/eigen3','-I'+str(include),
                    'tests/native/test_rtk_map_cloud.cpp','-lpcl_common','-o',str(binary)],check=True)
    completed=subprocess.run([str(binary)],capture_output=True,text=True)
    assert completed.returncode==0,completed.stderr
