"""Exercise Eigen values across the real core/adapter compilation ABI."""
from pathlib import Path
import shlex
import subprocess
import pytest


def test_orb_core_and_adapter_share_eigen_alignment_and_value_abi(tmp_path):
    build=Path('.deps/backends/build')
    core=build/'orb_core/CMakeFiles/ORB_SLAM3.dir/flags.make'
    adapter=build/'orb_adapter/CMakeFiles/orb_slam3_node.dir/flags.make'
    if not core.exists() or not adapter.exists():
        pytest.skip('private ORB builds required')
    def flags(path):
        line=next(row for row in path.read_text().splitlines() if row.startswith('CXX_FLAGS ='))
        return shlex.split(line.split('=',1)[1])
    library=tmp_path/'producer.cpp'
    library.write_text('''#include <Eigen/Core>
extern "C" unsigned alignment() {return alignof(Eigen::Matrix4f);}
extern "C" void fill(Eigen::Matrix4f& pose) {
  Eigen::Matrix4f value=Eigen::Matrix4f::Identity();value(0,3)=3.f;pose=value;
}
''')
    consumer=tmp_path/'consumer.cpp'
    consumer.write_text('''#include <Eigen/Core>
#include <cstdio>
extern "C" unsigned alignment();extern "C" void fill(Eigen::Matrix4f&);
int main() {
  if(alignment()!=alignof(Eigen::Matrix4f)) {
    std::fprintf(stderr,"Eigen ABI alignment: core=%u adapter=%zu\\n",alignment(),alignof(Eigen::Matrix4f));return 2;
  }
  Eigen::Matrix4f pose;fill(pose);
  return pose.allFinite() && pose(0,3)==3.f && pose(3,3)==1.f ? 0 : 3;
}
''')
    subprocess.run(['c++',*flags(core),'-shared','-fPIC','-I/usr/include/eigen3',
                    str(library),'-o',str(tmp_path/'libproducer.so')],check=True,capture_output=True)
    subprocess.run(['c++',*flags(adapter),'-I/usr/include/eigen3',str(consumer),
                    '-L'+str(tmp_path),'-lproducer','-Wl,-rpath,'+str(tmp_path),
                    '-o',str(tmp_path/'consumer')],check=True,capture_output=True)
    result=subprocess.run([str(tmp_path/'consumer')],capture_output=True,text=True,timeout=5)
    assert result.returncode==0,result.stdout+result.stderr
