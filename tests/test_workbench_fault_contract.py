import importlib,importlib.util


def api():
    assert importlib.util.find_spec('fault_scenario'),'workbench fault runner missing'
    return importlib.import_module('fault_scenario')


def test_only_exact_owned_core_or_ros_web_command_can_be_signaled():
    binary='/repo/.deps/backends/glim/install/lib/glim_ros/glim_rosnode'
    assert api().command_matches('backend-core-0',binary.encode()+b'\0--ros-args\0',binary)
    assert not api().command_matches('backend-core-0',b'/bin/echo\0'+binary.encode()+b'\0',binary)
    assert api().command_matches('web-observatory',b'/usr/bin/python3\0/opt/ros/humble/bin/ros2\0run\0uav_lab_experiments\0observatory\0')
    assert not api().command_matches('web-observatory',b'/bin/echo\0observatory\0')
