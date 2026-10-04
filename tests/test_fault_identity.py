from verify_fault import component_identity


def test_fault_targets_only_the_selected_component_command():
    assert component_identity('lio', b'/usr/bin/python3\0/opt/ros/humble/bin/ros2\0run\0glim_ros\0glim_rosnode\0')
    assert not component_identity('lio', b'/usr/bin/python3\0/opt/ros/humble/bin/ros2\0run\0uav_lab_bridge\0bridge\0')
    assert not component_identity('lio', b'/bin/echo\0glim_ros\0glim_rosnode\0')
