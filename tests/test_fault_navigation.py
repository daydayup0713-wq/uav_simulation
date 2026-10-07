from verify_fault import component_identity


def test_navigation_fault_only_targets_owned_ros_command():
    assert component_identity('navigation',b'/usr/bin/python3\0/opt/ros/humble/bin/ros2\0run\0uav_lab_navigation\0navigation\0')
    assert not component_identity('navigation',b'/bin/echo\0uav_lab_navigation\0navigation\0')
