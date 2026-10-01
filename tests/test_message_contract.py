from check_messages import MESSAGES, check_messages

def test_schema_check_ignores_comments_but_detects_version_change(tmp_path):
    firmware = tmp_path/'.deps/px4/msg/versioned'
    ros = tmp_path/'ros2_ws/src/px4_msgs/msg'
    firmware.mkdir(parents=True)
    ros.mkdir(parents=True)
    for name in MESSAGES:
        (firmware/(name+'.msg')).write_text('uint32 MESSAGE_VERSION = 1 # firmware\nfloat32 x\n')
        (ros/(name+'.msg')).write_text('# ROS\nuint32 MESSAGE_VERSION = 1\nfloat32 x # position\n')
    assert all(check_messages(tmp_path).values())
    (ros/'VehicleStatus.msg').write_text('uint32 MESSAGE_VERSION = 2\nfloat32 x\n')
    assert not check_messages(tmp_path)['VehicleStatus']
