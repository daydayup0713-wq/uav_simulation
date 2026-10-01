import json
from conftest import select_test_domain

def test_ros_checks_avoid_lab_domain_even_when_shell_does_not_inherit_it(tmp_path):
    run = tmp_path/'run'
    run.mkdir()
    (run/'ready').touch()
    (run/'manifest.json').write_text(json.dumps({'environment': {'ROS_DOMAIN_ID': '90'}}))
    assert select_test_domain(tmp_path, {'ROS_DOMAIN_ID': '42', 'LAB_DOMAIN_ID': '91'}) == 92
