import math
from test_controller import armed

def test_external_motion_to_target_cannot_jump_generated_setpoint():
    c = armed()
    token = c.fly('GOTO', 2.2, target=(3., 4., 2.), yaw=0.)
    for index in range(1, 121):
        now = 2.2+index*.05
        c.update(now, position=(3.,4.,2.), yaw=0., valid=True, armed=True, offboard=True, landed=False)
        previous = c.setpoint
        c.tick(now, .05)
        assert math.dist(previous, c.setpoint) <= .0500001
    assert c.results[token][0]
