import math
import pytest
from uav_lab_bridge.coordinate import enu_to_ned, ned_to_enu, px4_to_ros_quaternion, ros_to_px4_quaternion, yaw_to_ned, step_toward
from uav_lab_bridge.clock import Px4Clock

def test_axes_and_yaw_are_converted_at_boundary():
    assert enu_to_ned((1, 2, 3)) == (2, 1, -3)
    assert ned_to_enu((2, 1, -3)) == (1, 2, 3)
    assert yaw_to_ned(0) == pytest.approx(math.pi / 2)
    assert yaw_to_ned(math.pi / 2) == pytest.approx(0)

def test_north_facing_level_px4_is_ros_yaw_90():
    assert px4_to_ros_quaternion((1, 0, 0, 0)) == pytest.approx((0, 0, math.sqrt(.5), math.sqrt(.5)))
    assert ros_to_px4_quaternion((0, 0, math.sqrt(.5), math.sqrt(.5))) == pytest.approx((1, 0, 0, 0))

def test_diagonal_step_has_bounded_norm():
    assert step_toward((0, 0, 0), (3, 4, 0), 1) == pytest.approx((.6, .8, 0))
    assert step_toward((0, 0, 0), (.1, 0, 0), 1) == (.1, 0, 0)

def test_firmware_time_not_ros_epoch_and_regression_is_rejected():
    clock = Px4Clock()
    with pytest.raises(RuntimeError):
        clock.timestamp(0)
    clock.observe(5_000_000, 10_000_000_000)
    assert clock.timestamp(10_050_000_000) == 5_050_000
    with pytest.raises(RuntimeError, match='regression'):
        clock.timestamp(9_000_000_000)
