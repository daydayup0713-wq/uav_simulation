from supervise import FlightReadiness


def test_vision_startup_gates_fusion_while_arm_still_checks_actual_offboard_preflight():
    status = {'fresh':'True','armed':'False','landed':'True','preflight':'False','external_ready':'True'}
    vision = FlightReadiness(settle=0, require_preflight=False)
    assert vision.update(status, 1)
    assert not vision.update({**status, 'external_ready':'False'}, 2)
    assert not FlightReadiness(settle=0).update(status, 1)
