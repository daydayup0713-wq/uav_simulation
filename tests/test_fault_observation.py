from verify_fault import landing_completed

def test_disarmed_idle_offboard_label_does_not_invalidate_observed_failsafe_landing():
    final = {'armed': False, 'landed': True, 'offboard': True}
    assert landing_completed(final, fresh=True, landing_observed=True)
    assert not landing_completed(final, fresh=True, landing_observed=False)
    assert not landing_completed(final, fresh=False, landing_observed=True)
    assert not landing_completed({**final, 'armed': True}, fresh=True, landing_observed=True)
