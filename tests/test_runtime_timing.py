

def test_receipt_age_is_separate_from_source_age_and_actual_gap():
    from uav_lab_bridge.runtime_timing import RuntimeTiming
    trace=RuntimeTiming()
    trace.arrival('pose',10.,100.)
    trace.arrival('pose',10.1,100.2)
    value=trace.snapshot(100.7,10.15)['pose']
    assert abs(value['receipt_age_s']-.5)<1e-9
    assert abs(value['source_age_s']-.05)<1e-9
    assert abs(value['maximum_receipt_gap_s']-.2)<1e-9


def test_timing_keeps_bounded_channels_without_changing_readiness():
    from uav_lab_bridge.runtime_timing import RuntimeTiming
    trace=RuntimeTiming()
    for i in range(100):trace.arrival(str(i),float(i),float(i))
    assert len(trace.snapshot(101.,101.))<=16
    assert '99' in trace.snapshot(101.,101.)


def test_timing_reports_clock_stall_and_only_records_finite_values():
    from uav_lab_bridge.runtime_timing import RuntimeTiming
    trace=RuntimeTiming()
    trace.arrival('clock',1.,2.)
    trace.arrival('clock',1.,3.)
    assert trace.snapshot(3.,1.)['clock']['source_advance_s']==0.
    import pytest
    with pytest.raises(ValueError):trace.arrival('clock',float('nan'),4.)
