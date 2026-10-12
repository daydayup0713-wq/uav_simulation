from ground_acceptance import verdict


def test_ground_acceptance_requires_full_duration_and_all_components():
    assert not verdict(29,30,{'flight':'READY','localization':'READY'},[])['success']
    assert not verdict(30,30,{'flight':'READY'},[])['success']
    assert verdict(30,30,dict.fromkeys(['flight','localization','navigation'],'READY'),[])['success']


def test_ground_failure_remains_failure_after_recovery():
    assert not verdict(30,30,dict.fromkeys(['flight','localization','navigation'],'READY'),['navigation FAILED'])['success']


def test_humble_byte_diagnostic_levels():
    from ground_acceptance import diagnostic_level
    assert diagnostic_level(b'\x00')==0
    assert diagnostic_level(b'\x02')==2
    assert diagnostic_level(2)==2
