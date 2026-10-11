from accept_rtk_faults import summarize
import json
from types import SimpleNamespace


def test_actual_ros_diagnostic_byte_level_is_serialized_with_failure_intact():
    import accept_rtk_faults
    item=SimpleNamespace(level=b'\x02',message='FAILED',values=[SimpleNamespace(key='reason',value='source stale')])
    assert hasattr(accept_rtk_faults,'status_record')
    result=accept_rtk_faults.status_record(item,112.)
    assert json.loads(json.dumps(result))=={'source':112.,'level':2,'message':'FAILED','values':{'reason':'source stale'}}


def test_abnormal_observations_must_be_withheld_and_recovery_must_be_observed():
    events=[{'kind':'gnss','source_s':t,'admitted':accepted} for t,accepted in [(36,True),(51,False),(81,False),(92,True),(100,False),(105,True)]]
    assert summarize(events)['passed']
    events[4]['admitted']=True
    assert not summarize(events)['passed']
    assert not summarize(events[:3])['passed']
