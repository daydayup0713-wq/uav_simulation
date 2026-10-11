import numpy as np
from experiment_scenarios import definition


def test_indoor_worlds_have_physical_ceiling_to_measure_overhead_clearance():
    for scene in ('circle-eight','helix','multi-room','corridor','dense'):
        points,boxes,lower,upper=definition(scene)
        assert any(np.all(lo[:2]<=lower[:2]) and np.all(hi[:2]>=upper[:2]) and lo[2]>=5 and hi[2]>lo[2]
                   for lo,hi in boxes),scene
