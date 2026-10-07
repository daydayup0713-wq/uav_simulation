import numpy as np
from verify_navigation import clearance


def test_clearance_accounts_for_body_and_physical_wall():
    assert clearance([0,0,2],[1.3,-1,0],[1.7,1,4],[.4,.4,.3])==pytest.approx(.9)
    assert clearance([1.5,1.8,2],[1.3,-1,0],[1.7,1,4],[.4,.4,.3])==pytest.approx(.4)
    assert clearance([1.5,0,2],[1.3,-1,0],[1.7,1,4],[.4,.4,.3])<0
import pytest
