"""Recheck accepted coefficients rather than trust success flags or sampled plots."""
import numpy as np
from uav_lab_bridge.continuous_trajectory import Trajectory


def recheck(events):
    owners={};joins=0;errors=[];failed=[];maxima={'speed':0.,'acceleration':0.,'jerk':0.}
    for event in events:
        if event.get('event')!='trajectory_accepted':continue
        try:
            curve=Trajectory.from_dict(event['trajectory']);identifier=event['trajectory_id']
            values=curve.derivative_maxima()
            for key in maxima:maxima[key]=max(maxima[key],values[key])
            if any(values[k]>limit+1e-5 for k,limit in [('speed',.5),('acceleration',.5),('jerk',1.)]):
                failed.append('ANALYTIC_LIMITS')
            if identifier in owners:failed.append('DUPLICATE_OWNER')
            previous=event['replaces_id']
            if previous:
                old,started=owners[previous];elapsed=event['starts_at']-started
                if not 0<elapsed<old.duration:failed.append('JOIN_BEFORE_TERMINAL_REST')
                expected,actual=old.sample(elapsed),curve.sample(0.)
                error=max(float(np.max(np.abs(getattr(expected,k)-getattr(actual,k)))) for k in ('position','velocity','acceleration'))
                error=max(error,abs(expected.yaw-actual.yaw),abs(expected.yaw_rate-actual.yaw_rate))
                errors.append(error);joins+=1
                if error>1e-6:failed.append('C2_HANDOFF')
                if np.linalg.norm(expected.velocity)<1e-6:failed.append('INTERMEDIATE_REST')
            owners[identifier]=(curve,event['starts_at'])
        except (ValueError,KeyError,TypeError):failed.append('INVALID_ACCEPTED_CURVE')
    if not owners:failed.append('NO_ACCEPTED_CURVES')
    return {'passed':not failed,'failed_checks':sorted(set(failed)),'accepted_curves':len(owners),
        'joins':joins,'maximum_join_error':max(errors,default=0.),'analytic_maxima':maxima}
