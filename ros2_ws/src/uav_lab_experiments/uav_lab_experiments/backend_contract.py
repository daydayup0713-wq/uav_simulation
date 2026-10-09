"""Shared sensor-only replay contract and independent quality assessment."""
import numpy as np
from scipy.spatial.transform import Rotation
from .trajectory_evaluation import evaluate


def input_topics(backend):
    topics=['/clock','/uav001/lidar/points','/uav001/imu/data']
    if backend in ('fast_livo2','fast_livo2_rtk'):
        topics+=['/uav001/camera/image_raw','/uav001/camera/camera_info']
    if backend=='fast_livo2_rtk':topics+=['/uav001/gnss/fix']
    return topics


def normalized_pose(stamp,position,quaternion,imu):
    p,q=np.asarray(position,dtype=float),np.asarray(quaternion,dtype=float)
    if not np.isfinite(stamp) or stamp<=0 or p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3:
        raise ValueError('invalid source pose')
    rwb=Rotation.from_quat(q)*Rotation.from_euler('xyz',imu['rpy']).inv()
    body=p-rwb.apply(imu['xyz'])
    orientation=rwb.as_quat()
    if orientation[3]<0:orientation=-orientation
    return [float(stamp),*body.tolist(),*orientation.tolist()]


def trajectory_report(estimate,truth,expected,attitude_valid=True):
    if len(estimate)<3:return {'passed':False,'reason':'no valid trajectory','metrics':None}
    truth=np.asarray(truth,dtype=float)
    if truth.ndim!=2 or truth.shape[1]!=8 or len(truth)<2 or not np.isfinite(truth).all():
        return {'passed':False,'reason':'no valid independent reference','metrics':None}
    estimate=np.asarray(estimate)
    supported=estimate[(estimate[:,0]>=truth[0,0])&(estimate[:,0]<=truth[-1,0])]
    expected=np.asarray(expected)
    expected_supported=expected[(expected>=truth[0,0])&(expected<=truth[-1,0])]
    if not len(expected_supported):return {'passed':False,'reason':'no expected samples within truth support','metrics':None}
    try:metrics=evaluate(supported,truth,expected_supported,attitude_reference=attitude_valid)
    except ValueError as error:return {'passed':False,'reason':str(error),'metrics':None}
    if not attitude_valid:
        metrics['attitude_rmse_deg']=metrics['rpe_rotation_rmse_deg']=None
    passed=metrics['ate_rmse_m']<=.3 and metrics['coverage']>=.95
    if attitude_valid:passed=passed and metrics['attitude_rmse_deg']<=5.
    return {'passed':bool(passed),'reason':'quality thresholds passed' if passed else 'quality thresholds failed',
            'outside_truth_support':len(estimate)-len(supported),
            'expected_outside_truth_support':len(expected)-len(expected_supported),'metrics':metrics}
