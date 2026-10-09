"""Observed map transport and independent admission for native planner cores."""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import os
import signal
import subprocess
import time
import numpy as np
import psutil
from scipy.ndimage import distance_transform_edt
from uav_lab_bridge.continuous_trajectory import Trajectory,State,Limits
from .occupancy import vector
from .native_provenance import verified_build


def map_identity(collision):
    metadata={'frame':'odom','shape':list(collision.free.shape),
              'resolution':collision.resolution,'lower':collision.lower.tolist(),
              'version':collision.version,'source_stamp':collision.source_stamp}
    data=json.dumps(metadata,sort_keys=True,allow_nan=False).encode()+collision.free.astype('u1').tobytes()
    return hashlib.sha256(data).hexdigest()


def safe_corridors(collision,points):
    """Overlapping boxes certified entirely inside the inflated observed grid."""
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not 2<=len(points)<=128 or not np.isfinite(points).all():
        raise ValueError('finite bounded corridor guide required')
    boxes=[];shape=np.asarray(collision.free.shape)
    for a,b in zip(points,points[1:]):
        if not collision.segment_clear(a,b):raise ValueError('corridor guide blocked or unknown')
        steps=max(1,int(np.ceil(np.linalg.norm(b-a)/(collision.resolution*.2))))
        if steps>10000:raise ValueError('corridor sampling budget exceeded')
        for i in range(steps):
            pair=np.array([a+(b-a)*i/steps,a+(b-a)*(i+1)/steps])
            lo=np.minimum(*[collision.index(p) for p in pair]);hi=np.maximum(*[collision.index(p) for p in pair])
            def clear(l,h):
                return bool(np.all(l>=0) and np.all(h<shape) and
                    collision.free[tuple(slice(x,y+1) for x,y in zip(l,h))].all())
            if not clear(lo,hi):raise ValueError('no observed axis-aligned safe corridor')
            for _ in range(20):
                changed=False
                for axis in range(3):
                    l=lo.copy();l[axis]-=1
                    if clear(l,hi):lo=l;changed=True
                    h=hi.copy();h[axis]+=1
                    if clear(lo,h):hi=h;changed=True
                if not changed:break
            # Insets exclude an unknown cell at the upper closed box boundary.
            lower=collision.lower+lo*collision.resolution+1e-7
            upper=collision.lower+(hi+1)*collision.resolution-1e-7
            if np.any(pair<lower) or np.any(pair>upper):raise ValueError('corridor boundary not observed')
            if boxes and np.all(lower>=boxes[-1]['lower']) and np.all(upper<=boxes[-1]['upper']):continue
            boxes.append({'lower':lower.tolist(),'upper':upper.tolist()})
            if len(boxes)>256:raise ValueError('corridor count budget exceeded')
    return boxes


def write_request(directory,backend,collision,initial,goal,points,limits=Limits()):
    if backend not in ('ego','fast_planner','gcopter'):raise ValueError('unknown native planner')
    goal=vector(goal);limits.validate()
    if (collision.free.ndim!=3 or min(collision.free.shape)<2 or max(collision.free.shape)>512
            or np.prod(np.asarray(collision.free.shape,dtype='int64')+4)>2_000_000):
        raise ValueError('bounded three-dimensional map and search pool required')
    if not collision.point_clear(initial.position) or not collision.point_clear(goal):
        raise ValueError('start or goal blocked or unknown')
    p,v,a=map(vector,(initial.position,initial.velocity,initial.acceleration))
    if np.linalg.norm(v)>limits.speed or np.linalg.norm(a)>limits.acceleration:
        raise ValueError('initial derivative limit exceeded')
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    free=directory/'free.bin';collision.free.astype('u1').tofile(free)
    distance=directory/'distance.bin'
    # Include blocked exterior; the native field guides optimization only.
    # The original voxel mask independently admits the full returned curve.
    edt=distance_transform_edt(np.pad(collision.free,1,constant_values=False))[1:-1,1:-1,1:-1]
    np.maximum(0.,edt*collision.resolution-np.sqrt(3)*collision.resolution/2).astype('<f8').tofile(distance)
    request={'schema':1,'backend':backend,'frame':'odom','map_sha256':map_identity(collision),
             'shape':list(collision.free.shape),'lower':collision.lower.tolist(),
             'resolution':collision.resolution,'map_version':collision.version,'source_stamp':collision.source_stamp,
             'free_file':str(free),'distance_file':str(distance),'initial':{'position':p.tolist(),
             'velocity':v.tolist(),'acceleration':a.tolist()},'goal':goal.tolist(),
             'guide':np.asarray(points,dtype=float).tolist(),'limits':asdict(limits)}
    if backend=='gcopter':request['corridors']=safe_corridors(collision,points)
    (directory/'request.json').write_text(json.dumps(request,indent=2,allow_nan=False)+'\n')
    return request


def validate_curve(payload,collision,initial,goal,limits=Limits()):
    if payload.get('schema')!=1 or payload.get('success') is not True:
        raise ValueError('native planning failed: '+str(payload.get('reason','invalid result')))
    if payload.get('map_sha256')!=map_identity(collision):raise ValueError('native map identity mismatch')
    curve=Trajectory.from_dict(payload['trajectory'])
    if any(curve.limits.__dict__[key]>value for key,value in asdict(limits).items()):
        raise ValueError('native trajectory limit exceeds request')
    first=curve.sample(0.);last=curve.sample(curve.duration)
    if any(not np.allclose(getattr(first,key),getattr(initial,key),rtol=0,atol=1e-6)
           for key in ('position','velocity','acceleration')):
        raise ValueError('native initial p/v/a mismatch')
    if not np.allclose(last.position,vector(goal),rtol=0,atol=1e-6):raise ValueError('native goal mismatch')
    if not curve.collision_free(collision):raise ValueError('native curve collision or unknown space')
    return curve


def native_plan(root,backend,collision,initial,goal,directory,limits=Limits(),timeout=8.):
    """One verified core, one immutable map; no silent fallback to another method."""
    if backend not in ('ego','fast_planner','gcopter'):raise ValueError('unknown native planner')
    if not math.isfinite(timeout) or not 0<timeout<=30.:
        raise ValueError('positive finite native planner budget at most 30 seconds required')
    started=time.monotonic();directory=Path(directory).resolve()
    if directory.exists():raise ValueError('new planner evidence directory required')
    directory.mkdir(parents=True)
    result={'success':False,'reason':'','curve':None,'fallback':False,'core_peak_rss_bytes':0,'core_cpu_s':0.,
            'backend':backend,'map_sha256':map_identity(collision),'implementation_sha256':None,'core_source_ref':None}
    process=None
    try:
        build=verified_build(root,backend)
        result.update(core_source_ref=build['source_ref'],binary_sha256=build['binary_sha256'],
                      implementation_sha256=build['implementation_sha256'])
        from .planner import plan
        guide=plan(collision,initial.position,goal,timeout=min(2.,timeout))
        if not guide.success:raise ValueError(guide.reason)
        result['guide']=guide.points
        scale=1.
        for attempt in range(5):
            request=write_request(directory/('attempt-'+str(attempt)),backend,collision,initial,goal,guide.points,limits)
            request['time_scale']=scale;path=directory/('attempt-'+str(attempt))
            (path/'request.json').write_text(json.dumps(request,indent=2,allow_nan=False)+'\n')
            env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1',
                 'LD_LIBRARY_PATH':str(Path(root)/'.deps/planners/nlopt/install/lib')+':'+os.environ.get('LD_LIBRARY_PATH','')}
            remaining=timeout-(time.monotonic()-started)
            if remaining<=0:raise ValueError('native planner budget exceeded')
            with (path/'core.log').open('x') as log:
                process=subprocess.Popen([build['binary'],str(path/'request.json'),str(path/'result.json')],
                    env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                measured=psutil.Process(process.pid);attempt_cpu=0.
                while process.poll() is None:
                    try:
                        result['core_peak_rss_bytes']=max(result['core_peak_rss_bytes'],measured.memory_info().rss)
                        cpu=measured.cpu_times();attempt_cpu=max(attempt_cpu,cpu.user+cpu.system)
                    except psutil.NoSuchProcess:pass
                    if time.monotonic()-started>=timeout:raise ValueError('native planner budget exceeded')
                    time.sleep(.02)
                result['core_cpu_s']+=attempt_cpu
            payload=json.loads((path/'result.json').read_text())
            if process.returncode or payload.get('success') is not True:
                raise ValueError(payload.get('reason','native optimizer failed'))
            # Exact polynomial extrema decide retiming. Rerun with unchanged
            # initial p/v/a; scaling an in-flight curve would break continuity.
            from uav_lab_bridge.continuous_trajectory import Segment
            raw=Trajectory([Segment(s['duration'],s['coefficients']) for s in payload['trajectory']['segments']],limits)
            maxima=raw.derivative_maxima()
            ratio=max(maxima['speed']/limits.speed,np.sqrt(maxima['acceleration']/limits.acceleration),
                      (maxima['jerk']/limits.jerk)**(1/3))
            if ratio>1.+1e-7:
                scale*=max(1.1,ratio*1.1);continue
            result['curve']=validate_curve(payload,collision,initial,goal,limits)
            result.update(success=True,reason='native optimized and independently admitted',
                          attempt=attempt,method=payload.get('method'),source_result=str(path/'result.json'))
            break
        else:raise ValueError('native trajectory cannot meet derivative limits')
    except (ValueError,OSError,KeyError,TypeError) as error:result['reason']=str(error)
    finally:
        if process and process.poll() is None:
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                process.wait()
    result['runtime_s']=time.monotonic()-started
    saved={k:v for k,v in result.items() if k!='curve'}
    if result['curve'] is not None:saved['trajectory']=result['curve'].to_dict()
    (directory/'report.json').write_text(json.dumps(saved,indent=2,allow_nan=False)+'\n')
    return result


def with_heading(curve,initial,goal_yaw):
    """Compose one yaw quintic without altering the upstream position solution."""
    from numpy.polynomial import Polynomial
    from uav_lab_bridge.continuous_trajectory import quintic,Segment
    if not all(math.isfinite(x) for x in (initial.yaw,initial.yaw_rate,goal_yaw)):
        raise ValueError('finite heading state required')
    goal_yaw=initial.yaw+math.atan2(math.sin(goal_yaw-initial.yaw),math.cos(goal_yaw-initial.yaw))
    coeff=quintic(np.array([initial.yaw]),np.array([goal_yaw]),np.array([initial.yaw_rate]),
                  np.zeros(1),np.zeros(1),np.zeros(1),curve.duration)[0]
    offset=0.;segments=[]
    for segment in curve.segments:
        yaw=Polynomial(coeff)(Polynomial([offset/curve.duration,segment.duration/curve.duration])).coef
        segments.append(Segment(segment.duration,segment.coefficients.copy(),np.pad(yaw,(0,6-len(yaw)))))
        offset+=segment.duration
    return Trajectory(segments,curve.limits)


def plan_curve(root,backend,collision,initial,goal,directory,limits=Limits(),timeout=8.,goal_yaw=None):
    """Four complete pipelines; every returned curve passes the same admission."""
    if backend not in ('astar','ego','fast_planner','gcopter'):raise ValueError('unknown planner')
    if backend!='astar':
        result=native_plan(root,backend,collision,initial,goal,directory,limits,timeout)
    else:
        from .planner import plan
        from .native_provenance import fingerprint
        if not math.isfinite(timeout) or not 0<timeout<=30.:raise ValueError('invalid planner budget')
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=False);started=time.monotonic()
        result={'backend':backend,'map_sha256':map_identity(collision),'success':False,'curve':None,'fallback':False,
                'implementation_sha256':fingerprint(root,backend),'core_source_ref':'e295a54cd52e665edf8c06436d9fb195f462dc28',
                'method':'platform 26-neighbor A* + C2 minimum-jerk quintic'}
        try:
            guide=plan(collision,initial.position,goal,timeout=timeout)
            if not guide.success:raise ValueError(guide.reason)
            result['guide']=guide.points
            curve=Trajectory.generate(guide.points,initial=initial,limits=limits)
            if not curve.collision_free(collision):raise ValueError('continuous A* curve collision or unknown space')
            result.update(success=True,curve=curve,reason='continuous A* independently admitted')
        except ValueError as error:result['reason']=str(error)
        result['runtime_s']=time.monotonic()-started
    if result['success']:
        result['curve']=with_heading(result['curve'],initial,initial.yaw if goal_yaw is None else goal_yaw)
    saved={k:v for k,v in result.items() if k!='curve'}
    if result['curve'] is not None:saved['trajectory']=result['curve'].to_dict()
    (Path(directory)/'pipeline-report.json').write_text(json.dumps(saved,indent=2,allow_nan=False)+'\n')
    return result
