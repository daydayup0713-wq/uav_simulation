"""Live Web requests use fresh explicit state; file replay has no command path."""
import math,re
from pathlib import Path

SCENES=('circle-eight','helix','multi-room','corridor','dense','outdoor-rtk')
LOCALIZERS=('glim','fast_lio2','fast_livo2','fast_livo2_rtk','orb_slam3','lio_sam','vins_fusion')
PLANNERS=('astar','ego','fast_planner','gcopter')


def authorize(command,state,now,mode):
    if mode!='live':raise ValueError('replay mode cannot send flight or selection commands')
    received=state.get('received_at',-math.inf)
    if not math.isfinite(now) or not math.isfinite(received) or not 0<=now-received<=1.:
        raise ValueError('fresh flight state required')
    if any(state.get(key) not in ('True','False') for key in ('armed','landed','offboard')):
        raise ValueError('unknown flight state flags prohibit commands')
    armed,landed,offboard=[state.get(key)=='True' for key in ('armed','landed','offboard')]
    if command=='stop':
        if armed or not landed or state.get('phase') not in ('READY','FAILSAFE'):
            raise ValueError('disarmed landed stable ground state required')
    elif command=='select':
        if armed or not landed or state.get('phase')!='READY':raise ValueError('idle disarmed landed ground state required')
    elif command=='arm':
        if armed or not landed or state.get('phase')!='READY':raise ValueError('idle disarmed ground state required')
    elif command=='disarm':
        if not landed or state.get('phase') not in ('READY','HOLDING'):raise ValueError('idle landed ground state required')
    elif command in ('takeoff','goto','route'):
        if not armed or not offboard:raise ValueError('explicit armed Offboard state required')
        if state.get('phase')!='HOLDING':raise ValueError('idle hover required; duplicate motion rejected')
        if command=='takeoff' and not landed:raise ValueError('takeoff requires explicit ground arm')
        if command!='takeoff' and landed:raise ValueError('airborne hover required')
    elif command=='hold':
        if not armed or not offboard or state.get('phase') not in ('HOLDING','MOVING'):raise ValueError('armed Offboard hover or motion required')
    elif command=='land':
        if not armed or state.get('phase')=='LANDING':raise ValueError('armed vehicle outside existing landing required')
    else:raise ValueError('unknown Web command')
    return True


def configuration(value):
    if not isinstance(value,dict) or set(value)-{'localization','planning','scene','sensor_profile','rendering'}:
        raise ValueError('only explicit backend/scene/sensor configuration allowed')
    if value.get('localization') not in LOCALIZERS or value.get('planning') not in PLANNERS:raise ValueError('unknown backend')
    if value.get('scene') not in SCENES:raise ValueError('unknown scene')
    if value.get('sensor_profile') not in (None,'livox','livox-rtk','mechanical'):raise ValueError('unknown sensor profile')
    if 'rendering' in value and value['rendering'] not in ('auto','mesa-display'):raise ValueError('unknown rendering mode')
    return {key:value.get(key) for key in ('localization','planning','scene','sensor_profile')} | ({'rendering':value['rendering']} if 'rendering' in value else {})


def recording_path(root,identifier):
    if not isinstance(identifier,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',identifier):raise ValueError('invalid recording identifier')
    root=Path(root).resolve();path=(root/'recordings'/identifier).resolve()
    if path.parent!=root/'recordings':raise ValueError('recording identifier escapes owned root')
    return path
