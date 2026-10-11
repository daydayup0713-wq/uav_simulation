#!/usr/bin/python3
"""Read-only hardware configuration review; creates no ROS context or motor command."""
import argparse,json,math,re
from pathlib import Path


def check(data):
    missing=[] if isinstance(data,dict) else ['configuration_object']
    data=data if isinstance(data,dict) else {}
    def obj(value):return value if isinstance(value,dict) else {}
    def require(key,predicate):
        try:valid=predicate(data.get(key))
        except (TypeError,ValueError,KeyError):valid=False
        if not valid:missing.append(key)
    def finite(value):return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    def vector(value):return isinstance(value,list) and len(value)==3 and all(finite(x) for x in value)
    def text(value):return isinstance(value,str) and bool(value.strip())
    require('simulation',lambda x:x is False)
    require('firmware_commit',lambda x:isinstance(x,str) and bool(re.fullmatch('[0-9a-f]{40}',x)))
    require('xrce_client_major',lambda x:type(x) is int and x==2)
    require('xrce_agent_version',lambda x:x=='2.4.3')
    require('transport',lambda x:text(x) and x.startswith(('serial:///dev/','udp://')))
    require('computer_arch',lambda x:x in ('aarch64','x86_64'))
    require('mass_kg',lambda x:finite(x) and x>0)
    require('body_size_m',lambda x:vector(x) and min(x)>0)
    require('inertia_kg_m2',lambda x:vector(x) and min(x)>0 and 2*max(x)<=sum(x))
    for key in ('calibration_id','calibration_source','clock_contract'):require(key,text)
    require('clock_uncertainty_ms',lambda x:finite(x) and x>=0)
    for name in ('lidar','imu'):
        sensor=obj(obj(data.get('sensor_extrinsics')).get(name))
        if not (sensor.get('parent')=='base_link' and sensor.get('child')==name+'_link'
                and vector(sensor.get('xyz')) and vector(sensor.get('rpy'))):missing.append('sensor_extrinsics.'+name)
        latency=obj(data.get('sensor_latency_ms')).get(name)
        if not finite(latency) or latency<0:missing.append('sensor_latency_ms.'+name)
    bounds=obj(data.get('geofence_enu_m'))
    if not (vector(bounds.get('lower')) and vector(bounds.get('upper'))
            and all(a<b for a,b in zip(bounds['lower'],bounds['upper']))):missing.append('geofence_enu_m')
    return {'ready':not missing,'missing':missing,'hardware_flight_qualified':False,'ros_publications':0,
        'scope':'Static configuration completeness only; firmware, timing, calibration and flight must be measured on the actual hardware'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('configuration',type=Path)
    args=parser.parse_args();report=check(json.loads(args.configuration.read_text()))
    print(json.dumps(report,ensure_ascii=False,indent=2));raise SystemExit(0 if report['ready'] else 1)


if __name__=='__main__':main()
