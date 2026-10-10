#!/usr/bin/python3
"""Independent actual-flight fault observation inside a supervisor-owned run."""
import argparse,asyncio,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
from verify_fault import landing_completed

CASES=('hold','land','localization_exit','source_stale','web_disconnect')


def command_matches(component,command,binary=None):
    tokens=command.split(b'\0')
    if component=='backend-core-0':return binary is not None and tokens[0]==str(binary).encode()
    if component=='web-observatory':
        return any(tokens[i:i+4]==[b'/opt/ros/humble/bin/ros2',b'run',b'uav_lab_experiments',b'observatory']
                   for i in range(len(tokens)))
    return False


async def disconnect_browser():
    from websockets.legacy.client import connect
    async with connect('ws://127.0.0.1:8765',origin='http://127.0.0.1:8080',max_size=4_000_000) as connection:
        packet=await asyncio.wait_for(connection.recv(),5.)
        return {'received_packet_bytes':len(packet),'client_closed':True}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--case',choices=CASES,required=True)
    parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=Path(os.environ['LAB_ROOT']);run=args.run.resolve()
    if run.parent!=root/'.runtime' or not (run/'ready').exists():raise RuntimeError('owned ready run required')
    manifest=json.loads((run/'manifest.json').read_text())
    if manifest['run_id']!=run.name:raise RuntimeError('run identity mismatch')
    os.kill(manifest['supervisor_pid'],0)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleStatus,VehicleLandDetected
    from diagnostic_msgs.msg import DiagnosticArray
    from uav_lab_interfaces.msg import TrajectoryReference
    rclpy.init();node=rclpy.create_node('workbench_fault_observer')
    flight={};vehicle={'armed':None,'landed':None,'offboard':None};received={};references=[];events=[]
    landing=[False];started=[None];motion=None;stopped_pid=None;commands=[];passed=False;reason='';extra={}
    def state(message):
        vehicle.update(armed=message.arming_state==VehicleStatus.ARMING_STATE_ARMED,
            offboard=message.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        received['status']=time.monotonic()
        if started[0] is not None and vehicle['armed'] and message.nav_state==VehicleStatus.NAVIGATION_STATE_AUTO_LAND:landing[0]=True
        events.append({'time':time.monotonic(),'nav_state':message.nav_state,**vehicle})
    def land(message):vehicle['landed']=bool(message.landed);received['land']=time.monotonic()
    def diagnostic(message):
        for status in message.status:
            if status.name=='uav001/flight':
                flight.clear();flight.update(phase=status.message,**{v.key:v.value for v in status.values})
                received['flight']=time.monotonic()
    def reference(message):
        references.append({'time':time.monotonic(),'id':message.trajectory_id,'velocity':list(message.velocity),
            'acceleration':list(message.acceleration),'position':list(message.position)})
    node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',state,qos_profile_sensor_data)
    node.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',land,qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray,'/uav001/diagnostics',diagnostic,10)
    node.create_subscription(TrajectoryReference,'/uav001/trajectory/reference',reference,20)
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    def command(arguments,timeout=90,required=True):
        result=subprocess.run([str(root/'scripts/labctl'),*arguments],capture_output=True,text=True,timeout=timeout)
        commands.append({'arguments':arguments,'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr})
        if required and result.returncode:raise RuntimeError(result.stdout+result.stderr)
        return result
    def signal_component(component,signum):
        pid=json.loads((run/'processes.json').read_text())[component]
        build=json.loads((run/'configuration/selected-localization/build-manifest.json').read_text())
        commandline=Path(f'/proc/{pid}/cmdline').read_bytes()
        if os.getpgid(pid)!=pid or not command_matches(component,commandline,build['binary']):
            raise RuntimeError('component process identity mismatch')
        os.killpg(pid,signum);return pid
    try:
        command(['arm']);command(['takeoff','--height','2'])
        with (args.output.parent/'motion.log').open('w') as log:
            target=['0','2.5','2'] if args.case=='web_disconnect' else ['2.5','3.6','2']
            motion=subprocess.Popen([str(root/'scripts/labctl'),'nav','goto',*target],stdout=log,stderr=subprocess.STDOUT)
        if not wait(lambda:flight.get('phase')=='MOVING' and references and np.linalg.norm(references[-1]['velocity'])>.05,25.):
            raise RuntimeError('fault requires confirmed continuous airborne movement')
        started[0]=time.monotonic()
        if args.case=='hold':
            command(['hold'])
            if not wait(lambda:flight.get('phase')=='HOLDING' and references and np.linalg.norm(references[-1]['velocity'])<1e-6,15.):
                raise RuntimeError('constrained hold completion unconfirmed')
            motion.wait(timeout=10)
            if motion.returncode==0:raise RuntimeError('interrupted navigation claimed success')
            extra['constrained_stop_confirmed']=True;command(['land'])
        elif args.case=='land':command(['land'])
        elif args.case in ('localization_exit','source_stale'):
            signum=signal.SIGTERM if args.case=='localization_exit' else signal.SIGSTOP
            pid=signal_component('backend-core-0',signum)
            if signum==signal.SIGSTOP:stopped_pid=pid
        elif args.case=='web_disconnect':
            extra['browser']=asyncio.run(disconnect_browser())
            signal_component('web-observatory',signal.SIGTERM)
            motion.wait(timeout=65)
            if motion.returncode:raise RuntimeError('Web isolation flight interrupted')
            if not wait(lambda:flight.get('phase')=='HOLDING' and vehicle['armed'] and vehicle['offboard'],3.):
                raise RuntimeError('Web disconnect did not preserve flight')
            extra['web_process_exit_flight_continued']=True;command(['land'])
        if args.case in ('localization_exit','source_stale'):
            if not wait(lambda:landing_completed(vehicle,all(time.monotonic()-received.get(k,0)<2 for k in ('status','land')),landing[0]),50.):
                raise RuntimeError('independent failsafe landing/disarm not confirmed')
            extra['failsafe_landing_observed']=True
            if stopped_pid is not None:
                os.killpg(stopped_pid,signal.SIGCONT);stopped_pid=None
                wait(lambda:False,2.)
                extra['source_process_resumed']=True
            rejection=command(['arm'],required=False)
            if rejection.returncode==0:raise RuntimeError('source recovery allowed automatic control resumption')
            extra['recovery_arm_rejected']=True
        else:
            if not wait(lambda:vehicle['landed'] is True and vehicle['armed'] is False,5.):
                raise RuntimeError('landing/disarm state unconfirmed')
        if motion:
            motion.wait(timeout=10)
            if args.case!='web_disconnect' and motion.returncode==0:raise RuntimeError('faulted navigation claimed success')
        passed=True
    except (RuntimeError,OSError,subprocess.TimeoutExpired,KeyboardInterrupt) as error:
        reason=str(error)
        try:command(['land'],timeout=75,required=False)
        except (RuntimeError,OSError,subprocess.TimeoutExpired):pass
    finally:
        if stopped_pid is not None:
            try:os.killpg(stopped_pid,signal.SIGCONT)
            except ProcessLookupError:pass
        if motion and motion.poll() is None:
            motion.terminate()
            try:motion.wait(timeout=8)
            except subprocess.TimeoutExpired:motion.kill();motion.wait()
        node.destroy_node();rclpy.shutdown()
    report={'passed':passed,'reason':reason,'case':args.case,'run_id':run.name,
        'final':vehicle,'commands':commands,'events':events,'references':references,
        'fault_started_at':started[0],'motion_returncode':motion.returncode if motion else None,**extra}
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('events','references','commands')}));return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
