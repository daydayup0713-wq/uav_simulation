#!/usr/bin/python3
"""Local simulator RTK loss/recovery/outlier contract and real upstream batch test."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
from supervise import owned_run_ready
from experiment_configuration import prepare_backend


def status_record(item,source):
    level=int.from_bytes(item.level,'little') if isinstance(item.level,bytes) else int(item.level)
    return {'source':source,'level':level,'message':item.message,'values':{v.key:v.value for v in item.values}}


def summarize(events):
    fixes=[e for e in events if e.get('kind')=='gnss']
    def all_between(start,end,admitted):
        rows=[e for e in fixes if start<=e['source_s']<end]
        return bool(rows) and all(e['admitted'] is admitted for e in rows)
    checks={'fixed_admitted':all_between(35,45,True),'lost_withheld':all_between(50,80,False),
        'float_withheld':all_between(80,90,False),'recovery_admitted':all_between(91,99,True),
        'outliers_rejected':all_between(100,102,False),'post_outlier_recovery':all_between(104,110,True)}
    return {'passed':all(checks.values()),'checks':checks,'observations':len(fixes),
        'accepted':sum(e['admitted'] for e in fixes),'rejected':sum(not e['admitted'] for e in fixes)}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rendering',choices=('auto','mesa-display'),default='auto')
    args=parser.parse_args();root=Path(os.environ['LAB_ROOT']);output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    supervisor=None;children=[];handles=[];core=None;node=None;reason='';run=None;poses=[];status=[];completed=False
    def close(process):
        if process is None:return
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
    def launch(command,name,stdin=None):
        log=(output/(name+'.log')).open('w');handles.append(log)
        return subprocess.Popen([str(v) for v in command],stdout=log,stderr=subprocess.STDOUT,
                                start_new_session=True,stdin=stdin)
    try:
        supervisor=launch([root/'scripts/start_lab.sh','--profile','sensors','--sensor-profile','livox-rtk',
            '--scene','outdoor-rtk','--headless','--rendering',args.rendering],'supervisor')
        deadline=time.monotonic()+150
        while not owned_run_ready(root/'.runtime',supervisor.pid):
            if supervisor.poll() is not None or time.monotonic()>deadline:raise RuntimeError('owned RTK sensor startup failed')
            time.sleep(.2)
        run=Path((root/'.runtime/current-run').read_text().strip());manifest=json.loads((run/'manifest.json').read_text())
        os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(run/'configuration/fastdds-local.xml'),ROS_LOCALHOST_ONLY='0')
        commands=prepare_backend(root,manifest['calibration'],output/'selected-localization',output.name,{'localization':'fast_livo2_rtk'})
        import rclpy
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init();node=rclpy.create_node('rtk_fault_contract_observer');clock=[None]
        def pose(message):
            p=message.pose.pose.position;poses.append([message.header.stamp.sec+message.header.stamp.nanosec/1e9,p.x,p.y,p.z])
        def quality(message):
            for item in message.status:
                if item.name=='uav001/localization':status.append(status_record(item,clock[0]))
        node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec/1e9),10)
        node.create_subscription(Odometry,'/uav001/backends/fast_livo2_rtk/raw_odometry',pose,qos_profile_sensor_data)
        node.create_subscription(DiagnosticArray,'/uav001/localization/diagnostics',quality,10)
        for name,command in commands:
            process=launch(command,name,subprocess.PIPE if name=='backend-core-0' else None)
            children.append(process)
            if name=='backend-core-0':core=process
        deadline=time.monotonic()+160
        while clock[0] is None or clock[0]<112:
            rclpy.spin_once(node,timeout_sec=.02)
            if any(p.poll() is not None for p in children) or supervisor.poll() is not None:raise RuntimeError('owned RTK component exited')
            if time.monotonic()>deadline:raise RuntimeError('RTK abnormal source interval not completed')
        # Finish source generation first; posterior optimization is explicitly
        # offline and has no feedback connection to the stock ground PX4.
        supervisor.send_signal(signal.SIGTERM);supervisor.wait(timeout=20)
        core.stdin.write(b'\n');core.stdin.flush()
        deadline=time.monotonic()+180
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if '[Offline Optimization] Finished.' in (output/'backend-core-0.log').read_text():completed=True;break
            if core.poll() is not None:break
    except (RuntimeError,OSError,KeyboardInterrupt,subprocess.TimeoutExpired) as error:reason=str(error)
    finally:
        for process in reversed(children):close(process)
        close(supervisor)
        for handle in handles:handle.close()
        if node:node.destroy_node()
        import rclpy
        if rclpy.ok():rclpy.shutdown()
    events_path=output/'selected-localization/inputs.jsonl'
    events=[json.loads(line) for line in events_path.read_text().splitlines()] if events_path.exists() else []
    admission=summarize(events)
    rows=np.asarray(poses);jumps=np.linalg.norm(np.diff(rows[:,1:],axis=0),axis=1) if len(rows)>1 else []
    report={'passed':not reason and admission['passed'] and completed,'reason':reason,'run_id':run.name if run else None,
        'scope':'actual locally simulated GNSS/RTK + real FAST-LIVO2-RTK frontend and end-of-sequence posterior; ground test, no control qualification',
        'admission':admission,'posterior_completed':completed,'local_pose_count':len(poses),
        'local_max_pose_step_m':float(max(jumps)) if len(jumps) else None,
        'localization_watchdog_failed':any(item['message']=='FAILED' for item in status),
        'flight_qualified':False,'status':status,'poses':poses}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('poses','status')}));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
