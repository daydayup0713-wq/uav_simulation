#!/usr/bin/python3
"""Owned live sensor flight for localization validation; estimator does not control PX4."""
import argparse,json,os,subprocess,time,shutil,sys
from pathlib import Path
import numpy as np
import psutil,rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from supervise import owned_run_ready
from replay_backend import PoseObserver,stop_owned
from backend_configs import write_config
from backend_launch import core_command
from backend_provenance import save_provenance
from bootstrap_backends import verify_sources
from uav_lab_tools.datasets import file_hash
from uav_lab_experiments.backend_contract import trajectory_report
from uav_lab_experiments.benchmark_report import resource_summary

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['fast_lio2','fast_livo2'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    if output.exists():parser.error('new output required; evidence is immutable')
    output.mkdir(parents=True);(output/'ros-logs').mkdir()
    lock=json.loads((ROOT/'dependencies/backends.lock.json').read_text())
    verify_sources(ROOT,lock,lock['backends'][args.backend]['repositories'])
    provenance=save_provenance(ROOT,lock,args.backend,output,sys.argv)
    prefix=ROOT/'.deps/backends'/args.backend/'install'
    build=json.loads((prefix/(args.backend+'-build-manifest.json')).read_text())
    binary=Path(build['binary']).resolve()
    if not binary.is_relative_to(prefix.resolve()) or file_hash(binary)!=build['binary_sha256']:raise ValueError('private binary identity changed')
    (output/'build-manifest.json').write_text(json.dumps(build,indent=2)+'\n')
    env=dict(os.environ,ROS_LOG_DIR=str(output/'ros-logs'));processes=[];logs=[];supervisor=node=None
    truth=[];resources=[];commands=[];reason='';run=None;started=time.monotonic();last_resource=0.

    def launch(command,name,algorithm=True):
        log=(output/(name+'.log')).open('w');logs.append(log)
        process=subprocess.Popen([str(x) for x in command],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        if algorithm:processes.append(process)
        return process

    def observe_truth(msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        truth.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])

    def spin(guard=True):
        nonlocal last_resource
        rclpy.spin_once(node,timeout_sec=.01)
        if guard:
            if supervisor.poll() is not None:raise RuntimeError('owned simulation exited')
            if any(p.poll() is not None for p in processes):raise RuntimeError('owned localization dependency exited')
            if any(event['signature'][0]=='FAILED' for event in node.normalizer_events):raise RuntimeError('live localization watchdog failed')
        if time.monotonic()-last_resource>=.5:
            last_resource=time.monotonic();samples=[]
            for process in processes:
                try:
                    p=psutil.Process(process.pid);cpu=p.cpu_times()
                    samples.append({'pid':process.pid,'rss':p.memory_info().rss,'cpu_s':cpu.user+cpu.system})
                except psutil.NoSuchProcess:pass
            resources.append({'wall_s':last_resource-started,'source_s':node.clock,'phase':'live','processes':samples})

    def command(arguments,guard=True):
        process=launch([ROOT/'scripts/labctl',*arguments],'command-'+str(len(commands)),False)
        deadline=time.monotonic()+300
        try:
            while process.poll() is None:
                spin(guard)
                if time.monotonic()>deadline:raise RuntimeError('flight command timeout')
            commands.append({'arguments':arguments,'exit_code':process.returncode})
            if process.returncode:raise RuntimeError('flight command rejected: '+' '.join(arguments))
        finally:stop_owned(process)

    try:
        supervisor=launch([ROOT/'scripts/start_lab.sh','--profile','sensors','--sensor-profile','livox','--scene','circle-eight','--headless'],'supervisor',False)
        deadline=time.monotonic()+150
        while not owned_run_ready(ROOT/'.runtime',supervisor.pid):
            if supervisor.poll() is not None or time.monotonic()>deadline:raise RuntimeError('owned simulation startup failed')
            time.sleep(.2)
        run=Path((ROOT/'.runtime/current-run').read_text().strip())
        metadata=json.loads((run/'manifest.json').read_text());calibration=metadata['calibration']
        if 'gnss' in calibration:raise ValueError('no-GNSS localization validation requires Livox-only sensors')
        env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(run/'configuration/fastdds-local.xml');env['ROS_LOCALHOST_ONLY']='0'
        os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=env['FASTRTPS_DEFAULT_PROFILES_FILE'],ROS_LOCALHOST_ONLY='0')
        shutil.copy2(run/'manifest.json',output/'simulation-manifest.json')
        (output/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
        write_config(args.backend,calibration,output/'configuration',output.name)
        rclpy.init();node=PoseObserver(args.backend,calibration,output)
        node.create_subscription(Odometry,'/uav001/ground_truth/odometry',observe_truth,qos_profile_sensor_data)
        if args.backend=='fast_livo2':
            launch(['/opt/ros/humble/lib/demo_nodes_cpp/parameter_blackboard','--ros-args','--params-file',output/'configuration/camera.yaml'],'camera-parameters')
        for module,name in [('algorithm_sensor_node','input-adapter'),('backend_node','normalizer')]:
            launch(['/usr/bin/python3','-m','uav_lab_experiments.'+module,'--ros-args','-p','backend:='+args.backend,
                    '-p','calibration:='+str(output/'calibration.json')],name)
        launch(core_command(args.backend,binary,output/'configuration'),'backend')
        deadline=time.monotonic()+45
        while not node.normalizer_events or node.normalizer_events[-1]['signature'][0]!='READY':
            spin()
            if time.monotonic()>deadline:raise RuntimeError('localization did not reach READY')
        # Inspect the actual graph, not only our requested configuration.
        subscriptions=[]
        for name,namespace in node.get_node_names_and_namespaces():
            if name in ('laser_mapping','laserMapping','fast_livo','fastlivo_mapping'):
                subscriptions.extend(t for t,_ in node.get_subscriber_names_and_types_by_node(name,namespace))
        (output/'algorithm-subscriptions.json').write_text(json.dumps(subscriptions,indent=2)+'\n')
        if not subscriptions or any('gnss' in t or 'ground_truth' in t or 'sensor_physics' in t for t in subscriptions):
            raise RuntimeError('algorithm input isolation was not established')
        for arguments in [['arm'],['takeoff','--height','2'],['experiments','route',str(run/'configuration/scene/route.json')],['land']]:command(arguments)
    except (ValueError,RuntimeError,KeyboardInterrupt) as failure:
        reason=str(failure) or 'interrupted'
        if node and supervisor and supervisor.poll() is None:
            try:command(['land'],False)
            except (ValueError,RuntimeError):pass
    finally:
        for process in reversed(processes):stop_owned(process)
        stop_owned(supervisor)
        for log in logs:log.close()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    np.savetxt(output/'truth.tum',truth,fmt='%.9f')
    expected=[t for t in node.expected if t>=node.expected[0]+3] if node and node.expected else []
    quality=trajectory_report(node.rows if node else [],np.asarray(truth),expected)
    latency=np.asarray(node.latencies if node else [])
    report={'schema':1,'backend':args.backend,'success':not reason and quality['passed'] and not node.failures if node else False,
            'error':reason,'quality':quality,'normalizer_events':node.normalizer_events if node else [],
            'normalized_output_count':node.normalized if node else 0,'input_counts':node.counts if node else {},
            'invalid_poses':node.failures if node else [],'resources':resources,'resource_summary':resource_summary(resources),
            'latency_source_s':{'p50':float(np.median(latency)),'p95':float(np.quantile(latency,.95))} if len(latency) else None,
            'implementation_sha256':provenance['implementation_sha256'],'run_id':run.name if run else None,'commands':commands,
            'scope':'real-time localization from measured Livox/IMU/camera; no GNSS or truth input. Stock PX4 controls flight; no new backend closed-loop qualification.'}
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='resources'},indent=2))
    return 0 if report['success'] else 1


if __name__=='__main__':raise SystemExit(main())
