"""Sensor inspection, owned rosbag recording and isolated replay."""
import argparse
from datetime import datetime,timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
from contextlib import contextmanager
from .sensor_contract import SENSOR_TOPICS,RECORD_TOPICS,stamp_ns,record_topics
from .sensor_audit import SensorAudit
from .datasets import inspect_bag,load_dataset,file_hash,check_replay_domain

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    for cmd,default in (('sensors',10),('record',30)):
        s=sub.add_parser(cmd);s.add_argument('--duration',type=float,default=default)
    sub.add_parser('bag-check').add_argument('dataset',type=Path)
    play=sub.add_parser('replay');play.add_argument('dataset',type=Path);play.add_argument('--domain',type=int,default=77)
    play.add_argument('--rate',type=float,default=1.,help='wall playback speed; sensor source timestamps are preserved')
    return p

def runtime_manifest(root, ready=False):
    runtime=root/'.runtime'
    run=Path((runtime/'current-run').read_text().strip())
    if run.parent!=runtime:raise ValueError('invalid current run path')
    manifest=json.loads((run/'manifest.json').read_text())
    if manifest.get('profile') not in ('sensors','localization','slam','navigation'):
        raise ValueError('start a sensors, localization, slam or navigation lab profile first')
    if ready and not (run/'ready').exists():raise ValueError('sensor lab is not ready')
    if int(manifest['environment']['ROS_DOMAIN_ID'])!=int(os.environ.get('ROS_DOMAIN_ID','42')):
        raise ValueError('operator ROS domain differs from active lab')
    return run,manifest

class LiveAudit:
    def __init__(self,calibration):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        self.node=rclpy.create_node('sensor_audit_'+uuid.uuid4().hex[:8])
        self.audit=SensorAudit(calibration);self.clock=None
        from .sensor_contract import sensor_topics
        for topic,(typename,_,_) in sensor_topics(calibration).items():
            self.node.create_subscription(get_message(typename),topic,lambda msg,t=topic:self.audit.observe(t,msg,self.clock),qos_profile_sensor_data)
        self.node.create_subscription(get_message('rosgraph_msgs/msg/Clock'),'/clock',self.on_clock,qos_profile_sensor_data)
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(get_message('tf2_msgs/msg/TFMessage'),'/tf_static',self.audit.observe_static,qos)

    def on_clock(self,msg):
        self.clock=stamp_ns(msg.clock);self.audit.observe_clock(self.clock)

    def spin(self):
        import rclpy
        rclpy.spin_once(self.node,timeout_sec=.05)

    def collect(self,duration,process=None):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if process and process.poll() is not None:raise RuntimeError('recorder exited early: '+str(process.returncode))
            self.spin()
        return self.audit.report(live=True)

    def close(self):self.node.destroy_node()

def stop_owned(process):
    def send(sig):
        try:os.killpg(process.pid,sig)
        except ProcessLookupError:pass
    def group_exists():
        try:os.killpg(process.pid,0);return True
        except ProcessLookupError:return False
    # The ros2 wrapper can exit before its recorder/player descendants.
    # Group ownership, not leader poll(), governs all cleanup signals.
    send(signal.SIGINT)
    try:process.wait(timeout=15)
    except subprocess.TimeoutExpired:pass
    send(signal.SIGTERM)
    deadline=time.monotonic()+3
    while group_exists() and time.monotonic()<deadline:
        process.poll();time.sleep(.05)
    send(signal.SIGKILL)
    process.wait(timeout=3)

@contextmanager
def scoped_signals():
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    received=False
    def interrupt(signum,_):
        nonlocal received
        if not received:
            received=True
            raise KeyboardInterrupt(signal.Signals(signum).name)
    for sig in previous:signal.signal(sig,interrupt)
    try:yield
    finally:
        for sig,handler in previous.items():signal.signal(sig,handler)

def playback_timeout(report,rate=1.):
    return max(30,report['bag_duration_s']/rate*1.2+15)

def record(root, duration):
    run,manifest=runtime_manifest(root,ready=True)
    destination=root/'recordings'/(run.name+'-'+datetime.now(timezone.utc).strftime('%H%M%S')+'-'+uuid.uuid4().hex[:6])
    destination.mkdir(parents=True)
    shutil.copytree(run/'configuration',destination/'configuration')
    shutil.copy2(run/'manifest.json',destination/'run-manifest.json')
    metadata={'schema_version':1,'complete':False,'run_id':run.name,'calibration':manifest['calibration'],
              'requested_duration_s':duration,'record_topics':list(RECORD_TOPICS),
              'configuration_sha256':{str(p.relative_to(destination/'configuration')):file_hash(p) for p in (destination/'configuration').rglob('*') if p.is_file()}}
    metadata_path=destination/'dataset.json'
    def save():metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
    save()
    process=None;monitor=None;interrupted=False
    try:
        monitor=LiveAudit(manifest['calibration'])
        with (destination/'recorder.log').open('w') as log:
            process=subprocess.Popen(['ros2','bag','record','--storage','sqlite3','-o',str(destination/'bag'),
                 '--qos-profile-overrides-path',str(destination/'configuration/recording-qos.yaml'),*record_topics(manifest['calibration'])],
                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            metadata['recorder_pid']=process.pid;save()
            live_report=monitor.collect(duration,process)
            (destination/'live-audit.json').write_text(json.dumps(live_report,indent=2)+'\n')
            if not live_report['passed']:raise RuntimeError('live recording sensor contract failed; see live-audit.json')
            stop_owned(process)
        if process.returncode not in (0,-signal.SIGINT):raise RuntimeError('recorder failed: '+str(process.returncode))
        report,_=inspect_bag(destination,allow_incomplete=True)
        (destination/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
        metadata['complete']=report['passed']
        metadata['reason']='passed' if report['passed'] else 'recorded sensor contract failed; see audit.json'
        metadata['bag_sha256']={p.name:file_hash(p) for p in (destination/'bag').iterdir() if p.is_file()}
        save()
        return {'passed':metadata['complete'],'dataset':str(destination),'reason':metadata['reason'],'audit':report}
    except BaseException as exc:
        metadata['complete']=False
        interrupted=isinstance(exc,KeyboardInterrupt)
        metadata['reason']='recording interrupted' if interrupted else str(exc);save()
        if interrupted:raise RuntimeError('recording interrupted; incomplete dataset: '+str(destination)) from exc
        raise
    finally:
        if process:stop_owned(process)
        if monitor:monitor.close()

def active_domains(root):
    domains=[]
    for path in (root/'.runtime').glob('*/manifest.json'):
        if (path.parent/'ready').exists():domains.append(int(json.loads(path.read_text())['environment']['ROS_DOMAIN_ID']))
    return domains

def replay(root,options):
    import rclpy
    check_replay_domain(options.domain,active_domains(root))
    report,types=inspect_bag(options.dataset)
    if not report['passed']:return report
    monitor=LiveAudit(load_dataset(options.dataset)['calibration']);process=None
    try:
        # Discover existing publishers before starting playback. ROS rosout is harmless;
        # every actual data publisher or PX4 endpoint makes this domain unavailable.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:monitor.spin()
        occupied=[t for t,_ in monitor.node.get_topic_names_and_types()
                  if (t in record_topics(monitor.audit.calibration) or t.startswith('/fmu/')) and monitor.node.count_publishers(t)]
        if occupied:raise ValueError('replay domain already has data/control publishers: '+', '.join(occupied))
        with (options.dataset/'replay.log').open('w') as log:
            process=subprocess.Popen(['ros2','bag','play',str(options.dataset/'bag'),
                   '--rate',str(options.rate),
                   '--qos-profile-overrides-path',str(options.dataset/'configuration/recording-qos.yaml'),
                   '--topics',*sorted(types)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+playback_timeout(report,options.rate)
            while process.poll() is None:
                if time.monotonic()>deadline:raise RuntimeError('replay timeout')
                monitor.spin()
            # Drain final queued samples.
            end=time.monotonic()+.5
            while time.monotonic()<end:monitor.spin()
        result=monitor.audit.report(live=True)
        result.update(domain=options.domain,player_exit_code=process.returncode,dataset=str(options.dataset),
                      playback_rate=options.rate,scope='data integrity; does not grant realtime qualification')
        result['passed']=result['passed'] and process.returncode==0
        (options.dataset/'replay-audit.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
    finally:
        if process:stop_owned(process)
        monitor.close()

def main(args=None):
    with scoped_signals():return run_cli(args)

def run_cli(args=None):
    options=parser().parse_args(args)
    root=Path(os.environ.get('LAB_ROOT','.')).resolve()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    initialized=False;monitor=None
    try:
        if options.command in ('record','sensors') and (not math.isfinite(options.duration) or not 3<=options.duration<=3600):
            raise ValueError('--duration must be between 3 and 3600 seconds')
        if options.command=='replay' and (not math.isfinite(options.rate) or not .1<=options.rate<=4):
            raise ValueError('--rate must be finite and between 0.1 and 4')
        if options.command=='bag-check':result,_=inspect_bag(options.dataset)
        else:
            if options.command=='replay':
                check_replay_domain(options.domain,active_domains(root))
                os.environ['ROS_DOMAIN_ID']=str(options.domain)
                calibration=load_dataset(options.dataset)['calibration']
                if calibration.get('sensor_profile'):
                    archived=options.dataset/'configuration/fastdds-local.xml'
                    profile=archived if archived.exists() else root/'configs/fastdds-local.xml'
                    os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(profile.resolve()),ROS_LOCALHOST_ONLY='0')
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO);initialized=True
            if options.command=='sensors':
                run,manifest=runtime_manifest(root)
                monitor=LiveAudit(manifest['calibration']);result=monitor.collect(options.duration)
                (run/'sensor-audit.json').write_text(json.dumps(result,indent=2)+'\n')
            elif options.command=='record':result=record(root,options.duration)
            else:result=replay(root,options)
        print(json.dumps(result,ensure_ascii=False),flush=True)
        return 0 if result['passed'] else 1
    except (OSError,ValueError,RuntimeError,KeyError,KeyboardInterrupt) as exc:
        print(json.dumps({'passed':False,'reason':str(exc) or 'interrupted'},ensure_ascii=False),flush=True);return 1
    finally:
        if monitor:monitor.close()
        if initialized and rclpy.ok():rclpy.shutdown()
