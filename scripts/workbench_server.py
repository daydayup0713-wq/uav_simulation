#!/usr/bin/python3
"""Persistent loopback UI/API; supervision and flight authority remain in ROS.

The API observes a bounded WebSocket stream and submits existing CLI/Action
clients. It creates no ROS publishers. Recorded playback reads files only.
"""
import argparse,asyncio,json,math,os,signal,subprocess,threading,time,uuid
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit,parse_qs
from workbench_contract import authorize,configuration
from workbench_reports import comparison,runs,recordings,metrics
from stop_lab import supervisor_identity

ORIGINS=('http://127.0.0.1:8780','http://localhost:8780')


def validate_http_origin(origin,content_type,length):
    if origin not in ORIGINS or content_type!='application/json' or not 0<length<=16384:
        raise ValueError('same loopback origin, JSON and bounded body required')
    return True


class Workbench:
    def __init__(self,root,watch=True):
        self.root=Path(root).resolve();self.lock=threading.RLock();self.stopped=threading.Event()
        self.mode='live';self.telemetry=None;self.received=-math.inf;self.replay=None
        self.jobs={};self.processes={};self.active_job=None;self.replay_lock=threading.Lock()
        self.catalog=[];self.catalog_checking=True
        self.selection={'localization':'glim','planning':'fast_planner','scene':'circle-eight','sensor_profile':None}
        try:self.selection=configuration(json.loads((self.root/'.runtime/workbench-selection.json').read_text()))
        except (OSError,ValueError):pass
        self.threads=[]
        if watch:
            for target in (self.watch,self.refresh_catalog):
                thread=threading.Thread(target=target,daemon=True);thread.start();self.threads.append(thread)

    def ingest(self,message,now):
        if not isinstance(message,dict) or message.get('kind')!='telemetry':return
        with self.lock:self.telemetry=message;self.received=now

    def watch(self):
        async def receive():
            from websockets.legacy.client import connect
            while not self.stopped.is_set():
                try:
                    async with connect('ws://127.0.0.1:8765',origin=ORIGINS[0],max_size=1800000,max_queue=1,open_timeout=2) as socket:
                        while not self.stopped.is_set():
                            try:message=await asyncio.wait_for(socket.recv(),1.)
                            except asyncio.TimeoutError:continue
                            if isinstance(message,str):self.ingest(json.loads(message),time.monotonic())
                except (OSError,ValueError,TimeoutError,Exception) as error:
                    # Observer disconnect changes freshness, never flight state.
                    if self.stopped.is_set():break
                    await asyncio.sleep(.5)
        asyncio.run(receive())

    def refresh_catalog(self):
        from uav_lab_experiments.registry import Registry
        while not self.stopped.is_set():
            try:catalog=Registry(self.root/'configs/backends.json',self.root/'.runtime/backend-evidence').list()
            except (OSError,ValueError):catalog=[]
            with self.lock:self.catalog=catalog;self.catalog_checking=False
            self.stopped.wait(60.)

    def active_run(self):
        try:
            path=Path((self.root/'.runtime/current-run').read_text().strip()).resolve()
            if path.parent!=self.root/'.runtime':return None
            manifest=json.loads((path/'manifest.json').read_text());pid=manifest['supervisor_pid']
            command=[v.decode() for v in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if v]
            cwd=Path(f'/proc/{pid}/cwd').resolve()
            if not supervisor_identity(command,cwd,self.root/'scripts/supervise.py'):return None
            return path,manifest
        except (OSError,ValueError,KeyError):return None

    def flight_state(self):
        with self.lock:
            active=self.active_run()
            if active and (self.telemetry or {}).get('run_id')!=active[0].name:
                return {'phase':'UNKNOWN','received_at':-math.inf}
            item=(self.telemetry or {}).get('diagnostics',{}).get('uav001/flight',{})
            age=item.get('age_s',math.inf)
            received=self.received-age if isinstance(age,(float,int)) and math.isfinite(age) and age>=0 else -math.inf
            return {**item.get('values',{}),'phase':item.get('message'),'received_at':received}

    def status(self):
        with self.lock:
            state=self.flight_state();now=time.monotonic();allowed=[]
            active=self.active_run()
            if active:
                for command in ('select','stop','arm','disarm','takeoff','goto','route','hold','land'):
                    try:authorize(command,state,now,self.mode);allowed.append(command)
                    except ValueError:pass
            public_state={**state,'received_at':state['received_at'] if math.isfinite(state['received_at']) else None}
            return {'mode':self.mode,'selection':self.selection,'flight':public_state,
                'connected':0<=now-self.received<=1.,'active_run':active[0].name if active else None,
                'allowed':allowed,'jobs':list(self.jobs.values())[-20:],
                'backends':self.catalog,'catalog_checking':self.catalog_checking,
                'replay':self.replay.info() if self.replay else None}

    def job(self,command,work,interrupt=False):
        with self.lock:
            if self.active_job and not interrupt:raise ValueError('another experiment operation is pending')
            if sum(item['state']=='RUNNING' for item in self.jobs.values())>=3:raise ValueError('operation limit reached')
            token=uuid.uuid4().hex;record={'id':token,'command':command,'state':'RUNNING','success':None,'reason':''}
            self.jobs[token]=record
            if not interrupt:self.active_job=token
            def run():
                try:
                    result=work(token)
                    with self.lock:record.update(state='SUCCEEDED',success=True,result=result)
                except (ValueError,RuntimeError,OSError,subprocess.TimeoutExpired) as error:
                    with self.lock:record.update(state='FAILED',success=False,reason=str(error)[:2000])
                finally:
                    with self.lock:
                        if self.active_job==token:self.active_job=None
                        self.processes.pop(token,None)
                        for key in list(self.jobs)[:-40]:
                            if self.jobs[key]['state']!='RUNNING':self.jobs.pop(key)
                    directory=self.root/'.runtime/workbench-jobs';directory.mkdir(parents=True,exist_ok=True)
                    with (directory/(token+'.json')).open('x') as stream:json.dump(record,stream,indent=2,allow_nan=False)
            threading.Thread(target=run,daemon=True).start()
            return dict(record)

    def cli(self,token,arguments,timeout,active):
        env=dict(os.environ);run,manifest=active
        env.update(LAB_RUN_DIR=str(run),LAB_DOMAIN_ID=str(manifest.get('environment',{}).get('ROS_DOMAIN_ID',42)),
            FASTRTPS_DEFAULT_PROFILES_FILE=str(run/'configuration/fastdds-local.xml'),ROS_LOCALHOST_ONLY='0')
        process=subprocess.Popen([str(self.root/'scripts/env.sh'),*map(str,arguments)],env=env,cwd=self.root,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
        with self.lock:self.processes[token]=process
        try:output,error=process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            try:output,error=process.communicate(timeout=10)
            except subprocess.TimeoutExpired:raise RuntimeError('operation unconfirmed; client cleanup pending; inspect flight state')
            raise RuntimeError('operation timeout; '+output[-2000:]+error[-1000:])
        if process.returncode:raise RuntimeError(output[-3000:]+error[-1000:])
        result=json.loads(output.splitlines()[-1])
        if result.get('success') is not True:raise RuntimeError(result.get('reason','command failed'))
        return result

    def start(self,token):
        from experiment_configuration import binding
        with self.lock:
            if self.mode!='live' or self.active_run():raise ValueError('live mode with no active lab required')
            selected=dict(self.selection)
        filename='sensors-'+selected['sensor_profile']+'.json' if selected['sensor_profile'] else 'navigation-sensors.json'
        calibration=json.loads((self.root/'configs'/filename).read_text())
        binding(self.root,selected['localization'],selected['planning'],calibration)
        directory=self.root/'.runtime/workbench-jobs';directory.mkdir(parents=True,exist_ok=True)
        log=(directory/(token+'.log')).open('w')
        args=[self.root/'scripts/start_lab.sh','--profile','navigation','--continuous','--headless','--web',
            '--rendering',selected.get('rendering','auto'),'--localization-backend',selected['localization'],'--planner-backend',selected['planning'],'--scene',selected['scene']]
        if selected['sensor_profile']:args+=['--sensor-profile',selected['sensor_profile']]
        process=subprocess.Popen(list(map(str,args)),cwd=self.root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        log.close()
        from supervise import owned_run_ready
        deadline=time.monotonic()+180
        while not owned_run_ready(self.root/'.runtime',process.pid):
            if process.poll() is not None:raise RuntimeError('supervised startup failed; see '+str(directory/(token+'.log')))
            if time.monotonic()>deadline:
                process.send_signal(signal.SIGTERM);process.wait(timeout=20);raise RuntimeError('supervised readiness timeout')
            time.sleep(.2)
        return {'success':True,'run_id':self.active_run()[0].name,'reason':'experiment ready; explicitly arm before takeoff'}

    def request(self,command,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        with self.lock:
            if command=='start':
                if self.mode!='live':raise ValueError('replay cannot start a live experiment')
                if self.active_run():raise ValueError('lab already active')
                return self.job(command,self.start)
            if command=='mode':
                if payload.get('mode') not in ('live','replay'):raise ValueError('unknown display mode')
                if self.active_job:raise ValueError('operation pending')
                if self.active_run():authorize('select',self.flight_state(),time.monotonic(),'live')
                self.mode=payload['mode'];self.replay=None
                return {'success':True,'mode':self.mode}
            if command=='replay':
                if self.mode!='replay':raise ValueError('replay mode required')
                identifier=payload.get('identifier')
                from workbench_contract import recording_path
                recording_path(self.root,identifier)
                def load(token):
                    from web_replay import RecordedView
                    replay=RecordedView(self.root,identifier)
                    with self.lock:
                        if self.mode!='replay':raise ValueError('replay mode changed while loading')
                        self.replay=replay
                    return replay.info()
                return self.job(command,load)
            if self.mode!='live':raise ValueError('replay mode cannot send flight or selection commands')
            active=self.active_run()
            if command=='select':
                if self.active_job:raise ValueError('operation pending')
                selected=configuration(payload)
                if active:authorize('select',self.flight_state(),time.monotonic(),self.mode)
                path=self.root/'.runtime/workbench-selection.json';path.parent.mkdir(parents=True,exist_ok=True)
                temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(selected,indent=2)+'\n');temporary.replace(path)
                self.selection=selected
                return {'success':True,'restart_required':bool(active),'selection':selected}
            authorize(command,self.flight_state(),time.monotonic(),self.mode)
            if not active:raise ValueError('no owned lab instance')
            if command=='stop':
                # Recheck exact captured PID immediately before signaling. Do not
                # resolve current-run again and accidentally stop a new session.
                if self.active_job:raise ValueError('operation pending')
                if self.active_run()!=active:raise ValueError('owned run identity changed')
                os.kill(active[1]['supervisor_pid'],signal.SIGTERM)
                return {'success':True,'reason':'owned supervisor shutdown requested'}
            args=['ros2','run','uav_lab_tools','labctl'];timeout=130.
            if command in ('arm','disarm','hold','land'):args+=[command]
            elif command=='takeoff':
                height=float(payload.get('height',2.))
                if not math.isfinite(height) or not .5<=height<=4.5:raise ValueError('bounded finite takeoff height required')
                args+=['takeoff','--height',str(height)]
            elif command=='goto':
                target=payload.get('target');yaw=float(payload.get('yaw',0.))
                if not isinstance(target,list) or len(target)!=3 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in target) or not math.isfinite(yaw):raise ValueError('finite ENU target required')
                args=['/usr/bin/python3',str(self.root/'scripts/workbench_goto.py'),
                    *map(str,target),'--yaw',str(yaw),'--timeout','120']
            elif command=='route':
                route=active[0]/'configuration/scene/route.json'
                if not route.is_file():raise ValueError('active experiment has no archived route')
                args=['/usr/bin/python3',str(self.root/'scripts/workbench_route.py'),str(route)];timeout=3315.
            else:raise ValueError('unknown command')
            def execute(token):
                with self.lock:
                    authorize(command,self.flight_state(),time.monotonic(),self.mode)
                    if self.active_run()!=active:raise ValueError('owned run changed before command')
                return self.cli(token,args,timeout,active)
            return self.job(command,execute,interrupt=command in ('hold','land'))

    def frame(self,index):
        with self.replay_lock:
            with self.lock:
                if self.mode!='replay' or self.replay is None:raise ValueError('recorded display not ready')
                replay=self.replay
            return replay.frame(index)

    def close(self):
        self.stopped.set()
        with self.lock:
            for token,process in self.processes.items():
                if process.poll() is None and self.jobs[token]['command']!='land':process.send_signal(signal.SIGINT)
        for thread in self.threads:thread.join(timeout=3)


class Server(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,app,port=8780):
        self.app=app;self.budget=threading.BoundedSemaphore(8)
        super().__init__(('127.0.0.1',port),Handler)
    def process_request(self,request,address):
        if not self.budget.acquire(blocking=False):self.shutdown_request(request);return
        try:super().process_request(request,address)
        except Exception:self.budget.release();raise
    def process_request_thread(self,request,address):
        try:super().process_request_thread(request,address)
        finally:self.budget.release()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(args[2].app.root/'web/dist'),**kwargs)
    def setup(self):
        super().setup();self.connection.settimeout(3.)
    def log_message(self,*args):pass
    def reply(self,value,status=200):
        body=json.dumps(value,allow_nan=False).encode();self.send_response(status)
        self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_GET(self):
        path=urlsplit(self.path);app=self.server.app
        try:
            if path.path=='/api/status':self.reply(app.status())
            elif path.path=='/api/comparison':self.reply(comparison(app.root))
            elif path.path=='/api/runs':self.reply(runs(app.root))
            elif path.path=='/api/recordings':self.reply(recordings(app.root))
            elif path.path=='/api/metrics':self.reply(metrics(app.root,parse_qs(path.query).get('run',[''])[0]))
            elif path.path=='/api/replay/frame':
                packet=app.frame(int(parse_qs(path.query).get('index',['-1'])[0]));self.send_response(200)
                self.send_header('Content-Type','application/octet-stream');self.send_header('Content-Length',str(len(packet)))
                self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(packet)
            elif path.path.startswith('/api/'):self.reply({'reason':'unknown API'},404)
            else:super().do_GET()
        except (OSError,ValueError,RuntimeError,KeyError) as error:self.reply({'success':False,'reason':str(error)},400)
    def do_POST(self):
        try:
            length=int(self.headers.get('Content-Length','0'))
            validate_http_origin(self.headers.get('Origin'),self.headers.get('Content-Type'),length)
            if self.headers.get('Host') not in ('127.0.0.1:8780','localhost:8780'):raise ValueError('loopback host required')
            path=urlsplit(self.path).path
            if not path.startswith('/api/'):raise ValueError('unknown API')
            value=self.server.app.request(path[5:],json.loads(self.rfile.read(length)))
            self.reply(value,202 if value.get('state')=='RUNNING' else 200)
        except (OSError,ValueError,RuntimeError,KeyError) as error:self.reply({'success':False,'reason':str(error)},400)


def main():
    root=Path(__file__).resolve().parents[1]
    if not (root/'web/dist/index.html').is_file():raise RuntimeError('build Web first: npm --prefix web run build')
    app=Workbench(root);server=None
    try:
        server=Server(app);print('WORKBENCH READY http://127.0.0.1:8780',flush=True)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        if server:server.server_close()
        app.close()


if __name__=='__main__':main()
