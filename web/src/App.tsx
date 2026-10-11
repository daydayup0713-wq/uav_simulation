import {useEffect,useRef,useState} from 'react';
import {Controls,layerNames} from './Controls';
import type {Layers,Layer,Coloring} from './Controls';
import {Scene} from './Scene';
import {decodeCloud} from './protocol';
import type {Cloud,Telemetry,Backend} from './protocol';
import {api,Comparison,Configuration,FlightControls,MetricCurve} from './Workbench';
import type {WorkbenchStatus,ComparisonData,Selection,RecordedRun,Recording,CurveData} from './Workbench';
import './style.css';

const initialLayers:Layers={raw:true,registered:false,global:false,voxels:false};
export default function App(){
 const [connected,setConnected]=useState(false),[clouds,setClouds]=useState<Partial<Record<Layer,Cloud>>>({});
 const [telemetry,setTelemetry]=useState<Telemetry|null>(null),[backends,setBackends]=useState<Backend[]>([]);
 const [layers,setLayers]=useState(initialLayers),[color,setColor]=useState<Coloring>('height');
 const [follow,setFollow]=useState(false),[reset,setReset]=useState(0),[paths,setPaths]=useState<Record<string,number[][]>>({});
 const [error,setError]=useState(''),[draft,setDraft]=useState<Selection>({localization:'glim',planning:'fast_planner',scene:'circle-eight',sensor_profile:null});
 const [tab,setTab]=useState('空间观测'),[workbench,setWorkbench]=useState<WorkbenchStatus|null>(null),[apiAvailable,setApiAvailable]=useState(false);
 const [comparison,setComparison]=useState<ComparisonData|null>(null),[runs,setRuns]=useState<RecordedRun[]>([]),[recordings,setRecordings]=useState<Recording[]>([]);
 const [curve,setCurve]=useState<CurveData|null>(null),[replayIndex,setReplayIndex]=useState(0),[playing,setPlaying]=useState(false),[recording,setRecording]=useState('');
 const configured=useRef(false),mode=workbench?.mode??'live';
 const telemetryReceived=useRef(-Infinity),[wallNow,setWallNow]=useState(Date.now());
 const [replayCloud,setReplayCloud]=useState<Cloud|null>(null);
 useEffect(()=>{const timer=setInterval(()=>setWallNow(Date.now()),200);return()=>clearInterval(timer);},[]);
 useEffect(()=>{
  let closed=false,pending=false;
  const poll=async()=>{if(pending)return;pending=true;try{const state=await api<WorkbenchStatus>('status');if(!closed){setWorkbench(state);setApiAvailable(true);if(!configured.current){setDraft(state.selection);configured.current=true;}}}
   catch{if(!closed)setApiAvailable(false);}finally{pending=false;}};
  void poll();const timer=setInterval(poll,700);return()=>{closed=true;clearInterval(timer);};
 },[]);
 useEffect(()=>{
  if(tab==='空间观测'&&mode!=='replay')return;
  let closed=false,pending=false;
  const update=async()=>{
   if(pending)return;pending=true;
   try{
    const reads:Promise<void>[]=[];
    if(tab==='算法对照')reads.push(api<ComparisonData>('comparison').then(value=>{if(!closed)setComparison(value);}));
    if(tab==='实验记录')reads.push(api<RecordedRun[]>('runs').then(value=>{if(!closed)setRuns(value);}));
    if(mode==='replay')reads.push(api<Recording[]>('recordings').then(value=>{if(!closed)setRecordings(value);}));
    await Promise.all(reads);
   }catch{}finally{pending=false;}
  };
  void update();const timer=setInterval(update,15000);
  return()=>{closed=true;clearInterval(timer);};
 },[tab,mode]);
 const command=async(name:string,payload:unknown={})=>{try{await api(name,payload);setError('');if(name==='mode'){setPlaying(false);setReplayCloud(null);setReplayIndex(0);setClouds({});setPaths({});setTelemetry(null);}}
  catch(e){setError(e instanceof Error?e.message:'实验操作失败');}};
 useEffect(()=>{
  let socket:WebSocket|null=null,timer=0,closed=false;
  function open(){if(closed||mode!=='live')return;socket=new WebSocket(`ws://${location.hostname}:8765`);socket.binaryType='arraybuffer';
   socket.onopen=()=>{setConnected(true);setError('');};socket.onerror=()=>setError('观测桥未连接');
   socket.onclose=()=>{setConnected(false);if(!closed)timer=window.setTimeout(open,1000);};
   socket.onmessage=event=>{try{
    if(event.data instanceof ArrayBuffer){const cloud=decodeCloud(event.data);setClouds(previous=>({...previous,[cloud.layer]:cloud}));}
    else{const message=JSON.parse(event.data);
     if(message.kind==='telemetry'){telemetryReceived.current=Date.now();setWallNow(Date.now());setTelemetry(message);}
     else if(message.kind==='registry')setBackends(message.backends);
     else if(message.kind==='path')setPaths(previous=>({...previous,[message.layer]:message.points}));
    }
   }catch(e){setError(e instanceof Error?e.message:'观测数据错误');}};
  }open();return()=>{closed=true;clearTimeout(timer);setConnected(false);socket?.close();};
 },[mode]);
 useEffect(()=>{setPlaying(false);setReplayIndex(0);setReplayCloud(null);},[workbench?.replay?.identifier]);
 useEffect(()=>{
  if(mode!=='replay'||!workbench?.replay)return;
  const controller=new AbortController();let current=true;
  fetch('/api/replay/frame?index='+replayIndex,{signal:controller.signal,cache:'no-store'}).then(async response=>{if(!response.ok)throw Error((await response.json()).reason);return response.arrayBuffer();})
   .then(buffer=>{if(current)setReplayCloud(decodeCloud(buffer));}).catch(e=>{if(current&&e.name!=='AbortError')setError(e.message);});
  return()=>{current=false;controller.abort();};
 },[mode,replayIndex,workbench?.replay?.identifier]);
 useEffect(()=>{if(!playing||mode!=='replay'||!workbench?.replay)return;const info=workbench.replay;
  const timer=setInterval(()=>setReplayIndex(index=>{if(index>=info.frames-1){setPlaying(false);return index;}return index+1;}),200);return()=>clearInterval(timer);
 },[playing,mode,workbench?.replay?.identifier]);
 const displayClouds=mode==='replay'?(replayCloud?{raw:replayCloud}:{}):clouds;
 const displayPaths=mode==='replay'?{actual:workbench?.replay?.actual_path??[]}:paths;
 const observedFresh=connected&&wallNow-telemetryReceived.current<=1000;
 const vehicle=mode==='live'&&observedFresh?(telemetry?.vehicle??null):null,flight=mode==='live'?telemetry?.diagnostics['uav001/flight']:null;
 const armed=flight?.values.armed==='True',active=(Object.keys(layers) as Layer[]).filter(key=>layers[key]);
 const count=active.reduce((total,key)=>total+(displayClouds[key]?.count??0),0);
 const phase=mode==='replay'?'文件回放':!connected?'观测已断开':!observedFresh&&telemetry?'遥测已过期':flight?.message??'等待状态';
 const cloudAge=mode==='live'&&connected&&clouds.raw&&telemetry?Math.max(0,telemetry.simulation_time-clouds.raw.stamp):null;
 return <div className="workbench">
  <header><div className="brand-mark"><span/><span/><span/><span/></div><div><h1>UAV <b>LAB</b></h1><p>无人机算法实验台</p></div>
   <nav>{['空间观测','算法对照','实验记录'].map(name=><button className={tab===name?'nav-current':''} key={name} onClick={()=>setTab(name)}>{name}</button>)}</nav>
   <div className={connected?'connection live':'connection'}><i/>{connected?'观测桥已连接':'等待观测桥'}</div><span className="vehicle-tag">uav001</span>
  </header>
  <aside className="left-panel">
   <div className="workspace-label">实验工作区 <small>L1</small></div>
   <Controls layers={layers} onLayer={(key,value)=>setLayers(previous=>({...previous,[key]:value}))} color={color} onColor={setColor}
    follow={follow} onCamera={action=>action==='reset'?(setFollow(false),setReset(reset+1)):setFollow(!follow)}/>
   <div className="divider"/><Configuration status={workbench} available={apiAvailable&&(!workbench?.active_run||connected)} draft={draft} onDraft={setDraft} onCommand={(name,payload)=>void command(name,payload)}/>
   {!apiAvailable&&<p className="fine-print">实验控制 API 未连接。运行 ./scripts/workbench.sh 后访问本地 8780 端口。</p>}
   <div className="divider"/><div className="section-label">图例 <span>LEGEND</span></div>
   <div className="legend"><span><i className="planned-line"/>计划路径</span><span><i className="actual-line"/>实际轨迹</span></div>
   <p className="fine-print">ENU 坐标 · 地面网格 1m<br/>鼠标左键旋转 · 右键平移 · 滚轮缩放</p>
  </aside>
  <main>{tab==='算法对照'?<Comparison data={comparison}/>:tab==='实验记录'?<div className="report-view"><span className="eyebrow">TRACEABLE RUNS</span><h2>实验记录</h2><p className="report-note">包括成功与失败运行；选择记录查看原始跟踪指标。</p>
   <div className="run-list">{runs.map(run=><button key={run.id} onClick={()=>void api<CurveData>('metrics?run='+encodeURIComponent(run.id)).then(setCurve).catch(e=>setError(e.message))}><span className={run.passed?'pass':'fail'}>{run.passed?'通过':'失败'}</span><b>{run.id}</b><small>{run.reason||run.source}</small></button>)}</div><MetricCurve data={curve}/></div>:<>
   <div className="view-heading"><div><span className="eyebrow">{mode==='replay'?'RECORDED / READ ONLY':'LIVE SPATIAL OBSERVATORY'}</span><h2>三维点云观察</h2></div>
   <span className="view-frame">ENU / odom</span></div>
   <div className="display-mode"><button disabled={!apiAvailable||mode==='live'} onClick={()=>void command('mode',{mode:'live'})}>实时观测</button><button disabled={!apiAvailable||mode==='replay'} onClick={()=>void command('mode',{mode:'replay'})}>文件回放</button><span>{mode==='replay'?'历史画面 · 飞行控制禁用 · 不发布 ROS 数据':'原始点云与录制保留完整数据'}</span></div>
   {mode==='replay'&&<div className="replay-controls"><select aria-label="录制数据" value={recording} onChange={e=>setRecording(e.target.value)}><option value="">选择本机录制</option>{recordings.map(r=><option key={r.id} value={r.id}>{r.id}</option>)}</select>
    <button disabled={!recording||!apiAvailable} onClick={()=>void command('replay',{identifier:recording})}>载入</button>
    <button disabled={!workbench?.replay} onClick={()=>setPlaying(!playing)}>{playing?'暂停':'播放'}</button>
    {workbench?.replay&&<><input aria-label="回放进度" type="range" min="0" max={workbench.replay.frames-1} value={replayIndex} onChange={e=>setReplayIndex(Number(e.target.value))}/><span>{workbench.replay.frame_times_s[replayIndex]?.toFixed(1)} / {workbench.replay.duration_s.toFixed(1)} s · 5Hz</span></>}</div>}
   {mode==='replay'&&!!workbench?.replay?.unshown_source_frames&&<p className="fine-print">{workbench.replay.unshown_source_frames} 帧缺少位姿时间支持，未显示；完整原始录制保留。</p>}
   <div className="viewport"><Scene clouds={displayClouds} layers={layers} color={color} vehicle={vehicle} paths={displayPaths} follow={follow} reset={reset}/>
    {!count&&<div className="empty-view"><span className="empty-symbol">⠿</span><h3>等待点云数据</h3><p>启动传感器实验后，原始扫描会显示在这里。</p></div>}
    <div className="view-status"><span><i/>{count.toLocaleString()} 显示点</span><span>{mode==='replay'?'文件回放 · 5Hz':connected?'5Hz 显示上限':'缓存画面 · 观测已断开'}</span><span>{follow?'跟随视角':'自由视角'}</span></div>
    <div className="axis-tag"><b>X</b> 东 <b>Y</b> 北 <b>Z</b> 上</div>
   </div>
   <div className="pose-strip">{['X · 东','Y · 北','Z · 高度'].map((label,i)=><div key={label}><span>{label}</span><strong>{vehicle?vehicle.position[i].toFixed(2):'—'}<small>m</small></strong></div>)}
    <div><span>点云源延迟</span><strong>{cloudAge!==null?Math.round(cloudAge*1000):'—'}<small>ms</small></strong></div></div>
   <div className="sources"><div className="section-label">图层数据 <span>SOURCE DATA</span></div>{(Object.keys(layerNames) as Layer[]).map(key=><div className="source-row" key={key}>
    <i className={displayClouds[key]?'available':''}/><span>{layerNames[key]}</span><b>{displayClouds[key]?`${displayClouds[key]!.count.toLocaleString()} / ${displayClouds[key]!.source_count.toLocaleString()}`:'等待数据'}</b>
    <small>{mode==='replay'?'录制数据 / 仅原始图层':!connected?'缓存数据':telemetry?.layer_errors[key]??(clouds[key]?.display_decimated?'显示降采样 / 有限窗口':clouds[key]?.invalid_sample_points?'已过滤无效返回':clouds[key]?'完整单帧显示':'未收到话题')}</small></div>)}</div></>}
  </main>
  <aside className="right-panel"><div className="section-label">飞行状态 <span>TELEMETRY</span></div>
   <div className="state-card"><span className="state-caption">当前阶段</span><h3>{phase}</h3><div className="state-meta"><span>{mode==='replay'?'历史数据':!observedFresh?'解锁状态未知':armed?'已解锁':'未解锁'}</span><span>{vehicle&&vehicle.age_s<1?'遥测新鲜':'等待遥测'}</span></div></div>
   <div className="divider"/><FlightControls status={workbench} available={apiAvailable&&observedFresh&&mode==='live'} onCommand={(name,payload)=>void command(name,payload)}/>
   <div className="run-info"><label>运行编号</label><code>{telemetry?.run_id??'—'}</code><label>仿真时间</label><strong>{telemetry?telemetry.simulation_time.toFixed(1):'—'} <small>s</small></strong></div>
   <div className="divider"/><div className="section-label">组件健康 <span>HEALTH</span></div>
   {Object.entries(telemetry?.diagnostics??{}).map(([name,value])=><div className="health-row" key={name}><i className={observedFresh&&value.level===0&&value.age_s<2?'ok':'warn'}/><div><span>{name.replace('uav001/','')}</span><small>{!connected?'观测已断开':observedFresh?value.message:'遥测已过期'}</small></div></div>)}
   {!Object.keys(telemetry?.diagnostics??{}).length&&<p className="fine-print">连接观测桥后显示实际诊断。</p>}
   <div className="evaluation-note">{telemetry?.evaluation?'评估模式 · 真值独立展示':'观测模式 · 真值未订阅'}</div>
   {error&&<p className="error-message">{error}</p>}
  </aside>
  <footer><span>本机单机实验 · Gazebo / PX4 / ROS 2</span><span>原始数据与显示数据分别保留</span></footer>
 </div>;
}
