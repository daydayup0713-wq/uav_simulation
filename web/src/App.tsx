import {useEffect,useRef,useState} from 'react';
import {Controls,layerNames} from './Controls';
import type {Layers,Layer,Coloring} from './Controls';
import {Scene} from './Scene';
import {useReplay} from './useReplay';
import {useControlConnection,useObservation} from './useConnections';
import {useReports} from './useReports';
import {api,Comparison,Configuration,FlightControls,MetricCurve} from './Workbench';
import type {Selection,CurveData} from './Workbench';
import './style.css';

const initialLayers:Layers={raw:true,registered:false,global:false,voxels:false};
export default function App(){
 const [layers,setLayers]=useState(initialLayers),[color,setColor]=useState<Coloring>('height');
 const [follow,setFollow]=useState(false),[reset,setReset]=useState(0);
 const [error,setError]=useState(''),[draft,setDraft]=useState<Selection>({localization:'glim',planning:'fast_planner',scene:'circle-eight',sensor_profile:null});
 const [tab,setTab]=useState('空间观测');
 const [curveLoading,setCurveLoading]=useState(false),[curveError,setCurveError]=useState(''),[curveRun,setCurveRun]=useState('');
 const curveRequest=useRef(0);
 const [curve,setCurve]=useState<CurveData|null>(null),[replayIndex,setReplayIndex]=useState(0),[playing,setPlaying]=useState(false),[recording,setRecording]=useState('');
 const observationOnly=location.port==='8080';
 const control=useControlConnection(!observationOnly),workbench=control.status,apiAvailable=control.available;
 const configured=useRef(false),mode=workbench?.mode??'live';
 const observation=useObservation(mode);
 const {connected,clouds,telemetry,paths}=observation;
 const reports=useReports(tab,mode,!observationOnly);
 const {comparison,runs,recordings}=reports;
 const [wallNow,setWallNow]=useState(Date.now());
 const replay=useReplay(mode,workbench?.replay?.identifier,replayIndex);
 const {cloud:replayCloud,loading:replayLoading,loadedFrame}=replay;
 useEffect(()=>{if(replay.error)setPlaying(false);},[replay.error]);
 useEffect(()=>{const timer=setInterval(()=>setWallNow(Date.now()),200);return()=>clearInterval(timer);},[]);
 useEffect(()=>{if(workbench&&!configured.current){setDraft(workbench.selection);configured.current=true;}},[workbench]);
 const readCurve=async(identifier:string)=>{
  const request=++curveRequest.current;setCurveRun(identifier);setCurveLoading(true);setCurveError('');
  try{const data=await api<CurveData>('metrics?run='+encodeURIComponent(identifier));if(request===curveRequest.current)setCurve(data);}
  catch(e){if(request===curveRequest.current)setCurveError(e instanceof Error?e.message:'指标读取失败');}
  finally{if(request===curveRequest.current)setCurveLoading(false);}
 };
 const command=async(name:string,payload:unknown={})=>{try{await api(name,payload);setError('');if(name==='mode'){setPlaying(false);setReplayIndex(0);}}
  catch(e){setError(e instanceof Error?e.message:'实验操作失败');}};
 useEffect(()=>{setPlaying(false);setReplayIndex(0);},[workbench?.replay?.identifier]);
 useEffect(()=>{
  if(!playing||mode!=='replay'||!workbench?.replay||replayLoading||loadedFrame!==workbench.replay.identifier+':'+replayIndex)return;
  const frames=workbench.replay.frames;
  const timer=setTimeout(()=>{if(replayIndex>=frames-1)setPlaying(false);else setReplayIndex(replayIndex+1);},200);
  return()=>clearTimeout(timer);
 },[playing,mode,workbench?.replay?.identifier,replayIndex,replayLoading,loadedFrame]);
 const displayClouds=mode==='replay'?(replayCloud?{raw:replayCloud}:{}):clouds;
 const displayPaths=mode==='replay'?{actual:workbench?.replay?.actual_path??[]}:paths;
 const observedFresh=connected&&wallNow-observation.received<=1000;
 const vehicle=mode==='live'&&observedFresh?(telemetry?.vehicle??null):null,flight=mode==='live'?telemetry?.diagnostics['uav001/flight']:null;
 const armed=flight?.values.armed==='True',active=(Object.keys(layers) as Layer[]).filter(key=>layers[key]);
 const count=active.reduce((total,key)=>total+(displayClouds[key]?.count??0),0);
 const controlRejected=flight?.message==='READY'&&['navigation_ready','external_ready'].some(key=>flight.values[key]==='False');
 const phase=mode==='replay'?'文件回放':!connected?'观测已断开':!observedFresh&&telemetry?'遥测已过期':controlRejected?'控制未就绪':flight?.message??'等待状态';
 const cloudAge=mode==='live'&&connected&&clouds.raw&&telemetry?Math.max(0,telemetry.simulation_time-clouds.raw.stamp):null;
 return <div className="workbench">
  <header><div className="brand-mark"><span/><span/><span/><span/></div><div><h1>UAV <b>LAB</b></h1><p>无人机算法实验台</p></div>
   <nav>{['空间观测','算法对照','实验记录'].map(name=><button className={tab===name?'nav-current':''} key={name} onClick={()=>setTab(name)}>{name}</button>)}</nav>
   {!observationOnly&&<div className={apiAvailable?'connection live':'connection'}><i/>{apiAvailable?'控制 API 已连接':'控制 API 未连接'}</div>}
   <div className={connected?'connection live':'connection'}><i/>{connected?'观测桥已连接':'等待观测桥'}</div><span className="vehicle-tag">uav001</span>
  </header>
  <aside className="left-panel">
   <div className="workspace-label">实验工作区 <small>L1</small></div>
   <Controls layers={layers} onLayer={(key,value)=>setLayers(previous=>({...previous,[key]:value}))} color={color} onColor={setColor}
    follow={follow} onCamera={action=>action==='reset'?(setFollow(false),setReset(reset+1)):setFollow(!follow)}/>
   {!observationOnly&&<><div className="divider"/><Configuration status={workbench} available={apiAvailable&&(!workbench?.active_run||connected)} draft={draft} onDraft={setDraft} onCommand={(name,payload)=>void command(name,payload)}/></>}
   {!apiAvailable&&!observationOnly&&<p className="fine-print">实验控制 API 未连接。运行 ./scripts/workbench.sh 后访问本地 8780 端口。</p>}
   <div className="divider"/><div className="section-label">图例 <span>LEGEND</span></div>
   <div className="legend"><span><i className="planned-line"/>计划路径</span><span><i className="actual-line"/>实际轨迹</span></div>
   <p className="fine-print">ENU 坐标 · 地面网格 1m<br/>鼠标左键旋转 · 右键平移 · 滚轮缩放</p>
  </aside>
  <main>{observationOnly&&<div className="control-notice" role="status">只读观测入口 · 8080。配置、报告和飞行操作请使用 <a href="http://127.0.0.1:8780/">8780 实验控制台</a>。</div>}
   {reports.error&&<div className="request-error" role="alert">{reports.error} {comparison||runs.length?"保留上次读取的数据，尚未刷新。":""}<button onClick={reports.retry}>重新读取</button></div>}
   {reports.loading&&<p role="status">正在读取报告，请稍候…</p>}
   {tab==='算法对照'?(observationOnly?<p>此入口不提供报告查询。</p>:reports.error&&!comparison?null:<Comparison data={comparison}/>):tab==='实验记录'?<div className="report-view"><span className="eyebrow">TRACEABLE RUNS</span><h2>实验记录</h2><p className="report-note">包括成功与失败运行；选择记录查看原始跟踪指标。</p>
   {!reports.loading&&!reports.error&&!runs.length&&<p role="status">暂无实验记录；完成一次实验后会在这里显示。</p>}<div className="run-list">{runs.map(run=><button key={run.id} onClick={()=>void readCurve(run.id)}><span className={run.passed?'pass':'fail'}>{run.passed?'通过':'失败'}</span><b>{run.id}</b><small>{run.reason||run.source}</small></button>)}</div>{curveLoading&&<p role="status">正在读取跟踪指标，请稍候…</p>}{curveError&&<p role="alert" className="request-error">{curveError} {curve?'保留上次指标，尚未刷新。':''}<button onClick={()=>void readCurve(curveRun)}>重新读取指标</button></p>}{!curveLoading&&!curveError&&curve&&!curve.points.length&&<p role="status">该记录没有可用的原始跟踪指标。</p>}<MetricCurve data={curve}/></div>:<>
   <div className="view-heading"><div><span className="eyebrow">{mode==='replay'?'RECORDED / READ ONLY':'LIVE SPATIAL OBSERVATORY'}</span><h2>三维点云观察</h2></div>
   <span className="view-frame">ENU / odom</span></div>
   <div className="display-mode"><button disabled={!apiAvailable||mode==='live'} onClick={()=>void command('mode',{mode:'live'})}>实时观测</button><button disabled={!apiAvailable||mode==='replay'} onClick={()=>void command('mode',{mode:'replay'})}>文件回放</button><span>{mode==='replay'?'历史画面 · 飞行控制禁用 · 不发布 ROS 数据':'原始点云与录制保留完整数据'}</span></div>
   {mode==='replay'&&<div className="replay-controls"><select aria-label="录制数据" value={recording} onChange={e=>setRecording(e.target.value)}><option value="">选择本机录制</option>{recordings.map(r=><option key={r.id} value={r.id}>{r.id}</option>)}</select>
    <button disabled={!recording||!apiAvailable} onClick={()=>void command('replay',{identifier:recording})}>载入</button>
    <button disabled={!workbench?.replay} onClick={()=>setPlaying(!playing)}>{playing?'暂停':'播放'}</button>
    {workbench?.replay&&<><input aria-label="回放进度" type="range" min="0" max={workbench.replay.frames-1} value={replayIndex} onChange={e=>{setPlaying(false);setReplayIndex(Number(e.target.value));}}/><span>{workbench.replay.frame_times_s[replayIndex]?.toFixed(1)} / {workbench.replay.duration_s.toFixed(1)} s · 5Hz</span></>}</div>}
   {mode==='replay'&&replay.error&&<p role="alert" className="request-error"><span>{replay.error}</span><button onClick={replay.retry}>重新读取点云帧</button></p>}
   {mode==='replay'&&replayLoading&&<p role="status">正在读取点云帧，请稍候… 慢速读取时暂停推进，保留上次画面。</p>}
   {mode==='replay'&&!reports.loading&&!reports.error&&!recordings.length&&<p role="status" className="control-notice">暂无可用录制。请先完成传感器录制和数据检查，再载入回放。</p>}
   {mode==='replay'&&!!workbench?.replay?.unshown_source_frames&&<p className="fine-print">{workbench.replay.unshown_source_frames} 帧缺少位姿时间支持，未显示；完整原始录制保留。</p>}
   {cloudAge!==null&&cloudAge>1&&<p role="status" className="control-notice">点云画面已滞后 {cloudAge.toFixed(1)} 秒，正在保留最后一帧可用数据。请查看下方图层的时间／TF 提示；飞行健康请以右侧实际诊断为准。</p>}
   <div className="viewport"><Scene clouds={displayClouds} layers={layers} color={color} vehicle={vehicle} paths={displayPaths} follow={follow} reset={reset}/>
    {!count&&<div className="empty-view"><span className="empty-symbol">⠿</span><h3>{!active.length?'未选择点云图层':mode==='replay'?'等待录制载入':!connected?'等待观测桥连接':'等待点云数据'}</h3><p>{!active.length?'请勾选左侧至少一个点云图层。':mode==='replay'?'选择本机录制后点击载入；读取失败时查看右侧原因。':!connected?'检查实验是否启动；连接后会接收实际传感器数据。':'当前所选图层尚未收到数据，可查看下方话题提示。'}</p></div>}
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
   <div className="state-card"><span className="state-caption">当前阶段</span><h3>{phase}</h3>{controlRejected&&mode==='live'&&observedFresh&&<p role="status">遥测已就绪，但定位或导航控制门槛未通过。请查看组件诊断和按钮旁的原因；地面未解锁时可停止实验后排查。</p>}{phase==='FAILSAFE'&&<div className="fault-explanation" role="alert"><b>飞控保护已锁存，当前禁止解锁和起飞。</b><p>数据恢复不会自动续飞。地面未解锁时可停止实验，查看原因后重启。</p><details><summary>故障原因与观测时间</summary>{observation.fault&&<p>首次观测（界面本机）：{new Date(observation.fault.receivedAt).toLocaleString()} · 仿真 {observation.fault.simulationTime.toFixed(1)} s。实际故障发生时刻以运行日志为准。</p>}<code>{flight?.values.reason??workbench?.flight?.reason??'等待诊断详情'}</code></details></div>}<div className="state-meta"><span>{mode==='replay'?'历史数据':!observedFresh?'解锁状态未知':armed?'已解锁':'未解锁'}</span><span>{vehicle&&vehicle.age_s<1?'遥测新鲜':'等待遥测'}</span></div></div>
   {!observationOnly&&<><div className="divider"/><FlightControls status={workbench} available={apiAvailable&&observedFresh&&mode==='live'} onCommand={(name,payload)=>void command(name,payload)}/></>}
   <div className="run-info"><label>运行编号</label><code>{telemetry?.run_id??'—'}</code><label>仿真时间</label><strong>{telemetry?telemetry.simulation_time.toFixed(1):'—'} <small>s</small></strong></div>
   <div className="divider"/><div className="section-label">组件健康 <span>HEALTH</span></div>
   {Object.entries(telemetry?.diagnostics??{}).map(([name,value])=><div className="health-row" key={name}><i className={observedFresh&&value.level===0&&value.age_s<2&&!(name==='uav001/flight'&&controlRejected)?'ok':'warn'}/><div><span>{name.replace('uav001/','')}</span><small>{!connected?'观测已断开':observedFresh?value.message:'遥测已过期'}</small>{value.values.reason&&<small>{value.values.reason}</small>}</div></div>)}
   {!Object.keys(telemetry?.diagnostics??{}).length&&<p className="fine-print">连接观测桥后显示实际诊断。</p>}
   <div className="evaluation-note">{telemetry?.evaluation?'评估模式 · 真值独立展示':'观测模式 · 真值未订阅'}</div>
   {(error||control.error||(mode==='live'&&observation.error))&&<p className="error-message" role="alert">{error||control.error||(mode==='live'?observation.error:null)}</p>}
  </aside>
  <footer><span>本机单机实验 · Gazebo / PX4 / ROS 2</span><span>原始数据与显示数据分别保留</span></footer>
 </div>;
}
