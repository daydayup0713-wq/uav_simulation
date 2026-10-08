import {useEffect,useState} from 'react';
import {Controls,layerNames} from './Controls';
import type {Layers,Layer,Coloring} from './Controls';
import {Scene} from './Scene';
import {decodeCloud} from './protocol';
import type {Cloud,Telemetry,Backend} from './protocol';
import './style.css';

const initialLayers:Layers={raw:true,registered:false,global:false,voxels:false};
export default function App(){
 const [connected,setConnected]=useState(false),[clouds,setClouds]=useState<Partial<Record<Layer,Cloud>>>({});
 const [telemetry,setTelemetry]=useState<Telemetry|null>(null),[backends,setBackends]=useState<Backend[]>([]);
 const [layers,setLayers]=useState(initialLayers),[color,setColor]=useState<Coloring>('height');
 const [follow,setFollow]=useState(false),[reset,setReset]=useState(0),[paths,setPaths]=useState<Record<string,number[][]>>({});
 const [error,setError]=useState(''),[draft,setDraft]=useState({localization:'glim',planning:'astar'});
 useEffect(()=>{
  let socket:WebSocket|null=null,timer=0,closed=false;
  function open(){if(closed)return;socket=new WebSocket(`ws://${location.hostname}:8765`);socket.binaryType='arraybuffer';
   socket.onopen=()=>{setConnected(true);setError('');};socket.onerror=()=>setError('观测桥未连接');
   socket.onclose=()=>{setConnected(false);if(!closed)timer=window.setTimeout(open,1000);};
   socket.onmessage=event=>{try{
    if(event.data instanceof ArrayBuffer){const cloud=decodeCloud(event.data);setClouds(previous=>({...previous,[cloud.layer]:cloud}));}
    else{const message=JSON.parse(event.data);
     if(message.kind==='telemetry')setTelemetry(message);
     else if(message.kind==='registry')setBackends(message.backends);
     else if(message.kind==='path')setPaths(previous=>({...previous,[message.layer]:message.points}));
    }
   }catch(e){setError(e instanceof Error?e.message:'观测数据错误');}};
  }open();return()=>{closed=true;clearTimeout(timer);socket?.close();};
 },[]);
 const vehicle=connected?(telemetry?.vehicle??null):null,flight=telemetry?.diagnostics['uav001/flight'];
 const armed=flight?.values.armed==='True',active=(Object.keys(layers) as Layer[]).filter(key=>layers[key]);
 const count=active.reduce((total,key)=>total+(clouds[key]?.count??0),0);
 const phase=!connected?'观测已断开':flight?.message??'等待状态';
 const cloudAge=connected&&clouds.raw&&telemetry?Math.max(0,telemetry.simulation_time-clouds.raw.stamp):null;
 return <div className="workbench">
  <header><div className="brand-mark"><span/><span/><span/><span/></div><div><h1>UAV <b>LAB</b></h1><p>无人机算法实验台</p></div>
   <nav><span className="nav-current">空间观测</span><span>算法对照</span><span>实验记录</span></nav>
   <div className={connected?'connection live':'connection'}><i/>{connected?'观测桥已连接':'等待观测桥'}</div><span className="vehicle-tag">uav001</span>
  </header>
  <aside className="left-panel">
   <div className="workspace-label">实验工作区 <small>L1</small></div>
   <Controls layers={layers} onLayer={(key,value)=>setLayers(previous=>({...previous,[key]:value}))} color={color} onColor={setColor}
    follow={follow} onCamera={action=>action==='reset'?(setFollow(false),setReset(reset+1)):setFollow(!follow)}/>
   <div className="divider"/><div className="section-label">下一实验配置 <span>DRAFT</span></div>
   {(['localization','planning'] as const).map(role=><label className="control-label" key={role}>{role==='localization'?'定位后端':'规划后端'}
    <select value={draft[role]} disabled={!connected||!flight||flight.age_s>=1||armed} onChange={event=>setDraft(previous=>({...previous,[role]:event.target.value}))}>
     {(backends.filter(item=>item.role===role).length?backends.filter(item=>item.role===role):[{id:role==='localization'?'glim':'astar',name:role==='localization'?'GLIM':'A*',stage:'configured'}]).map(item=>
      <option key={item.id} value={item.id}>{item.name??item.id} · {item.stage==='configured'?'已配置':item.stage}</option>)}</select></label>)}
   <p className="fine-print">此处选择为草稿，当前实验由 CLI 启动。<br/>{armed?'飞行期间禁止更换后端':'安装、回放、实时与闭环资格分别验收。'}</p>
   <div className="divider"/><div className="section-label">图例 <span>LEGEND</span></div>
   <div className="legend"><span><i className="planned-line"/>计划路径</span><span><i className="actual-line"/>实际轨迹</span></div>
   <p className="fine-print">ENU 坐标 · 地面网格 1m<br/>鼠标左键旋转 · 右键平移 · 滚轮缩放</p>
  </aside>
  <main><div className="view-heading"><div><span className="eyebrow">LIVE SPATIAL OBSERVATORY</span><h2>三维点云观察</h2></div>
   <span className="view-frame">ENU / odom</span></div>
   <div className="viewport"><Scene clouds={clouds} layers={layers} color={color} vehicle={vehicle} paths={paths} follow={follow} reset={reset}/>
    {!count&&<div className="empty-view"><span className="empty-symbol">⠿</span><h3>等待点云数据</h3><p>启动传感器实验后，原始扫描会显示在这里。</p></div>}
    <div className="view-status"><span><i/>{count.toLocaleString()} 显示点</span><span>{connected?'5Hz 显示上限':'缓存画面 · 观测已断开'}</span><span>{follow?'跟随视角':'自由视角'}</span></div>
    <div className="axis-tag"><b>X</b> 东 <b>Y</b> 北 <b>Z</b> 上</div>
   </div>
   <div className="pose-strip">{['X · 东','Y · 北','Z · 高度'].map((label,i)=><div key={label}><span>{label}</span><strong>{vehicle?vehicle.position[i].toFixed(2):'—'}<small>m</small></strong></div>)}
    <div><span>点云源延迟</span><strong>{cloudAge!==null?Math.round(cloudAge*1000):'—'}<small>ms</small></strong></div></div>
   <div className="sources"><div className="section-label">图层数据 <span>SOURCE DATA</span></div>{(Object.keys(layerNames) as Layer[]).map(key=><div className="source-row" key={key}>
    <i className={clouds[key]?'available':''}/><span>{layerNames[key]}</span><b>{clouds[key]?`${clouds[key]!.count.toLocaleString()} / ${clouds[key]!.source_count.toLocaleString()}`:'等待数据'}</b>
    <small>{!connected?'缓存数据':telemetry?.layer_errors[key]??(clouds[key]?.display_decimated?'显示降采样 / 有限窗口':clouds[key]?.invalid_sample_points?'已过滤无效返回':clouds[key]?'完整单帧显示':'未收到话题')}</small></div>)}</div>
  </main>
  <aside className="right-panel"><div className="section-label">飞行状态 <span>TELEMETRY</span></div>
   <div className="state-card"><span className="state-caption">当前阶段</span><h3>{phase}</h3><div className="state-meta"><span>{!connected?'解锁状态未知':armed?'已解锁':'未解锁'}</span><span>{vehicle&&vehicle.age_s<1?'遥测新鲜':'等待遥测'}</span></div></div>
   <div className="run-info"><label>运行编号</label><code>{telemetry?.run_id??'—'}</code><label>仿真时间</label><strong>{telemetry?telemetry.simulation_time.toFixed(1):'—'} <small>s</small></strong></div>
   <div className="divider"/><div className="section-label">组件健康 <span>HEALTH</span></div>
   {Object.entries(telemetry?.diagnostics??{}).map(([name,value])=><div className="health-row" key={name}><i className={connected&&value.level===0&&value.age_s<2?'ok':'warn'}/><div><span>{name.replace('uav001/','')}</span><small>{connected?value.message:'观测已断开'}</small></div></div>)}
   {!Object.keys(telemetry?.diagnostics??{}).length&&<p className="fine-print">连接观测桥后显示实际诊断。</p>}
   <div className="evaluation-note">{telemetry?.evaluation?'评估模式 · 真值独立展示':'观测模式 · 真值未订阅'}</div>
   {error&&<p className="error-message">{error}</p>}
  </aside>
  <footer><span>本机单机实验 · Gazebo / PX4 / ROS 2</span><span>原始数据与显示数据分别保留</span></footer>
 </div>;
}
