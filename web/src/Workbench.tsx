import {useState} from 'react';
import type {Backend} from './protocol';
export type Selection={localization:string;planning:string;scene:string;sensor_profile:string|null;rendering?:string};
export type ReplayInfo={identifier:string;frames:number;duration_s:number;frame_times_s:number[];actual_path:number[][];ros_publications:number;scope:string;unshown_source_frames?:number;unshown_reason?:string};
export type Job={id:string;command:string;state:string;success:boolean|null;reason:string};
export type WorkbenchStatus={mode:'live'|'replay';allowed:string[];active_run:string|null;connected:boolean;selection:Selection;
 jobs:Job[];backends:Backend[];catalog_checking:boolean;replay:ReplayInfo|null};
type Run={backend:string;success:boolean;source_verified:boolean;current_implementation?:boolean;error?:string;
 playback_rate?:number;input_counts?:Record<string,number>;
 quality?:{metrics?:Record<string,number|null>;reason?:string};global_quality?:{metrics?:Record<string,number|null>;reason?:string};
 rtk_batch?:{completed:boolean;reason?:string;pose_target?:string;quality?:{passed?:boolean;metrics?:Record<string,number|null>}};
 capability_observations?:{loop_closure?:string;relocalization?:string;confirmed_loop_edges?:number;global_pose_messages?:number};
 resource_measurements?:{phase:string;mean_cpu_cores?:number;peak_rss_mib?:number;scope:string}[];resource_summary?:{mean_cpu_cores?:number;peak_combined_rss_bytes?:number};latency_source_s?:{p95?:number}};
export type ComparisonData={groups:{input_group:string;cases:{dataset_identity:string;comparison_identity?:string;
 playback_rate?:number;requested_source_duration_s?:number|null;runs:Run[]}[]}[];
 planning:{scope?:string;rows:Record<string,unknown>[]};closed_loop:{scope?:string;rows:Record<string,unknown>[]}};
export type RecordedRun={id:string;passed:boolean;reason:string;run_id:string|null;source:string};
export type Recording={id:string;duration_s:number|null;complete:boolean};
export type CurveData={scope:string;points:{time_s:number;error_m:number;speed_m_s:number;acceleration_m_s2:number}[]};
export const scenes=['circle-eight','helix','multi-room','corridor','dense','outdoor-rtk'];
const sceneNames=['圆形与八字','多高度螺旋','多房间闭环','重复结构长走廊','密集静态障碍','室外路线 / RTK 异常'];
const stageNames:Record<string,string>={configured:'已配置',installed:'已安装',replay_passed:'回放通过',realtime_passed:'实时通过',closed_loop_qualified:'允许闭环'};
export async function api<T>(path:string,payload?:unknown):Promise<T>{
 const response=await fetch('/api/'+path,payload===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
 const data=await response.json();if(!response.ok)throw Error(data.reason??'实验台请求失败');return data;
}
function number(value:unknown,digits=3){return typeof value==='number'&&Number.isFinite(value)?value.toFixed(digits):'—';}

export function FlightControls({status,available,onCommand}:{status:WorkbenchStatus|null;available:boolean;onCommand:(command:string,payload?:unknown)=>void}){
 const [target,setTarget]=useState(['0','0','2']),[height,setHeight]=useState('2');
 const can=(name:string)=>available&&status?.mode==='live'&&status.allowed.includes(name);
 return <div className="flight-controls"><div className="section-label">实验控制 <span>EXPLICIT COMMANDS</span></div>
  <div className="command-grid"><button disabled={!can('arm')} onClick={()=>onCommand('arm',{})}>解锁</button>
   <button disabled={!can('disarm')} onClick={()=>onCommand('disarm',{})}>上锁</button>
   <button disabled={!can('hold')} onClick={()=>onCommand('hold',{})}>悬停</button>
   <button className="land-command" disabled={!can('land')} onClick={()=>onCommand('land',{})}>降落</button></div>
  <label className="control-label">起飞高度 / m<input type="number" min=".5" max="4.5" step=".1" value={height} onChange={e=>setHeight(e.target.value)}/></label>
  <button disabled={!can('takeoff')||!Number.isFinite(Number(height))} onClick={()=>onCommand('takeoff',{height:Number(height)})}>起飞</button>
  <label className="control-label">导航目标 ENU / m</label><div className="target-inputs">{target.map((value,i)=><label key={i}>{['X','Y','Z'][i]}<input aria-label={'目标 '+['X','Y','Z'][i]} type="number" step=".1" value={value} onChange={e=>setTarget(old=>old.map((x,j)=>i===j?e.target.value:x))}/></label>)}</div>
  <div className="command-grid"><button disabled={!can('goto')||!target.every(x=>Number.isFinite(Number(x)))} onClick={()=>onCommand('goto',{target:target.map(Number)})}>前往目标</button>
   <button disabled={!can('route')} onClick={()=>onCommand('route',{})}>执行固定航线</button></div>
  <p className="fine-print">先解锁，再起飞。航线结束保持悬停，请显式降落。失效状态禁止控制；HOLD / LAND 可打断运动。</p>
  {status?.jobs.slice(-3).reverse().map(job=><div className={'job '+(job.success===false?'failed':'')} key={job.id}><b>{job.command} · {job.state}</b><small>{job.reason||'等待服务应答与实际状态确认'}</small></div>)}
 </div>;
}

export function Configuration({status,available,draft,onDraft,onCommand}:{status:WorkbenchStatus|null;available:boolean;draft:Selection;onDraft:(value:Selection)=>void;onCommand:(command:string,payload?:unknown)=>void}){
 const canSelect=available&&status?.mode==='live'&&(!status.active_run||status.allowed.includes('select'));
 return <><div className="section-label">下一实验配置 <span>{status?.active_run?'NEXT RUN':'CONFIGURATION'}</span></div>
  {(['localization','planning'] as const).map(role=><label className="control-label" key={role}>{role==='localization'?'定位后端':'规划后端'}
   <select value={draft[role]} disabled={!canSelect} onChange={e=>onDraft({...draft,[role]:e.target.value})}>
    {(status?.backends.filter(x=>x.role===role).length?status.backends.filter(x=>x.role===role):[{id:draft[role],name:draft[role],role,stage:'configured'}]).map(x=><option key={x.id} value={x.id}>{x.name??x.id} · {stageNames[x.stage]??x.stage}</option>)}
   </select></label>)}
  <label className="control-label">实验场景<select value={draft.scene} disabled={!canSelect} onChange={e=>onDraft({...draft,scene:e.target.value})}>{scenes.map((x,i)=><option key={x} value={x}>{sceneNames[i]}</option>)}</select></label>
  <label className="control-label">传感器输入<select value={draft.sensor_profile??'sync'} disabled={!canSelect} onChange={e=>onDraft({...draft,sensor_profile:e.target.value==='sync'?null:e.target.value})}>
   <option value="sync">同步通用雷达 / 回归基线</option><option value="livox">Livox 类 + IMU + 单目</option><option value="livox-rtk">Livox 类 + RTK</option><option value="mechanical">旋转雷达 + 姿态 IMU</option></select></label>
  <label className="control-label">渲染模式<select value={draft.rendering??'auto'} disabled={!canSelect} onChange={e=>onDraft({...draft,rendering:e.target.value})}>
   <option value="auto">自动 / 无窗口 EGL</option><option value="mesa-display">Intel / Mesa 显示渲染</option></select></label>
  <div className="command-grid"><button disabled={!canSelect} onClick={()=>onCommand('select',draft)}>保存配置</button>
   <button disabled={!available||status?.mode!=='live'||!!status?.active_run||status?.catalog_checking} onClick={()=>onCommand('start',{})}>启动实验</button>
   <button disabled={!available||!status?.allowed.includes('stop')||status.mode!=='live'} onClick={()=>onCommand('stop',{})}>停止实验</button></div>
  <p className="fine-print">启动使用已保存配置，只允许当前版本与指定组合通过闭环资格的后端。飞行期间禁止切换，修改在重启后生效。</p>
 </>;
}

export function Comparison({data}:{data:ComparisonData|null}){
 if(!data)return <div className="report-view"><h2>算法对照</h2><p>正在读取实际验收报告…</p></div>;
 return <div className="report-view"><span className="eyebrow">MEASURED / GROUPED EVIDENCE</span><h2>算法对照</h2><p className="report-note">仅在相同传感器输入、数据集、回放速率和源时间范围内比较。半速离线通过不代表实时通过，允许闭环另行验收。</p>
  {data.groups.map(group=><section className="report-group" key={group.input_group}><h3>{group.input_group}</h3>{group.cases.map(test=><div key={test.comparison_identity??test.dataset_identity}><p className="dataset-id">数据集指纹 {test.dataset_identity.slice(0,20)} · 回放 {number(test.playback_rate,2)}× · 源时间范围 {test.requested_source_duration_s===undefined?'未记录':test.requested_source_duration_s===null?'完整序列':test.requested_source_duration_s+' s'}</p>
   <div className="table-scroll"><table><thead><tr>{['后端','实测结果','回放速率','ATE / m','RPE / m','姿态 / °','覆盖率','延迟 P95 / s','CPU / 核','内存 / MiB','版本证据'].map(x=><th key={x}>{x}</th>)}</tr></thead>
    <tbody>{test.runs.map((run,i)=><tr key={run.backend+i}><td>{run.backend}</td><td className={run.success?'pass':'fail'}>{run.success?'通过':'失败'}</td>
     <td>{run.playback_rate==null?'—':number(run.playback_rate,2)+'×'}</td>
     <td>{number(run.quality?.metrics?.ate_rmse_m)}</td><td>{number(run.quality?.metrics?.rpe_translation_rmse_m)}</td><td>{number(run.quality?.metrics?.attitude_rmse_deg,2)}</td>
     <td>{number(run.quality?.metrics?.coverage,3)}</td><td>{number(run.latency_source_s?.p95)}</td><td>{number(run.resource_measurements?.[0]?.mean_cpu_cores,2)}</td><td>{number(run.resource_measurements?.[0]?.peak_rss_mib,1)}</td>
     <td>{!run.source_verified?'证据缺失或改变':run.current_implementation?'当前实现':'历史实现'}</td></tr>)}</tbody></table></div>
   {test.runs.map((run,i)=><div className="capability-note" key={'detail'+i}>
    {run.input_counts?.['/uav001/camera/image_raw']!==undefined&&<p className="fine-print">{`${run.backend} 回放观测器接收计数：图像 ${run.input_counts['/uav001/camera/image_raw']}，CameraInfo ${run.input_counts['/uav001/camera/camera_info']??'未测'}。用于核对数据输送；核心接收情况另核对原始日志，同一录制文件不保证消息完整到达。`}</p>}
    {run.resource_measurements?.map(item=><p className="fine-print" key={item.phase}>{`${run.backend} · ${item.phase} · CPU ${number(item.mean_cpu_cores,2)} 核 · 内存 ${number(item.peak_rss_mib,1)} MiB · ${item.scope}`}</p>)}
    {run.global_quality?.metrics&&<p className="fine-print">{`全局优化层（仅评估）：ATE ${number(run.global_quality.metrics.ate_rmse_m)} m，RPE ${number(run.global_quality.metrics.rpe_translation_rmse_m)} m，覆盖 ${number(run.global_quality.metrics.coverage)}；修正不输入实时飞控。`}</p>}
    {run.rtk_batch&&<p className="fine-print">{`RTK 序列结束后处理：${run.rtk_batch.completed?'完成':'未完成'} · ${run.rtk_batch.reason??'未记录原因'}；质量${run.rtk_batch.quality?.passed===undefined?'未评估':run.rtk_batch.quality.passed?'通过':'失败'}，ATE ${number(run.rtk_batch.quality?.metrics?.ate_rmse_m)} m，RPE ${number(run.rtk_batch.quality?.metrics?.rpe_translation_rmse_m)} m，覆盖 ${number(run.rtk_batch.quality?.metrics?.coverage)}；测量对象 ${run.rtk_batch.pose_target??'见原始报告'}。修正不输入实时飞控。`}</p>}
    {run.capability_observations&&<p className="fine-print">{`实际能力证据：回环 ${run.capability_observations.loop_closure??'未验证'}，已确认边 ${run.capability_observations.confirmed_loop_edges??0}；重定位 ${run.capability_observations.relocalization??'未验证'}`}</p>}
   </div>)}
   {test.runs.filter(x=>!x.success).map((run,i)=><p className="failure-note" key={i}>{run.backend}: {run.error||run.quality?.reason||'精度、覆盖或时效未达门禁'}</p>)}</div>)}</section>)}
  {[['固定地图规划',data.planning],['PX4 连续闭环',data.closed_loop]].map(([title,report])=>{const item=report as ComparisonData['planning'];return <section className="report-group" key={String(title)}><h3>{String(title)}</h3><p className="report-note">{item.scope}</p><p className="fine-print">规划成功率仅统计预期可达目标；预期结果通过率另含障碍与未知空间拒绝检查。闭环成功率按实际完成架次统计，失败尝试保留。</p><div className="table-scroll"><table><thead><tr><th>后端 / 场景</th><th>成功率</th><th>预期结果通过率</th><th>耗时 / s</th><th>路径长 / m</th><th>跟踪 P95 / m</th><th>机体净距 / m</th><th>速度 / 加速度 / jerk 峰值</th><th>结果</th></tr></thead><tbody>{item.rows.map((row,i)=>{const maxima=row.analytic_maxima as {speed?:number;acceleration?:number;jerk?:number}|undefined;return <tr key={i}><td>{String(row.backend??row.planning??'—')} / {String(row.scene??'固定地图')}</td>
   <td>{number(row.success_rate,2)}</td><td>{number(row.expected_outcome_rate,2)}</td><td>{number(row.mean_runtime_s??row.runtime_s)}</td><td>{number(row.mean_length_m??row.length_m)}</td><td>{number(row.tracking_p95_m)}</td><td>{number(row.minimum_body_clearance_m)}</td><td>{maxima?`${number(maxima.speed)} / ${number(maxima.acceleration)} / ${number(maxima.jerk)}`:'—'}</td><td className={row.passed===false?'fail':''}>{row.source_verified===false?'证据缺失或改变':row.reason?String(row.reason):row.passed===false?'失败':row.passed===true?(row.current_implementation===false?'历史实现通过':'通过'):'见分项报告'}</td></tr>})}</tbody></table></div></section>})}
 </div>;
}

export function MetricCurve({data}:{data:CurveData|null}){
 if(!data?.points.length)return <p className="fine-print">选择带原始跟踪记录的实验查看误差、速度和加速度曲线。</p>;
 const start=data.points[0].time_s,duration=Math.max(.01,data.points.at(-1)!.time_s-start);
 return <section className="metric-curves"><p>{data.scope}</p>{(['error_m','speed_m_s','acceleration_m_s2'] as const).map((key,i)=>{
  const max=Math.max(.3,...data.points.map(p=>p[key]));return <div key={key}><label>{['跟踪误差 / m','速度 / m·s⁻¹','加速度 / m·s⁻²'][i]} · 最大 {number(max)}</label>
   <svg viewBox="0 0 600 100" role="img" aria-label={['跟踪误差曲线','速度曲线','加速度曲线'][i]}><line x1="0" y1="95" x2="600" y2="95" stroke="#293f53"/><polyline fill="none" stroke={['#7fcfc1','#88b8eb','#d6b576'][i]} strokeWidth="1.5" points={data.points.map(p=>`${(p.time_s-start)/duration*600},${95-p[key]/max*90}`).join(' ')}/></svg></div>})}</section>;
}
