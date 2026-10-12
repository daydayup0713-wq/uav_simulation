/** Deadlines bound waiting. A timed-out POST never authorizes a retry. */
async function bounded<T>(task:(signal:AbortSignal)=>Promise<T>,milliseconds:number,uncertain:boolean,caller?:AbortSignal):Promise<T>{
 if(caller?.aborted)throw new DOMException('读取已取消','AbortError');
 const controller=new AbortController();let cancel:(()=>void)|undefined;let timer:ReturnType<typeof setTimeout>|undefined;
 const timeout=new Promise<never>((_,reject)=>{timer=setTimeout(()=>{
  controller.abort();reject(Error(uncertain?'命令提交超时，结果待确认。请检查任务和飞行状态，勿重复发送。':'读取超时，请检查服务后重新读取。'));
 },milliseconds);});
 const cancelled=new Promise<never>((_,reject)=>{cancel=()=>{controller.abort();reject(new DOMException('读取已取消','AbortError'));};caller?.addEventListener('abort',cancel,{once:true});});
 try{return await Promise.race([task(controller.signal),timeout,cancelled]);}finally{clearTimeout(timer);if(cancel)caller?.removeEventListener('abort',cancel);}
}
export function api<T>(path:string,payload?:unknown,caller?:AbortSignal):Promise<T>{
 return bounded(async signal=>{
  const response=await fetch('/api/'+path,payload===undefined?{cache:'no-store',signal}:
   {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload),signal});
  let data;try{data=await response.json();}catch{throw Error('服务返回了无效数据；请确认使用 8780 实验控制入口。');}
  if(!response.ok)throw Error(data.reason??'实验台请求失败');return data;
 },payload===undefined?(path==='status'?3000:30000):10000,payload!==undefined,payload===undefined?caller:undefined);
}
export function apiBuffer(path:string,caller?:AbortSignal):Promise<ArrayBuffer>{
 return bounded(async signal=>{
  const response=await fetch('/api/'+path,{signal,cache:'no-store'});
  if(!response.ok)throw Error((await response.json()).reason??'回放读取失败');
  return response.arrayBuffer();
 },30000,false,caller);
}
