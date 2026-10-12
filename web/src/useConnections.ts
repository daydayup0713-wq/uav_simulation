import {useEffect,useState} from 'react';
import {api} from './api';
import {decodeCloud} from './protocol';
import type {Cloud,Telemetry} from './protocol';
import type {Layer} from './Controls';
import type {WorkbenchStatus} from './Workbench';

export function useControlConnection(enabled:boolean){
 const [status,setStatus]=useState<WorkbenchStatus|null>(null),[available,setAvailable]=useState(false),[error,setError]=useState('');
 useEffect(()=>{
  if(!enabled)return;let closed=false,pending=false;
  const poll=async()=>{if(pending)return;pending=true;
   try{const value=await api<WorkbenchStatus>('status');if(!closed){setStatus(value);setAvailable(true);setError('');}}
   catch(e){if(!closed){setAvailable(false);setError(e instanceof Error?e.message:'控制服务连接失败');}}
   finally{pending=false;}};
  void poll();const timer=setInterval(poll,700);
  return()=>{closed=true;clearInterval(timer);};
 },[enabled]);
 return {status,available,error};
}

export function useObservation(mode:'live'|'replay'){
 const [connected,setConnected]=useState(false),[clouds,setClouds]=useState<Partial<Record<Layer,Cloud>>>({});
 const [telemetry,setTelemetry]=useState<Telemetry|null>(null),[paths,setPaths]=useState<Record<string,number[][]>>({});
 const [received,setReceived]=useState(-Infinity),[error,setError]=useState('');
 useEffect(()=>{
  let socket:WebSocket|null=null,timer=0,closed=false;
  setConnected(false);setClouds({});setTelemetry(null);setPaths({});
  function open(){if(closed||mode!=='live')return;
   socket=new WebSocket(`ws://${location.hostname}:8765`);socket.binaryType='arraybuffer';
   socket.onopen=()=>{if(!closed){setConnected(true);setError('');}};
   socket.onerror=()=>{if(!closed)setError('观测桥未连接，显示数据可能停留在上次接收时刻。');};
   socket.onclose=()=>{if(closed)return;setConnected(false);timer=window.setTimeout(open,1000);};
   socket.onmessage=event=>{if(closed)return;try{
    if(event.data instanceof ArrayBuffer){const cloud=decodeCloud(event.data);setClouds(previous=>({...previous,[cloud.layer]:cloud}));}
    else{const message=JSON.parse(event.data);
     if(message.kind==='telemetry'){setReceived(Date.now());setTelemetry(message);}
     else if(message.kind==='path')setPaths(previous=>({...previous,[message.layer]:message.points}));}
   }catch(e){setError(e instanceof Error?e.message:'观测数据错误');}};
  }
  open();return()=>{closed=true;clearTimeout(timer);socket?.close();};
 },[mode]);
 return {connected,clouds,telemetry,paths,received,error};
}
