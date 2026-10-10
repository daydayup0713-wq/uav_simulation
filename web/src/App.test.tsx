import {render,screen,cleanup,act,fireEvent} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
vi.mock('./Scene',()=>({Scene:()=> <div>3D 观测</div>}));
import App from './App';
afterEach(()=>{cleanup();vi.unstubAllGlobals();});
afterEach(()=>vi.useRealTimers());
it('观测连接仍在但诊断过期时，不继续显示新鲜飞行状态',()=>{
 vi.useFakeTimers();const sockets:FakeSocket[]=[];
 class FakeSocket{binaryType='';onopen:(()=>void)|null=null;onmessage:((e:{data:string})=>void)|null=null;constructor(){sockets.push(this);}close(){}}
 vi.stubGlobal('WebSocket',FakeSocket);render(<App/>);
 act(()=>{sockets[0].onopen?.();sockets[0].onmessage?.({data:JSON.stringify({kind:'telemetry',simulation_time:2,vehicle:null,layer_errors:{},evaluation:false,
 diagnostics:{'uav001/flight':{message:'HOLDING',values:{armed:'True'},level:0,age_s:0}}})});});
 act(()=>vi.advanceTimersByTime(2500));
 expect(screen.getByRole('heading',{name:'遥测已过期'})).toBeTruthy();
 expect(screen.getByRole('button',{name:'解锁'}).hasAttribute('disabled')).toBe(true);
});
it('断连明确显示观测失效，不继续声称实时飞行状态',()=>{
 const sockets:FakeSocket[]=[];
 class FakeSocket{binaryType='';onopen:(()=>void)|null=null;onclose:(()=>void)|null=null;onmessage:((e:{data:string})=>void)|null=null;
  constructor(){sockets.push(this);}close(){} }
 vi.stubGlobal('WebSocket',FakeSocket);
 render(<App/>);
 act(()=>{sockets[0].onopen?.();sockets[0].onmessage?.({data:JSON.stringify({kind:'telemetry',simulation_time:2,vehicle:null,layer_errors:{},evaluation:false,
  diagnostics:{'uav001/flight':{message:'MOVING',values:{armed:'True'},level:0,age_s:0}}})});});
 expect(screen.getByRole('heading',{name:'MOVING'})).toBeTruthy();
 act(()=>sockets[0].onclose?.());
 expect(screen.getByRole('heading',{name:'观测已断开'})).toBeTruthy();
 expect(screen.getByRole('combobox',{name:'定位后端'}).hasAttribute('disabled')).toBe(true);
});

it('空间观测不反复验证离线报告，慢对照请求不会累积',async()=>{
 vi.useFakeTimers();
 class FakeSocket{close(){}}
 vi.stubGlobal('WebSocket',FakeSocket);
 const status={mode:'live',selection:{localization:'glim',planning:'fast_planner',scene:'circle-eight',sensor_profile:null},
  flight:{phase:'UNKNOWN',received_at:null},active_run:null,allowed:[],jobs:[],backends:[],catalog_checking:false,replay:null};
 const requests=vi.fn((url:string)=>url.includes('comparison')?new Promise(()=>{}):Promise.resolve({ok:true,json:async()=>status}));
 vi.stubGlobal('fetch',requests);
 await act(async()=>{render(<App/>);});
 const comparisons=()=>requests.mock.calls.filter(([url])=>url.includes('comparison')).length;
 expect(comparisons()).toBe(0);
 await act(async()=>{fireEvent.click(screen.getByRole('button',{name:'算法对照'}));});
 expect(comparisons()).toBe(1);
 await act(async()=>{vi.advanceTimersByTime(45000);});
 expect(comparisons()).toBe(1);
 await act(async()=>{fireEvent.click(screen.getByRole('button',{name:'空间观测'}));vi.advanceTimersByTime(15000);});
 expect(comparisons()).toBe(1);
});
