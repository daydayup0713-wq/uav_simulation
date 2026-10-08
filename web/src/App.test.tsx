import {render,screen,cleanup,act} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
vi.mock('./Scene',()=>({Scene:()=> <div>3D 观测</div>}));
import App from './App';
afterEach(()=>{cleanup();vi.unstubAllGlobals();});
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
