import {render,screen,cleanup,act,fireEvent} from '@testing-library/react';
import {afterEach,it,expect,vi} from 'vitest';
import {FlightControls} from './Workbench';
vi.mock('./Scene',()=>({Scene:()=> <div>3D 观测</div>}));
import App from './App';
afterEach(()=>{cleanup();vi.unstubAllGlobals();vi.useRealTimers();});

it('禁用解锁旁显示保护原因并关联可访问说明',()=>{
 const status={mode:'live' as const,allowed:['stop'],active_run:'run',connected:true,jobs:[],
  selection:{localization:'glim',planning:'fast_planner',scene:'circle-eight',sensor_profile:null},
  backends:[],catalog_checking:false,replay:null,blocked:{arm:'飞控已进入保护状态，需要停止实验并排查。'}};
 render(<FlightControls status={status} available={true} onCommand={()=>{}}/>);
 expect(screen.getByText(/飞控已进入保护状态/)).toBeTruthy();
 expect(screen.getByRole('button',{name:'解锁'}).getAttribute('aria-describedby')).toBeTruthy();
});

it('8080明确只读，隐藏飞行控制并指向控制台',async()=>{
 vi.stubGlobal('location',{hostname:'127.0.0.1',port:'8080'});
 vi.stubGlobal('WebSocket',class{close(){}});
 const fetcher=vi.fn();vi.stubGlobal('fetch',fetcher);
 await act(async()=>{render(<App/>);});
 expect(screen.getByText(/只读观测入口/)).toBeTruthy();
 expect(screen.queryByRole('button',{name:'解锁'})).toBeNull();
 expect(fetcher).not.toHaveBeenCalled();
});

it('报告读取失败有明确错误和只读重试，不能永久加载',async()=>{
 vi.stubGlobal('WebSocket',class{close(){}});
 const state={mode:'live',selection:{localization:'glim',planning:'fast_planner',scene:'circle-eight',sensor_profile:null},
  active_run:null,allowed:[],jobs:[],backends:[],catalog_checking:false,replay:null};
 vi.stubGlobal('fetch',vi.fn((url:string)=>url.includes('comparison')?Promise.reject(Error('报告服务不可用')):
  Promise.resolve({ok:true,json:async()=>state})));
 await act(async()=>{render(<App/>);});
 await act(async()=>{fireEvent.click(screen.getByRole('button',{name:'算法对照'}));});
 expect(screen.getByRole('alert').textContent).toContain('报告服务不可用');
 expect(screen.getByRole('button',{name:'重新读取'})).toBeTruthy();
 expect(screen.queryByText('正在读取实际验收报告…')).toBeNull();
});

it('命令提交超时标记结果待确认，绝不自动重发',async()=>{
 vi.useFakeTimers();
 vi.stubGlobal('fetch',vi.fn(()=>new Promise(()=>{})));
 const {api}=await import('./Workbench');
 const result=api('arm',{}).catch(error=>error.message);
 await vi.advanceTimersByTimeAsync(10001);
 expect(await result).toContain('结果待确认');
 expect(fetch).toHaveBeenCalledTimes(1);
});

it('所有图层关闭时解释无显示数据而不提示启动实验',async()=>{
 vi.stubGlobal('location',{hostname:'127.0.0.1',port:'8080'});vi.stubGlobal('WebSocket',class{close(){}});
 await act(async()=>{render(<App/>);});
 await act(async()=>{fireEvent.click(screen.getByRole('checkbox',{name:'原始单帧点云'}));});
 expect(screen.getByText('未选择点云图层')).toBeTruthy();
 expect(screen.queryByText('等待点云数据')).toBeNull();
});

it('控制状态查询三秒超时且不会发送控制指令',async()=>{
 vi.useFakeTimers();const fetcher=vi.fn(()=>new Promise(()=>{}));vi.stubGlobal('fetch',fetcher);
 const {api}=await import('./api');const result=api('status').catch(e=>e.message);
 await vi.advanceTimersByTimeAsync(3001);expect(await result).toContain('读取超时');
 expect(fetcher).toHaveBeenCalledTimes(1);
});

it('API断开时仍保留状态但不开放飞行按钮',async()=>{
 vi.stubGlobal('WebSocket',class{close(){}});vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(Error('控制服务不可用'))));
 await act(async()=>{render(<App/>);});
 expect(screen.getByText('控制 API 未连接')).toBeTruthy();
 expect(screen.getByRole('button',{name:'解锁'}).hasAttribute('disabled')).toBe(true);
});

it('点云时间落后时明确标记保留旧帧，与飞行健康分开',async()=>{
 vi.stubGlobal('location',{hostname:'127.0.0.1',port:'8080'});
 let socket:any;
 vi.stubGlobal('WebSocket',class{onopen?:()=>void;onmessage?:(event:unknown)=>void;constructor(){socket=this;}close(){}});
 await act(async()=>{render(<App/>);});
 const text=new TextEncoder().encode(JSON.stringify({kind:'cloud',layer:'raw',count:1,source_count:1,frame:'odom',stamp:1}));
 const offset=4+Math.ceil(text.length/4)*4,buffer=new ArrayBuffer(offset+16);
 new DataView(buffer).setUint32(0,text.length,true);new Uint8Array(buffer,4,text.length).set(text);
 await act(async()=>{socket.onopen();socket.onmessage({data:buffer});socket.onmessage({data:JSON.stringify({kind:'telemetry',simulation_time:10,vehicle:null,diagnostics:{},layer_errors:{}})});});
 expect(screen.getByText(/点云画面已滞后 9.0 秒/)).toBeTruthy();
});
