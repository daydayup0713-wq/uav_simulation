import {render,screen,fireEvent,cleanup} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {FlightControls,Comparison,Configuration} from './Workbench';
afterEach(cleanup);

it('labels offline replay speed and actual received image counts',()=>{
 render(<Comparison data={{groups:[{input_group:'livox',cases:[{dataset_identity:'bag',playback_rate:.5,
  requested_source_duration_s:200,runs:[{backend:'fast_livo2',success:false,source_verified:true,
   playback_rate:.5,input_counts:{'/uav001/camera/image_raw':3000,'/uav001/camera/camera_info':6061}}]}]}],
  planning:{rows:[]},closed_loop:{rows:[]}}}/>);
 expect(screen.getByText('0.50×')).toBeTruthy();
 expect(screen.getByText(/图像 3000.*CameraInfo 6061/)).toBeTruthy();
 expect(screen.getByText(/源时间范围 200/)).toBeTruthy();
});

it('只显示已确认状态允许的操作，回放无法发送飞行命令',()=>{
 const command=vi.fn();const status={mode:'replay' as const,allowed:['arm','land'],active_run:'test',connected:true,jobs:[],selection:{localization:'glim',planning:'ego',scene:'helix',sensor_profile:null},backends:[],catalog_checking:false,replay:null};
 render(<FlightControls status={status} available={true} onCommand={command}/>);
 fireEvent.click(screen.getByRole('button',{name:'解锁'}));
 expect(command).not.toHaveBeenCalled();
 expect(screen.getByRole('button',{name:'降落'}).hasAttribute('disabled')).toBe(true);
});

it('对照保留输入组和失败结果，不将缺失证据显示为通过',()=>{
 render(<Comparison data={{groups:[{input_group:'视觉+惯性',cases:[{dataset_identity:'bag1',runs:[{backend:'orb_slam3',success:false,current_implementation:true,source_verified:true,quality:{metrics:{coverage:.03,ate_rmse_m:.01}}}]}]},{input_group:'雷达+惯性',cases:[{dataset_identity:'bag2',runs:[{backend:'glim',success:true,source_verified:false}]}]}],planning:{rows:[]},closed_loop:{rows:[]}}}/>);
 expect(screen.getByText('视觉+惯性')).toBeTruthy();expect(screen.getByText('雷达+惯性')).toBeTruthy();
 expect(screen.getByText('失败')).toBeTruthy();expect(screen.getByText('证据缺失或改变')).toBeTruthy();
});


test('shows separate measured resource phases and RTK posterior without calling it live SLAM',()=>{
 const data={groups:[{input_group:'RTK',cases:[{dataset_identity:'bag',runs:[{backend:'fast_livo2_rtk',success:true,source_verified:true,
  quality:{metrics:{ate_rmse_m:.1}},rtk_batch:{completed:true,quality:{passed:true,metrics:{ate_rmse_m:.02}},pose_target:'GNSS antenna'},
  resource_measurements:[{phase:'replay',mean_cpu_cores:1.2,peak_rss_mib:123,scope:'algorithm only'}],
  capability_observations:{loop_closure:'not observed',relocalization:'not verified'}}]}]}],planning:{rows:[]},closed_loop:{rows:[]}};
 render(<Comparison data={data}/>);
 expect(screen.getByText(/replay.*1.20.*123.0/)).toBeTruthy();
 expect(screen.getByText(/后处理.*0.020/)).toBeTruthy();
 expect(screen.getByText(/GNSS antenna/)).toBeTruthy();
 expect(screen.getByText(/not observed/)).toBeTruthy();
});

it('reports an unfinished RTK batch explicitly without relabeling global odometry',()=>{
 render(<Comparison data={{groups:[{input_group:'RTK',cases:[{dataset_identity:'bag',runs:[{
  backend:'fast_livo2_rtk',success:false,source_verified:true,
  global_quality:{metrics:{ate_rmse_m:.333}},rtk_batch:{completed:false,reason:'batch timeout'}}]}]}],
  planning:{rows:[]},closed_loop:{rows:[]}}}/>);
 expect(screen.getByText(/RTK 序列结束后处理.*未完成.*batch timeout/)).toBeTruthy();
 expect(screen.getByText(/全局优化层.*0.333/)).toBeTruthy();
});

it('does not label a pose graph map correction as RTK posterior',()=>{
 render(<Comparison data={{groups:[{input_group:'mechanical',cases:[{dataset_identity:'bag',runs:[{backend:'lio_sam',success:false,source_verified:true,global_quality:{metrics:{ate_rmse_m:.348}}}]}]}],planning:{rows:[]},closed_loop:{rows:[]}}}/>);
 expect(screen.queryByText(/RTK 序列结束后处理/)).toBeNull();
 expect(screen.getByText(/全局优化层.*0.348/)).toBeTruthy();
});

it('saves an explicit renderer choice without silently choosing it',()=>{
 const command=vi.fn(),changed=vi.fn();
 const status={mode:'live' as const,allowed:[],active_run:null,connected:false,jobs:[],selection:{localization:'glim',planning:'ego',scene:'helix',sensor_profile:null},backends:[],catalog_checking:false,replay:null};
 render(<Configuration status={status} available={true} draft={status.selection} onDraft={changed} onCommand={command}/>);
 fireEvent.change(screen.getByLabelText('渲染模式'),{target:{value:'mesa-display'}});
 expect(changed).toHaveBeenCalledWith({...status.selection,rendering:'mesa-display'});
});

it('distinguishes planning success from expected rejection and exposes measured trajectory derivatives',()=>{
 render(<Comparison data={{groups:[],planning:{rows:[{backend:'ego',success_rate:.5,expected_outcome_rate:.75}]},closed_loop:{rows:[{
   backend:'glim + fast_planner',passed:true,runs:3,successful_runs:3,analytic_maxima:{speed:.481,acceleration:.361,jerk:.621}}]}}}/>);
 expect(screen.getAllByText('预期结果通过率').length).toBeGreaterThan(0);
 expect(screen.getByText('0.481 / 0.361 / 0.621')).toBeTruthy();
});
