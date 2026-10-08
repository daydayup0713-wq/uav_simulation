import { render,screen,cleanup } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { afterEach,expect,it,vi } from 'vitest';
import { Controls } from './Controls';
afterEach(cleanup);
it('原始点云、全局地图和着色分别控制，跟随与复位可操作',async()=>{
 const layers={raw:true,registered:false,global:false,voxels:false};
 const changed=vi.fn(),color=vi.fn(),camera=vi.fn();
 render(<Controls layers={layers} onLayer={changed} color="height" onColor={color} follow={false} onCamera={camera}/>);
 await userEvent.click(screen.getByRole('checkbox',{name:'全局优化地图'}));
 expect(changed).toHaveBeenCalledWith('global',true);
 await userEvent.selectOptions(screen.getByRole('combobox',{name:'点云着色'}),'intensity');
 expect(color).toHaveBeenCalledWith('intensity');
 await userEvent.click(screen.getByRole('button',{name:'跟随无人机'}));
 expect(camera).toHaveBeenCalledWith('follow');
 await userEvent.click(screen.getByRole('button',{name:'复位视角'}));
 expect(camera).toHaveBeenCalledWith('reset');
 expect(screen.getByText(/显示降采样/)).toBeTruthy();
});
