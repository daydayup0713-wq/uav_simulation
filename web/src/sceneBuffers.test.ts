import {it,expect} from 'vitest';
import * as THREE from 'three';

it('更新点云复用几何与缓冲，缩短点数不绘制旧点',async()=>{
 const {writeCloudGeometry}=await import('./sceneBuffers');
 const geometry=new THREE.BufferGeometry();
 const values=new Float32Array([1,2,3,40,4,5,6,50]);
 writeCloudGeometry(geometry,values,2,'height');
 const position=geometry.getAttribute('position'),colors=geometry.getAttribute('color');
 writeCloudGeometry(geometry,new Float32Array([7,8,9,60]),1,'intensity');
 expect(geometry.getAttribute('position')).toBe(position);
 expect(geometry.getAttribute('color')).toBe(colors);
 expect(position.getX(0)).toBe(7);expect(geometry.drawRange.count).toBe(1);
});

it('缓冲扩容前释放已注册几何，常规更新不重复释放',async()=>{
 const {writeCloudGeometry}=await import('./sceneBuffers');const geometry=new THREE.BufferGeometry();
 writeCloudGeometry(geometry,new Float32Array(4),1,'height');
 let released=0;geometry.addEventListener('dispose',()=>{released++;expect(geometry.getAttribute('position').count).toBe(1024);});
 writeCloudGeometry(geometry,new Float32Array(1500*4),1500,'height');
 expect(released).toBe(1);expect(geometry.getAttribute('position').count).toBeGreaterThanOrEqual(1500);
 writeCloudGeometry(geometry,new Float32Array(1501*4),1501,'height');expect(released).toBe(1);
});
