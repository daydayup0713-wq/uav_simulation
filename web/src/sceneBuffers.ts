import * as THREE from 'three';
import type {Coloring} from './Controls';
const blue=new THREE.Color(0x3e6ebc),teal=new THREE.Color(0x54d5ad),yellow=new THREE.Color(0xf3d381);

function buffer(geometry:THREE.BufferGeometry,name:string,count:number){
 let value=geometry.getAttribute(name) as THREE.BufferAttribute|undefined;
 if(!value||value.count<count){value=new THREE.BufferAttribute(new Float32Array(Math.max(count,Math.min(100000,2**Math.ceil(Math.log2(Math.max(1024,count)))))*3),3);value.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute(name,value);}
 return value;
}

function reserve(geometry:THREE.BufferGeometry,names:string[],count:number){
 // Three deletes registered GPU buffers on geometry disposal, before replacing attributes.
 if(names.some(name=>{const attribute=geometry.getAttribute(name);return attribute&&attribute.count<count;}))geometry.dispose();
}

export function writeCloudGeometry(geometry:THREE.BufferGeometry,points:Float32Array,count:number,color:Coloring){
 reserve(geometry,['position','color'],count);
 const position=buffer(geometry,'position',count),colors=buffer(geometry,'color',count),working=new THREE.Color();
 for(let i=0;i<count;i++){
  const source=i*4,target=i*3;
  for(let axis=0;axis<3;axis++)position.array[target+axis]=points[source+axis];
  const t=THREE.MathUtils.clamp(color==='height'?(points[source+2]+.5)/5.5:points[source+3]/100,0,1);
  working.lerpColors(t<.5?blue:teal,t<.5?teal:yellow,t<.5?t*2:(t-.5)*2);
  working.toArray(colors.array,target);
 }
 position.needsUpdate=true;colors.needsUpdate=true;geometry.setDrawRange(0,count);geometry.boundingSphere=null;
}

export function writePathGeometry(geometry:THREE.BufferGeometry,points:number[][]){
 reserve(geometry,['position'],points.length);
 const position=buffer(geometry,'position',points.length);
 for(let i=0;i<points.length;i++)for(let axis=0;axis<3;axis++)position.array[i*3+axis]=points[i][axis];
 position.needsUpdate=true;geometry.setDrawRange(0,points.length);geometry.boundingSphere=null;
}
