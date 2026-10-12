import {useEffect,useRef} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import type {Layers,Layer,Coloring} from './Controls';
import type {Cloud,Vehicle} from './protocol';
import {writeCloudGeometry,writePathGeometry} from './sceneBuffers';

export function Scene({clouds,layers,color,vehicle,paths,follow,reset}:{clouds:Partial<Record<Layer,Cloud>>;layers:Layers;color:Coloring;
 vehicle:Vehicle|null;paths:Record<string,number[][]>;follow:boolean;reset:number}){
 const host=useRef<HTMLDivElement>(null),handle=useRef<{update:()=>void;reset:()=>void}|undefined>(undefined);
 const latest=useRef({clouds,layers,color,vehicle,paths,follow});latest.current={clouds,layers,color,vehicle,paths,follow};
 useEffect(()=>{
  const container=host.current!;let frame=0;
  const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x0a1220);container.appendChild(renderer.domElement);
  const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,1,.05,500);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.08;
  controls.minDistance=1;controls.maxDistance=300;
  const grid=new THREE.GridHelper(80,80,0x324159,0x172538);grid.rotation.x=Math.PI/2;scene.add(grid);
  const axis=new THREE.AxesHelper(1.2);scene.add(axis);
  scene.add(new THREE.HemisphereLight(0xdaecff,0x2a3b56,2));
  const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(2,-3,8);scene.add(light);
  const drone=new THREE.Group();drone.visible=false;scene.add(drone);
  const shell=new THREE.MeshStandardMaterial({color:0xe0edf5,metalness:.3,roughness:.4});
  drone.add(new THREE.Mesh(new THREE.BoxGeometry(.34,.22,.12),shell));
  for(const [x,y] of [[-.3,-.3],[-.3,.3],[.3,-.3],[.3,.3]]){
   const arm=new THREE.Mesh(new THREE.CylinderGeometry(.024,.024,.42,8),shell);arm.rotation.z=-Math.atan2(x,y);arm.position.set(x/2,y/2,0);drone.add(arm);
   const rotor=new THREE.Mesh(new THREE.CylinderGeometry(.13,.13,.014,32),new THREE.MeshStandardMaterial({color:0x45c9bf,transparent:true,opacity:.65}));
   rotor.rotation.x=Math.PI/2;rotor.position.set(x,y,.055);drone.add(rotor);
  }
  const objects:Partial<Record<Layer,THREE.Points>>={},lines:Record<string,THREE.Line>={};
  const cached:Partial<Record<Layer,{cloud:Cloud;color:Coloring}>>={};
  const cachedPaths:Record<string,number[][]>={};
  function resetView(){camera.position.set(10,-13,10);controls.target.set(1,0,1.5);controls.update();}
  function update(){
   const value=latest.current;
   for(const layer of Object.keys(value.layers) as Layer[]){
    const cloud=value.clouds[layer];if(objects[layer])objects[layer]!.visible=value.layers[layer]&&Boolean(cloud);if(!cloud)continue;
    if(cached[layer]?.cloud===cloud&&cached[layer]?.color===value.color)continue;
    if(!objects[layer]){
     objects[layer]=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:layer==='voxels'?.18:.035,vertexColors:true,transparent:true,opacity:layer==='raw'?1:.7}));
     objects[layer]!.frustumCulled=false;scene.add(objects[layer]!);
    }
    writeCloudGeometry(objects[layer]!.geometry,cloud.points,cloud.count,value.color);
    objects[layer]!.visible=value.layers[layer];cached[layer]={cloud,color:value.color};
   }
   drone.visible=Boolean(value.vehicle&&value.vehicle.age_s<2);
   if(value.vehicle){drone.position.fromArray(value.vehicle.position);drone.quaternion.fromArray(value.vehicle.quaternion);}
   for(const name of Object.keys(lines))lines[name].visible=name in value.paths;
   for(const [name,points] of Object.entries(value.paths)){
    if(lines[name])lines[name].visible=true;
    if(cachedPaths[name]===points)continue;
    if(!lines[name]){lines[name]=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:name==='planned'?0xffc96b:0x80baff,transparent:true,opacity:.9}));scene.add(lines[name]);}
    writePathGeometry(lines[name].geometry,points);cachedPaths[name]=points;
   }
  }
  handle.current={update,reset:resetView};resetView();update();
  const resize=new ResizeObserver(()=>{const {width,height}=container.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/Math.max(1,height);camera.updateProjectionMatrix();});resize.observe(container);
  let lastRender=0;
  function animate(now=0){frame=requestAnimationFrame(animate);if(document.hidden||now-lastRender<1000/30)return;lastRender=now;if(latest.current.follow&&drone.visible){const delta=drone.position.clone().sub(controls.target);controls.target.addScaledVector(delta,.07);camera.position.addScaledVector(delta,.07);}controls.update();renderer.render(scene,camera);}
  animate();return()=>{cancelAnimationFrame(frame);resize.disconnect();controls.dispose();scene.traverse(o=>{if(o instanceof THREE.Mesh||o instanceof THREE.Points||o instanceof THREE.Line){o.geometry.dispose();const materials=Array.isArray(o.material)?o.material:[o.material];materials.forEach(m=>m.dispose());}});renderer.dispose();renderer.domElement.remove();handle.current=undefined;};
 },[]);
 useEffect(()=>{handle.current?.update();},[clouds,layers,color,vehicle,paths]);
 useEffect(()=>{handle.current?.reset();},[reset]);
 return <div ref={host} className="scene" aria-label="ENU 三维点云视图"/>;
}
