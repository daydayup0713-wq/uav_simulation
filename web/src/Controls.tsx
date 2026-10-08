export type Layer = 'raw'|'registered'|'global'|'voxels';
export type Layers = Record<Layer,boolean>;
export type Coloring = 'height'|'intensity';
export const layerNames:Record<Layer,string>={raw:'原始单帧点云',registered:'配准累积点云',global:'全局优化地图',voxels:'障碍体素'};
export function Controls({layers,onLayer,color,onColor,follow,onCamera}:{
 layers:Layers;onLayer:(layer:Layer,value:boolean)=>void;color:Coloring;onColor:(color:Coloring)=>void;
 follow:boolean;onCamera:(action:'follow'|'reset')=>void;
}){
 return <><div className="section-label">点云图层 <span>LAYERS</span></div>
 <div className="layers">{(Object.keys(layerNames) as Layer[]).map((key,i)=><label key={key} className={layers[key]?'layer enabled':'layer'}>
 <span className={`layer-dot dot-${i}`}/><span>{layerNames[key]}</span>
 <input aria-label={layerNames[key]} type="checkbox" checked={layers[key]} onChange={e=>onLayer(key,e.target.checked)}/></label>)}</div>
 <label className="control-label">点云着色<select aria-label="点云着色" value={color} onChange={e=>onColor(e.target.value as Coloring)}>
 <option value="height">高度 · ENU Z</option><option value="intensity">反射强度</option></select></label>
 <div className="camera-controls"><button className={follow?'selected':''} onClick={()=>onCamera('follow')}>跟随无人机</button>
 <button onClick={()=>onCamera('reset')}>复位视角</button></div>
 <p className="fine-print">显示降采样 · 5Hz / 每层最多 10 万点<br/>录制保留完整接收数据</p></>;
}
