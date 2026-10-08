import type {Layer} from './Controls';
export type Cloud = {kind:'cloud';layer:Layer;frame:string;source_frame:string;stamp:number;count:number;source_count:number;display_decimated:boolean;invalid_sample_points?:number;display_transform_time_difference_s?:number;points:Float32Array};
export function decodeCloud(buffer:ArrayBuffer):Cloud{
 if(buffer.byteLength<4)throw Error('点云帧缺少长度');
 const length=new DataView(buffer).getUint32(0,true),offset=4+Math.ceil(length/4)*4;
 if(length>8192||offset>buffer.byteLength)throw Error('点云帧头超界');
 const header=JSON.parse(new TextDecoder().decode(new Uint8Array(buffer,4,length)));
 if(header.kind!=='cloud'||!['raw','registered','global','voxels'].includes(header.layer)||
  !Number.isInteger(header.count)||header.count<0||header.count>100000||offset+header.count*16!==buffer.byteLength)
  throw Error('点云帧格式或点数超界');
 return {...header,points:new Float32Array(buffer,offset,header.count*4)};
}
export type Vehicle={position:[number,number,number];quaternion:[number,number,number,number];stamp:number;age_s:number};
export type Diagnostic={level:number;message:string;values:Record<string,string>;age_s:number};
export type Telemetry={kind:'telemetry';run_id:string|null;simulation_time:number;vehicle:Vehicle|null;
 diagnostics:Record<string,Diagnostic>;layer_errors:Partial<Record<Layer,string>>;evaluation:boolean};
export type Backend={id:string;role:string;name?:string;stage:string;capabilities:Record<string,boolean|string>};
