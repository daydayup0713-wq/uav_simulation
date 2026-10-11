import {expect,it} from 'vitest';
import {decodeCloud} from './protocol';
function packet(count:number){const text=new TextEncoder().encode(JSON.stringify({kind:'cloud',layer:'raw',count,frame:'odom'}));
 const offset=4+Math.ceil(text.length/4)*4,buffer=new ArrayBuffer(offset+16*count);
 new DataView(buffer).setUint32(0,text.length,true);new Uint8Array(buffer,4,text.length).set(text);return buffer;}
it('读取有对齐填充的真实二进制格式，拒绝截断和超预算',()=>{
 expect(decodeCloud(packet(2)).points.length).toBe(8);
 expect(()=>decodeCloud(packet(2).slice(0,-1))).toThrow();
 expect(()=>decodeCloud(packet(100001))).toThrow();
});
