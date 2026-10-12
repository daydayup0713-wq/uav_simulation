import {useEffect,useState} from 'react';
import {api} from './api';
import type {ComparisonData,RecordedRun,Recording} from './Workbench';

export function useReports(tab:string,mode:string,enabled:boolean){
 const [comparison,setComparison]=useState<ComparisonData|null>(null),[runs,setRuns]=useState<RecordedRun[]>([]),[recordings,setRecordings]=useState<Recording[]>([]);
 const [loading,setLoading]=useState(false),[error,setError]=useState(''),[refresh,setRefresh]=useState(0);
 useEffect(()=>{
  if(!enabled||(tab==='空间观测'&&mode!=='replay'))return;
  let closed=false,pending=false,failed=false;
  setError('');
  const update=async()=>{if(pending||failed)return;pending=true;setLoading(true);
   try{
    const reads:Promise<void>[]=[];
    if(tab==='算法对照')reads.push(api<ComparisonData>('comparison').then(value=>{if(!closed)setComparison(value);}));
    if(tab==='实验记录')reads.push(api<RecordedRun[]>('runs').then(value=>{if(!closed)setRuns(value);}));
    if(mode==='replay')reads.push(api<Recording[]>('recordings').then(value=>{if(!closed)setRecordings(value);}));
    await Promise.all(reads);
   }catch(e){failed=true;if(!closed)setError(e instanceof Error?e.message:'报告读取失败');}
   finally{pending=false;if(!closed)setLoading(false);}};
  void update();const timer=setInterval(update,15000);
  return()=>{closed=true;clearInterval(timer);};
 },[tab,mode,enabled,refresh]);
 return {comparison,runs,recordings,loading,error,retry:()=>setRefresh(value=>value+1)};
}
