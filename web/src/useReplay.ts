import {useEffect,useState} from 'react';
import {apiBuffer} from './api';
import {decodeCloud} from './protocol';
import type {Cloud} from './protocol';

/** Read-only latest-frame loading; playback never advances over an unfinished read. */
export function useReplay(mode:string,identifier:string|undefined,replayIndex:number){
 const [replayCloud,setReplayCloud]=useState<Cloud|null>(null),[replayLoading,setReplayLoading]=useState(false),[loadedFrame,setLoadedFrame]=useState('');
 const [replayError,setReplayError]=useState(''),[refresh,setRefresh]=useState(0);
 useEffect(()=>{setReplayCloud(null);setLoadedFrame('');},[mode,identifier]);
 useEffect(()=>{
  if(mode!=='replay'||!identifier){setReplayLoading(false);return;}
  let current=true;const controller=new AbortController();setReplayLoading(true);setReplayError('');
  // Scrubbing keeps only the latest requested index; playback waits for its read.
  const timer=setTimeout(()=>{void apiBuffer('replay/frame?index='+replayIndex,controller.signal)
   .then(buffer=>{if(current){setReplayCloud(decodeCloud(buffer));setLoadedFrame(identifier+':'+replayIndex);}})
   .catch(e=>{if(current&&e.name!=='AbortError'){setReplayError(e.message);}})
   .finally(()=>{if(current)setReplayLoading(false);});},100);
  return()=>{current=false;clearTimeout(timer);controller.abort();};
 },[mode,replayIndex,identifier,refresh]);
 return {cloud:replayCloud,loading:replayLoading,loadedFrame,error:replayError,retry:()=>setRefresh(value=>value+1)};
}
