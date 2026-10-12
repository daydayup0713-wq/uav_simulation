#!/usr/bin/python3
"""Explicit retention of unreferenced closed runs; never traverse symlinks."""
import argparse
from collections import defaultdict
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import time
import uuid

RUN=re.compile(r'\d{8}T\d{6}Z-[a-f0-9]{8}')
TEMPORARY=re.compile(r'policy-clean-checkout-[A-Za-z0-9_-]{1,64}')
POLICY={'retention_days':7,'successes_per_group':3,'failures_per_group':3}


def read(path):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return None


def tree_files(path):
    """Do not recurse into directory symlinks, including shared worktrees."""
    path=Path(path)
    if path.is_symlink():raise ValueError('symlink protected')
    if path.is_file():yield path;return
    for directory,names,files in os.walk(path,followlinks=False):
        for name in names+files:
            item=Path(directory)/name
            if item.is_symlink():raise ValueError('symlink protected: '+str(item))
        for name in files:yield Path(directory)/name


def identity(path):
    rows=[]
    root=Path(path)
    files=list(tree_files(root))
    directories=[Path(directory)/name for directory,names,_ in os.walk(root,followlinks=False) for name in names]
    for item in [root,*sorted(files+directories)]:
        stat=item.stat()
        rows.append([str(item.relative_to(root)),stat.st_dev,stat.st_ino,stat.st_size,stat.st_mtime_ns,stat.st_ctime_ns])
    return hashlib.sha256(json.dumps(rows).encode()).hexdigest()


def size(path):
    try:return sum(p.stat().st_size for p in tree_files(path))
    except (OSError,ValueError):return None


def usage(path):
    logical=allocated=links=0;seen=set()
    for directory,names,files in os.walk(path,followlinks=False):
        for name in names+files:
            item=Path(directory)/name
            if item.is_symlink():links+=1;continue
            if not item.is_file():continue
            info=item.stat();key=(info.st_dev,info.st_ino)
            if key in seen:continue
            seen.add(key);logical+=info.st_size;allocated+=info.st_blocks*512
    return {'bytes':logical,'allocated_bytes':allocated,'symlinks_retained':links,
        'scope':'unique regular files; symlinks are retained and not followed'}


def inspect(root):
    root=Path(root).resolve()
    return {'root':str(root),'categories':{name:usage(root/name) if not (root/name).is_symlink()
        else {'bytes':None,'scope':'shared symlink retained'}
        for name in ['scripts','ros2_ws/src','web/src','docs','.deps','.runtime','recordings','maps']
        if (root/name).exists()},'policy':POLICY}


def reference_closure(root):
    root=Path(root).resolve();runtime=root/'.runtime';protected={};pending=[];visited=set()
    for base in [root/'configs',root/'docs',runtime/'backend-evidence',runtime/'pair-evidence']:
        if base.is_dir():
            pending.extend(p for p in base.rglob('*') if p.is_file() and p.suffix in ('.json','.md') )
    # Raw recordings are retained; their origin manifests must remain traceable.
    if (root/'recordings').is_dir():pending.extend((root/'recordings').glob('*/dataset.json'))
    current=runtime/'current-run'
    if current.is_file():pending.append(current)
    def protect(path,why):
        try:
            path=Path(path)
            if not path.is_absolute():path=root/path
            path=Path(os.path.normpath(path))
            relative=path.relative_to(runtime)
        except ValueError:return
        if not relative.parts:return
        unit=runtime/relative.parts[0]
        protected[str(unit.relative_to(root))]=why
        if path.is_file() and path.suffix=='.json':pending.append(path)
    def strings(value,key=''):
        if isinstance(value,str):
            if RUN.fullmatch(value) and key in ('run_id','run'):protect(runtime/value,'reference: run identifier')
            for token in re.findall(r'(?:/[^\s"`<>\[\],]+)?\.runtime/[^\s"`<>\[\],)]+',value):protect(token,'reference: report/configuration')
        elif isinstance(value,dict):
            for k,v in value.items():strings(v,k)
        elif isinstance(value,list):
            for v in value:strings(v,key)
    while pending:
        path=Path(pending.pop())
        if str(path) in visited:continue
        visited.add(str(path))
        try:
            text=path.read_text()
            if path==current:protect(text.strip(),'current run')
            elif path.suffix=='.json':
                value=json.loads(text);strings(value)
            else:strings(text)
        except (OSError,ValueError,UnicodeError):
            # Unreadable roots make pruning unsafe: preserve every run.
            for run in runtime.glob('*'):
                if run.is_dir():protected[str(run.relative_to(root))]='unresolved reference root: '+str(path)
    return protected


def active(path,root):
    if (path/'ready').exists():return True
    value=read(path/'manifest.json') or {}
    try:
        pid=int(value['supervisor_pid'])
        args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        return any(str(root/'scripts/supervise.py').encode()==arg for arg in args)
    except (OSError,KeyError,TypeError,ValueError):return False


def plan(root,now=None):
    root=Path(root).resolve();now=time.time() if now is None else now
    runtime=root/'.runtime';protected=reference_closure(root);candidates=[];groups=defaultdict(list)
    if runtime.is_symlink():raise ValueError('runtime symlink cannot be pruned')
    for path in runtime.iterdir():
        relative=str(path.relative_to(root))
        if not path.is_dir() or not RUN.fullmatch(path.name):
            if relative not in protected and not TEMPORARY.fullmatch(path.name):protected[relative]='outside managed closed-run scope; retained'
            continue
        manifest=read(path/'manifest.json')
        if not manifest or active(path,root) or not (path/'closed').is_file():
            protected[relative]='active, incomplete or unknown run';continue
        try:
            created=datetime.strptime(path.name[:16],'%Y%m%dT%H%M%SZ').replace(tzinfo=timezone.utc).timestamp()
            selection=manifest.get('backend_selection') or {}
            group=tuple(selection.get(k) for k in ('localization','planning','localization_input_group'))+(manifest.get('scene'),manifest.get('profile'))
            log=(path/'bridge.log').read_text() if (path/'bridge.log').exists() else ''
            failed='FAILSAFE' in log or any('FAILED' in (read(p) or {}).get('state','') for p in path.glob('*readiness.json'))
            accepted=read(path/'acceptance.json') or read(path/'report.json') or {}
            result=accepted.get('success',accepted.get('passed'))
            outcome='failure' if failed or result is False else 'success' if result is True else 'unknown'
            if outcome=='unknown':protected[relative]='unknown outcome';continue
            groups[(group,outcome)].append((created,path))
            if now-created<7*86400:protected[relative]='last seven days'
            identity(path)
        except (OSError,ValueError,TypeError):protected[relative]='unreadable or symlink-containing run';continue
    for (_,outcome),rows in groups.items():
        rows.sort(reverse=True)
        for _,path in rows[:3]:protected.setdefault(str(path.relative_to(root)),'latest three '+outcome+' runs in group')
        for _,path in rows[3:]:
            relative=str(path.relative_to(root))
            if relative not in protected:
                candidates.append({'path':relative,'bytes':size(path),'identity':identity(path),
                    'reason':'closed, older than seven days, unreferenced, outside recent outcome retention'})
    if not any(active(p,root) for p in runtime.iterdir() if p.is_dir()):
        # Only disposable source caches, never installed libraries or binaries.
        caches=[root/'.pytest_cache']
        for path in runtime.iterdir():
            if not path.is_dir() or not TEMPORARY.fullmatch(path.name):continue
            relative=str(path.relative_to(root));metadata=read(path/'maintenance-disposable.json') or {}
            if (metadata.get('schema')!=1 or metadata.get('kind')!='source-checkout'
                    or metadata.get('root')!=str(root) or metadata.get('closed') is not True
                    or any((path/name).exists() for name in ('.deps','.git','build','install','recordings','maps'))):
                protected[relative]='temporary origin unknown or contains persistent dependency/data state';continue
            caches.append(path)
        for base in (root/'scripts',root/'tests',root/'ros2_ws/src'):
            if base.is_dir() and not base.is_symlink():
                caches.extend(p for p in base.rglob('__pycache__') if not any(parent.is_symlink() for parent in p.parents if parent!=root))
        for path in caches:
            if not path.exists():continue
            relative=str(path.relative_to(root))
            try:
                files=list(tree_files(path))
                newest=max([path.stat().st_mtime,*[p.stat().st_mtime for p in files]])
                if now-newest<7*86400:protected[relative]='cache used within seven days';continue
                if relative in protected:continue
                candidates.append({'path':relative,'bytes':size(path),'identity':identity(path),
                    'reason':'whitelisted reproducible source cache, unused for seven days'})
            except (OSError,ValueError):protected[relative]='unreadable or symlink-containing cache'
    return {'schema':1,'root':str(root),'created_at':now,'policy':POLICY,'candidates':candidates,
        'protected':[{'path':p,'bytes':size(root/p),'reason':why} for p,why in sorted(protected.items())],
        'estimated_logical_bytes':sum(row['bytes'] or 0 for row in candidates)}


def remove_owned(root,relative,expected):
    # Anchor every parent to opened directories, so renaming an ancestor or
    # replacing it with a symlink cannot redirect rmtree outside this project.
    if not getattr(shutil.rmtree,'avoids_symlink_attacks',False):raise ValueError('safe fd-based removal unavailable')
    flags=os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW
    descriptor=os.open(root,flags)
    try:
        for part in relative.parts[:-1]:
            child=os.open(part,flags,dir_fd=descriptor);os.close(descriptor);descriptor=child
        anchored=Path('/proc/self/fd')/str(descriptor)/relative.name
        if identity(anchored)!=expected:raise ValueError('object changed before anchored removal')
        shutil.rmtree(anchored)
    finally:os.close(descriptor)


def apply(root,proposal,now=None):
    root=Path(root).resolve();removed=[];refused=[];errors=[];bytes_removed=0
    before=shutil.disk_usage(root).free
    if proposal.get('schema')!=1 or proposal.get('root')!=str(root) or proposal.get('policy')!=POLICY:
        return {'success':False,'removed':[],'refused':['plan identity/policy mismatch'],'errors':[]}
    for item in proposal.get('candidates',[]):
        name=item.get('path','')
        try:
            path=root/name
            relative=Path(name)
            run_scope=path.parent==root/'.runtime' and RUN.fullmatch(path.name)
            temporary_scope=path.parent==root/'.runtime' and TEMPORARY.fullmatch(path.name)
            cache_scope=name=='.pytest_cache' or (relative.parts and relative.parts[0] in ('scripts','tests','ros2_ws') and path.name=='__pycache__')
            if relative.is_absolute() or '..' in relative.parts or not (run_scope or cache_scope or temporary_scope):
                raise ValueError('outside managed closed-run/cache scope')
            fresh={r['path']:r for r in plan(root,now)['candidates']}
            current=fresh.get(name)
            if not current or current['identity']!=item.get('identity'):raise ValueError('referenced, active, changed or no longer eligible')
            amount=size(path) or 0
            remove_owned(root,relative,item['identity'])
            removed.append(name);bytes_removed+=amount
        except ValueError as error:refused.append({'path':name,'reason':str(error)})
        except OSError as error:errors.append({'path':name,'reason':str(error)})
    return {'success':not refused and not errors,'removed':removed,'refused':refused,'errors':errors,
        'logical_bytes_removed':bytes_removed,'filesystem_free_delta_bytes':shutil.disk_usage(root).free-before,
        'space_scope':'free-space delta includes concurrent writes; logical bytes are completely removed managed objects'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('inspect','plan','apply'))
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--plan',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not (args.root/'dependencies/lock.json').is_file() or not (args.root/'README.md').is_file():parser.error('UAV Lab project root required')
    if args.command=='apply' and args.plan is None:parser.error('apply requires --plan')
    result=inspect(args.root) if args.command=='inspect' else plan(args.root) if args.command=='plan' else apply(args.root,read(args.plan) or {})
    if args.command=='apply':
        directory=args.root/'.runtime/maintenance';directory.mkdir(parents=True,exist_ok=True)
        audit=directory/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]+'.json')
        result['audit_path']=str(audit)
        with audit.open('x') as stream:json.dump({'plan':str(args.plan),'result':result},stream,indent=2)
    if args.output:
        with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2));return 0 if result.get('success',True) else 1


if __name__=='__main__':raise SystemExit(main())
