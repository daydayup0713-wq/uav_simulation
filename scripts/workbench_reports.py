"""Local measured evidence for Web/CLI; comparisons retain sensor/dataset groups."""
import argparse,bisect,copy,hashlib,json,math,re
from pathlib import Path


def read(path,default):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return default


def comparison(root):
    root=Path(root).resolve();directory=root/'docs/validation/l1'
    fingerprints={row['id']:row.get('implementation_sha256') for row in
        read(root/'configs/backends.json',{'backends':[]})['backends']}
    result=copy.deepcopy(read(directory/'localization-comparison.json',{'groups':[]}))
    result['scope']='Measured runs grouped by consumed inputs and exact dataset; no cross-group ranking or control qualification'
    for group in result['groups']:
        for case in group['cases']:
            for row in case['runs']:
                from export_backend_comparison import measured_scopes
                measured_scopes(row)
                row['current_implementation']=bool(row.get('implementation_sha256')) and row['implementation_sha256']==fingerprints.get(row.get('backend'))
                source=Path(row.get('source_report','/missing')).resolve()
                try:
                    valid=source.is_relative_to(root) and source.is_file() and hashlib.sha256(source.read_bytes()).hexdigest()==row.get('source_report_sha256')
                except OSError:valid=False
                row['source_verified']=bool(valid)
                row['resource_measurements']=[{'phase':phase,'mean_cpu_cores':item.get('mean_cpu_cores'),
                    'peak_rss_mib':item.get('peak_combined_rss_bytes',0)/1048576,'scope':item.get('scope','unspecified owned process scope')}
                    for phase,item in row.get('resource_summary',{}).items() if isinstance(item,dict)]
    result['planning']=read(directory/'planner-comparison.json',{'rows':[],'scope':'No measured planning report'})
    for row in result['planning']['rows']:
        cases=row.get('cases',[]);admitted=[c for c in cases if c.get('expected_success',c.get('purpose')!='rejection')]
        normal=[c for c in admitted if c.get('success')]
        row['success_rate']=sum(bool(c.get('success')) for c in admitted)/len(admitted) if admitted else None
        row['expected_outcome_rate']=sum(bool(c.get('passed',False)) for c in cases)/len(cases) if cases else None
        for output,key in [('mean_runtime_s','runtime_s'),('mean_length_m','length_m')]:
            values=[c[key] for c in normal if isinstance(c.get(key),(int,float))]
            row[output]=sum(values)/len(values) if values else None
    result['closed_loop']=read(directory/'flight-comparison.json',{'rows':[],'scope':'Native acceptance pending'})
    for row in result['closed_loop']['rows']:
        row['source_verified']=all(verified(root,a) for a in row.get('artifacts',[])) and bool(row.get('artifacts'))
        row['current_implementation']=flight_identity(root,row)
    return result


def runs(root):
    runtime=Path(root)/'.runtime';paths=list(runtime.glob('*/acceptance.json'))+list(runtime.glob('*/*/acceptance.json'))
    rows=[]
    for path in sorted(paths,key=lambda p:p.stat().st_mtime,reverse=True)[:100]:
        data=read(path,{})
        if 'passed' not in data:continue
        rows.append({'id':str(path.parent.relative_to(runtime)),'run_id':data.get('run_id'),
            'passed':data['passed'],'reason':data.get('reason',''),'runs':data.get('runs'),
            'backend_pair':data.get('backend_pair'),
            'report_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'source':str(path.relative_to(Path(root)))})
    return rows



def verified(root,artifact):
    try:
        path=Path(artifact['path']).resolve()
        return path.is_relative_to(root) and path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==artifact['sha256']
    except (OSError,KeyError,TypeError):return False


def flight_identity(root,row):
    root=Path(root)
    run=root/'.runtime'/str(row.get('run_id'))
    manifest=read(run/'manifest.json',{})
    from control_provenance import matches
    if not manifest.get('control_implementation') or not matches(root,manifest['control_implementation']):return False
    try:
        from qualify_pair import identity_checks
        identity_checks(root,run,manifest['backend_selection'])
        return True
    except (OSError,ValueError,KeyError,TypeError):return False


def flight_comparison(root):
    root=Path(root).resolve();result=[]
    for item in runs(root):
        source=root/item['source'];acceptance=read(source,{})
        pair=acceptance.get('backend_pair')
        if not isinstance(pair,dict) or not pair.get('planning'):continue
        directory=source.parent;tracking=read(directory/'tracking.json',{})
        geometry=read(directory/'geometry.json',{});handoffs=read(directory/'handoffs.json',{})
        current=flight_identity(root,item)
        artifacts=[{'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in
            [source,directory/'tracking.json',directory/'geometry.json',directory/'handoffs.json'] if p.is_file()]
        result.append({**pair,'backend':pair['localization']+' + '+pair['planning'],'run_id':item['run_id'],
            'report_id':item['id'],'runs':acceptance.get('runs'),'passed':acceptance['passed'],
            'reason':acceptance.get('reason',''),'current_implementation':current,'qualified':False,
            'source_verified':True,'artifacts':artifacts,'resources':acceptance.get('resources'),
            'tracking_p95_m':tracking.get('tracking_p95_m'),
            'minimum_body_clearance_m':geometry.get('min_body_clearance_m'),
            'handoffs':geometry.get('handoffs'),'successful_runs':geometry.get('successful_runs'),
            'analytic_maxima':geometry.get('accepted_curve_check',{}).get('analytic_maxima'),
            'success_rate':geometry.get('successful_runs',0)/acceptance['runs'] if acceptance.get('runs') else None,
            'scope':'Acceptance outcome; pair control qualification requires separate independent raw-evidence revalidation'})
    return {'schema':1,'scope':'Actual repeated PX4 runs and failed attempts, each with its own scene/source; historical passes do not qualify current control','rows':result}

def recordings(root):
    rows=[]
    for path in sorted((Path(root)/'recordings').glob('*/dataset.json'),reverse=True)[:100]:
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',path.parent.name):continue
        data=read(path,{})
        if data:
            rows.append({'id':path.parent.name,'complete':data.get('complete',False),
                'duration_s':data.get('duration_s'),'schema':data.get('schema_version'),
                'scope':'Recorded raw cloud and PX4 odometry; viewer never publishes ROS data'})
    return rows


def metrics(root,identifier):
    row=next((row for row in runs(root) if row['id']==identifier),None)
    if row is None:raise ValueError('unknown measured run identifier')
    source=Path(root)/row['source'];path=source.parent/'tracking.samples.json'
    if not path.is_file():return {'scope':'This run has no raw tracking samples','points':[]}
    if path.stat().st_size>64*1024*1024:raise ValueError('tracking samples exceed bounded display read')
    data=json.loads(path.read_text());refs=sorted(data['references'],key=lambda r:r['stamp']);times=[r['stamp'] for r in refs]
    points=[]
    for actual in data['actual']:
        t=actual['stamp'];upper=bisect.bisect_right(times,t)
        if not 0<upper<len(refs):continue
        left,right=refs[upper-1],refs[upper];gap=right['stamp']-left['stamp']
        if left['id']!=right['id'] or not 0<gap<=.100001:continue
        fraction=(t-left['stamp'])/gap
        def interpolate(key):return [a*(1-fraction)+b*fraction for a,b in zip(left[key],right[key])]
        points.append({'time_s':t,'error_m':math.dist(actual['position'],interpolate('position')),
            'speed_m_s':math.sqrt(sum(v*v for v in interpolate('velocity'))),
            'acceleration_m_s2':math.sqrt(sum(v*v for v in interpolate('acceleration')))})
    return {'scope':'Raw source-time tracking; interpolation within same trajectory ≤100ms; display decimated, full validation retained',
        'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'points':points[::max(1,math.ceil(len(points)/2000))]}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--flight-index',action='store_true')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    result=flight_comparison(root) if args.flight_index else {'comparison':comparison(root),'runs':runs(root),'recordings':recordings(root)}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({'output':str(args.output.resolve()),'groups':len(result.get('comparison',{}).get('groups',[]))}))


if __name__=='__main__':main()
