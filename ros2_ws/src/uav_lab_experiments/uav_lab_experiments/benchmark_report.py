"""Measured resource aggregation; no process-start or readiness inference."""
import math


def resource_summary(rows):
    phases={}
    previous_wall=-math.inf
    for row in rows:
        wall=float(row['wall_s'])
        if not math.isfinite(wall) or wall<previous_wall:raise ValueError('resource clock regression')
        previous_wall=wall
        phase=phases.setdefault(row['phase'],{'first':wall,'last':wall,'peak':0,'processes':{}})
        phase['last']=wall
        total=0
        for process in row['processes']:
            cpu=float(process['cpu_s']);rss=int(process['rss']);pid=int(process['pid'])
            if not math.isfinite(cpu) or cpu<0 or rss<0:raise ValueError('invalid resource counter')
            counters=phase['processes'].setdefault(pid,{'first':cpu,'last':cpu,'peak':rss})
            if cpu<counters['last']:raise ValueError('CPU counter regression')
            counters['last']=cpu;counters['peak']=max(counters['peak'],rss);total+=rss
        phase['peak']=max(phase['peak'],total)
    output={}
    for name,phase in phases.items():
        duration=phase['last']-phase['first']
        cpu=sum(p['last']-p['first'] for p in phase['processes'].values())
        output[name]={'observed_wall_s':duration,'observed_cpu_s':cpu,
                      'mean_cpu_cores':cpu/duration if duration>0 else None,
                      'peak_combined_rss_bytes':phase['peak'],
                      'process_peak_rss_bytes':{str(pid):p['peak'] for pid,p in phase['processes'].items()},
                      'scope':'sampled owned algorithm, adapters and camera parameter process; excludes bag player'}
    return output
