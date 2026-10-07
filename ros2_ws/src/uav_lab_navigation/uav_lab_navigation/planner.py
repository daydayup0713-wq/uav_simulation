"""Bounded 26-neighbor A*, with conservative edges and checked shortcuts."""
from dataclasses import dataclass, field
from itertools import product
import heapq
import math
import time
import numpy as np
from .occupancy import vector


@dataclass
class Plan:
    success: bool
    reason: str
    points: list = field(default_factory=list)
    expanded: int = 0


def plan(collision, start, goal, max_expansions=50000, timeout=2.):
    try: start,goal=vector(start),vector(goal)
    except (ValueError,TypeError): return Plan(False,'INVALID_GOAL')
    if not isinstance(max_expansions,int) or max_expansions<=0 or not math.isfinite(timeout) or timeout<=0:
        return Plan(False,'INVALID_SEARCH_BUDGET')
    first,last=collision.index(start),collision.index(goal)
    if any(not all(0<=v<s for v,s in zip(p,collision.free.shape)) for p in (first,last)):
        return Plan(False,'OUTSIDE_BOUNDS')
    if not collision.point_clear(start): return Plan(False,'START_BLOCKED_OR_UNOBSERVED')
    if not collision.point_clear(goal): return Plan(False,'GOAL_BLOCKED_OR_UNOBSERVED')
    if collision.segment_clear(start,goal): return Plan(True,'DIRECT',[start.tolist(),goal.tolist()])
    deadline=time.monotonic()+timeout
    h=lambda key: math.dist(key,last)
    frontier=[(h(first),0.,first)]; costs={first:0.}; parent={}; closed=set(); expanded=0
    offsets=[o for o in product((-1,0,1),repeat=3) if any(o)]
    while frontier:
        if expanded>=max_expansions or time.monotonic()>deadline:
            return Plan(False,'SEARCH_BUDGET_EXCEEDED',expanded=expanded)
        _,cost,current=heapq.heappop(frontier)
        if current in closed: continue
        closed.add(current); expanded+=1
        if current==last:
            keys=[last]
            while keys[-1]!=first: keys.append(parent[keys[-1]])
            points=[start]+[collision.center(k) for k in reversed(keys)]+[goal]
            # Verify exact request-to-cell connectors, too.
            if not all(collision.segment_clear(a,b) for a,b in zip(points,points[1:])):
                return Plan(False,'ENDPOINT_CONNECTOR_BLOCKED',expanded=expanded)
            smooth=[points[0]]; index=0
            while index<len(points)-1:
                if time.monotonic()>deadline: return Plan(False,'SEARCH_BUDGET_EXCEEDED',expanded=expanded)
                next_index=len(points)-1
                while next_index>index+1 and not collision.segment_clear(points[index],points[next_index]):
                    next_index-=1
                smooth.append(points[next_index]);index=next_index
            return Plan(True,'PLANNED',[p.tolist() for p in smooth],expanded)
        for offset in offsets:
            neighbor=tuple(c+d for c,d in zip(current,offset))
            if neighbor in closed or not collision.cell_clear(neighbor): continue
            candidate=cost+math.sqrt(sum(d*d for d in offset))
            if candidate>=costs.get(neighbor,math.inf): continue
            if not collision.segment_clear(collision.center(current),collision.center(neighbor)): continue
            costs[neighbor]=candidate;parent[neighbor]=current
            heapq.heappush(frontier,(candidate+h(neighbor),candidate,neighbor))
    return Plan(False,'NO_PATH',expanded=expanded)
