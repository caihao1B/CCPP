"""Full student-paper coverage order layer: flat or connected clusters.

Algorithms solve a VISIT problem, not vehicle steering. Graph costs are weighted
metres. An exact Held-Karp solve is explicitly size-limited. No TSP name is a
placeholder. Christofides' 3/2 guarantee is NOT claimed for open/fixed-endpoint,
nonmetric cluster costs, or the later forward-vehicle conversion.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from collections import deque
import math,time
import numpy as np
import networkx as nx
from .models import Config, PlanningError
from .order_core import (route_cost,two_opt,tsp_order,graph_distance_matrix,mst_euler_walk,
                         greedy_astar_coverage,connected_clusters)

@dataclass
class OrderResult:
    order:list[int]
    raw_order:list[int]
    clusters:list[list[int]]
    cluster_order:list[int]
    gates:dict
    stats:dict

def nearest_order(D,start=0,goal=None):
    todo=set(range(len(D)))-{start};out=[start]
    if goal is not None and goal!=start:todo.discard(goal)
    while todo:
        v=min(todo,key=lambda n:(D[out[-1],n],n));out.append(v);todo.remove(v)
    if goal is not None and goal!=start:out.append(goal)
    return out

def held_karp(D,start=0,goal=None,closed=False,limit=15):
    """Exact Hamiltonian VISIT order on a finite cost matrix, O(n^2 2^n).
    This is not a Hamiltonian path in the underlying sparse cell graph.
    """
    D=np.asarray(D,float);n=len(D)
    if n>limit:raise PlanningError(f'exact Held-Karp has {n} nodes > limit {limit}; choose clusters or another solver')
    if n<=1:return [start] if n else []
    internal=[i for i in range(n) if i!=start and (goal is None or goal==start or i!=goal)]
    if not internal:return [start,goal]
    dp={};parent={};m=len(internal)
    for j,v in enumerate(internal):dp[(1<<j,j)]=float(D[start,v]);parent[(1<<j,j)]=None
    for mask in range(1,1<<m):
        for j in range(m):
            old=dp.get((mask,j))
            if old is None:continue
            for k in range(m):
                if mask&(1<<k):continue
                key=(mask|(1<<k),k);cost=old+D[internal[j],internal[k]]
                if cost<dp.get(key,math.inf):dp[key]=cost;parent[key]=(mask,j)
    full=(1<<m)-1
    end=goal if goal is not None else start if closed else None
    j=min(range(m),key=lambda j:dp[(full,j)]+(D[internal[j],end] if end is not None else 0))
    key=(full,j);out=[]
    while key is not None:
        mask,j=key;out.append(internal[j]);key=parent[key]
    out=[start]+list(reversed(out))
    if goal is not None and goal!=start:out.append(goal)
    return out

def matrix_order(D,solver,start=0,goal=None,cfg=None,seed=None,closed=False):
    cfg=cfg or Config();seed=cfg.seed if seed is None else seed;D=np.asarray(D,float)
    if D.ndim!=2 or D.shape[0]!=D.shape[1] or not np.all(np.isfinite(D)) or np.any(D<0):
        raise PlanningError('Ordering needs a finite nonnegative square cost matrix')
    if len(D)<=1:return [start] if len(D) else []
    if solver=='exact':return held_karp(D,start,goal,closed,cfg.exact_limit)
    route=nearest_order(D,start,goal)
    if solver=='nearest':return route
    if solver=='nn2opt':return two_opt(route,D,closed,goal is not None and goal!=start,passes=8)
    if solver in ('ga','christofides'):
        if solver=='christofides' and not np.allclose(D,D.T,atol=1e-10):
            raise ValueError('Christofides is only exposed for symmetric cost matrices; use GA/SA for directed costs')
        return tsp_order(D,start=start,goal=(start if closed else goal),solver=solver,seed=seed,
                         population=cfg.ga_population,generations=cfg.ga_generations)
    if solver!='sa':raise ValueError(f'Unknown matrix solver {solver}')
    rng=np.random.default_rng(seed);cur=two_opt(route,D,closed,goal is not None,passes=3)
    cost=route_cost(cur,D,closed);best=cur[:];bestcost=cost
    limit=len(cur)-1 if goal is not None and goal!=start else len(cur)
    if limit<=2:return best
    t0=max(cost/len(cur)*.5,1e-3)
    for i in range(cfg.sa_iterations):
        a,b=sorted(rng.choice(np.arange(1,limit),2,replace=False).tolist())
        cand=cur[:a]+cur[a:b+1][::-1]+cur[b+1:];c=route_cost(cand,D,closed)
        T=t0*(.003**(i/max(1,cfg.sa_iterations-1)))
        if c<cost or rng.random()<math.exp(min(0.,(cost-c)/max(T,1e-12))):cur,cost=cand,c
        if cost<bestcost:best,bestcost=cur[:],cost
    return two_opt(best,D,closed,goal is not None and goal!=start,passes=8)


def _bounded_groups(G,cfg,warnings):
    """A hard node cap bounds dense local costs. Every group remains connected."""
    size=cfg.cluster_size
    if cfg.solver=='exact':
        size=min(size,cfg.exact_limit)
        if size!=cfg.cluster_size:warnings.append({'stage':'partition','reason':'exact_local_size_cap','requested':cfg.cluster_size,'actual':size})
    if cfg.partition=='none':
        groups=[sorted(c) for c in nx.connected_components(G)]
        if any(len(g)>cfg.max_metric_nodes for g in groups):
            if cfg.fallback_policy=='strict':raise PlanningError('partition=none exceeds the local resource limit; use bfs or kmeans')
            warnings.append({'stage':'partition','reason':'resource_limit','requested':'none','actual':'bfs'})
            groups=connected_clusters(G,min(size,cfg.max_metric_nodes),'bfs',cfg.seed)
    else:groups=connected_clusters(G,min(size,cfg.max_metric_nodes),cfg.partition,cfg.seed)
    # Minimum size is soft. Merge across an actual edge without exceeding cap.
    groups=[set(g) for g in groups];labels={u:i for i,g in enumerate(groups) for u in g}
    for i in sorted(range(len(groups)),key=lambda i:(len(groups[i]),i)):
        if not groups[i] or len(groups[i])>=cfg.cluster_min:continue
        counts={}
        for u in groups[i]:
            for v in G[u]:
                j=labels[v]
                if j!=i and len(groups[i])+len(groups[j])<=size:counts[j]=counts.get(j,0)+1
        if counts:
            j=min(counts,key=lambda j:(-counts[j],len(groups[j]),j))
            for u in groups[i]:labels[u]=j
            groups[j].update(groups[i]);groups[i].clear()
    groups=[sorted(g) for g in groups if g]
    flat=[u for g in groups for u in g]
    if len(flat)!=len(set(flat)) or set(flat)!=set(G) or not all(nx.is_connected(G.subgraph(g)) for g in groups):
        raise PlanningError('Partition disjointness/coverage/connectivity invariant failed')
    return groups


def _dfs(G,start):
    stack=[(start,iter(sorted(G[start])))];seen={start};route=[start]
    while stack:
        _,it=stack[-1]
        try:v=next(it)
        except StopIteration:
            stack.pop()
            if stack:route.append(stack[-1][0])
            continue
        if v not in seen:
            seen.add(v);route.append(v);stack.append((v,iter(sorted(G[v]))))
    if seen!=set(G):raise PlanningError('DFS on disconnected local graph')
    return route


def local_order(G,grid,cfg,start,goal=None):
    ids=sorted(G)
    if len(ids)<=1:return ids
    method=cfg.solver
    if method in ('serpentine','column_sweep'):
        cells=grid.by_id;buckets={}
        for n in ids:
            key=cells[n].row if method=='serpentine' else round(grid.positions[n][0]/(grid.side*.5))
            buckets.setdefault(key,[]).append(n)
        route=[];axis=0 if method=='serpentine' else 1
        for k,key in enumerate(sorted(buckets)):
            route.extend(sorted(buckets[key],key=lambda n:(grid.positions[n][axis],n),reverse=bool(k%2)))
        # Starting from an explicit gate is required; rotate, don't drop nodes.
        i=route.index(start);route=route[i:]+route[:i]
        if goal is not None and goal!=start:route=[n for n in route if n!=goal]+[goal]
    elif method=='greedy':
        route,_=greedy_astar_coverage(G,grid.positions,start)
        if goal is not None and route[-1]!=goal:route+=nx.shortest_path(G,route[-1],goal,weight='weight')[1:]
    elif method=='mst_euler':route=mst_euler_walk(G,start,goal)
    elif method in ('dfs','double_tree'):
        route=_dfs(nx.minimum_spanning_tree(G,weight='weight') if method=='double_tree' else G,start)
        if goal is not None and route[-1]!=goal:route+=nx.shortest_path(G,route[-1],goal,weight='weight')[1:]
    else:
        D=graph_distance_matrix(G,ids)
        result=matrix_order(D,method,ids.index(start),ids.index(goal) if goal is not None and goal!=start else None,
                            cfg=cfg,closed=goal==start)
        route=[ids[i] for i in result]
    if not cfg.preserve_mst_walk and method in ('greedy','mst_euler','dfs','double_tree'):
        route=list(dict.fromkeys(route))
        if goal is not None and goal!=start:route=[n for n in route if n!=goal]+[goal]
    if goal==start and route[-1]!=start:route.append(start)
    if route[0]!=start or not set(ids).issubset(route) or (goal is not None and route[-1]!=goal):
        raise PlanningError('Local visit/endpoint invariant failed')
    return route


def build_order(grid,cfg,navigator,check=lambda:None):
    """Connected partitions, medoid tour, boundary gates, complete local visits.

    Inter-cluster medoid costs are an explicit proxy, not true joint gate/route
    optimization. Fixed endpoint Christofides is a heuristic, not a 3/2 result.
    """
    t0=time.perf_counter();G=grid.graph;warnings=[]
    groups=_bounded_groups(G,cfg,warnings);m=len(groups)
    pos=grid.positions;medoids=[]
    for group in groups:
        centre=np.mean([pos[n] for n in group],axis=0)
        medoids.append(min(group,key=lambda n:(math.dist(pos[n],centre),n)))
    start=cfg.start_id if cfg.start_id is not None else min(pos,key=lambda n:(pos[n][1],pos[n][0],n))
    if start not in G:raise PlanningError('Requested start is not a reachable safe task centre')
    first=next(i for i,g in enumerate(groups) if start in g)
    cost_method=cfg.inter_cost
    if m>cfg.max_cluster_matrix:
        if cfg.fallback_policy=='strict':
            raise PlanningError(f'{m} clusters exceed matrix budget {cfg.max_cluster_matrix}')
        warnings.append({'stage':'cluster_order','reason':'dense_matrix_budget_cap','requested':cfg.cluster_solver,
                         'actual':'nearest_streaming','nodes':m})
        # O(m) memory, O(m²) arithmetic. No global all-cell dense distance matrix.
        points=np.asarray([pos[n] for n in medoids]);todo=np.ones(m,dtype=bool);todo[first]=False;cs=[first]
        while np.any(todo):
            check();d=np.sum((points-points[cs[-1]])**2,axis=1);d[~todo]=np.inf
            j=int(np.argmin(d));cs.append(j);todo[j]=False
        actual_inter='nearest_streaming';cost_method='euclidean_medoid_streaming'
    else:
        D=np.zeros((m,m));cost_method=cfg.inter_cost
        if cost_method=='navigation_graph':
            anchors=[navigator.anchor(pos[n]) for n in medoids]
            if any(a is None for a in anchors):raise PlanningError('Medoid cannot attach to navigation graph')
            for i,u in enumerate(anchors):
                check();dist=nx.single_source_dijkstra_path_length(navigator.road.graph,u,weight='weight')
                for j,v in enumerate(anchors):
                    if i!=j:D[i,j]=math.dist(pos[medoids[i]],navigator.road.positions[u])+dist[v]+math.dist(pos[medoids[j]],navigator.road.positions[v])
        else:
            points=np.asarray([pos[n] for n in medoids]);D=np.linalg.norm(points[:,None,:]-points[None,:,:],axis=2)
        actual_inter=cfg.cluster_solver
        if actual_inter=='exact' and m>cfg.exact_limit:
            if cfg.fallback_policy=='strict':raise PlanningError(f'Exact inter-cluster TSP has {m} nodes > {cfg.exact_limit}')
            warnings.append({'stage':'cluster_order','requested':'exact','actual':'nn2opt','reason':'explicit_exponential_budget_cap','nodes':m})
            actual_inter='nn2opt'
        check();cs=matrix_order(D,actual_inter,start=first,cfg=cfg,closed=cfg.return_to_start)
    gates={};boundaries=[]
    for group in groups:
        own=set(group)
        bd=[u for u in group if any(v not in own for v in G[u])] or list(group)
        # Sparse candidate set; geometric lower-bound gate criterion is recorded.
        boundaries.append(sorted(bd)[::max(1,math.ceil(len(bd)/12))])
    for a,b in zip(cs,cs[1:]):
        u,v=min(((u,v) for u in boundaries[a] for v in boundaries[b]),key=lambda uv:(math.dist(pos[uv[0]],pos[uv[1]]),uv))
        gates[(a,b)]=(u,v)
    route=[];sections=[]
    for k,cid in enumerate(cs):
        check();entry=start if k==0 else gates[(cs[k-1],cid)][1]
        exit_=gates[(cid,cs[k+1])][0] if k+1<m else None
        local=local_order(G.subgraph(groups[cid]),grid,cfg,entry,exit_)
        sections.append({'cluster':cid,'entry':entry,'exit':local[-1],'nodes':local})
        route.extend(local)
    if cfg.return_to_start and route[-1]!=start:route.append(start)
    if set(route)!=set(G) or route[0]!=start:raise PlanningError('Global visit invariant failed')
    return OrderResult(route,route[:],groups,cs,{f'{a}->{b}':[u,v] for (a,b),(u,v) in gates.items()},
        {'ordering_seconds':time.perf_counter()-t0,'intra_solver':cfg.solver,'requested_inter_solver':cfg.cluster_solver,
         'actual_inter_solver':actual_inter,'partition':cfg.partition,'cluster_count':m,
         'cluster_sizes':[len(g) for g in groups],'medoid_cost_model':cost_method,
         'gate_cost_model':'Euclidean boundary-candidate proxy; every resulting motion is separately verified',
         'warnings':warnings,'sections':sections,'preserve_repeated_walk':cfg.preserve_mst_walk,
         'open_christofides_factor_claimed':False})
