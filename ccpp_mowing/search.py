"""Readable A*, Dijkstra, RRT, RRT-Connect, RRT* and reusable PRM.

All planners call the same World.segment_free. This low-level module never falls back. The application layer may use
explicitly logged A* recovery: failures retain their original reason. A* optimizes its finite graph,
not the un-discretized plane. RRT* is finite-budget, not a global certificate.
"""
from __future__ import annotations
from collections import defaultdict
import heapq, itertools, math, time
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
from .geometry import World, path_length, dedup
from .models import SearchResult, Config, PlanningError


def astar_graph(neighbors, start, goal, positions, heuristic=True, turn_weight=0.0):
    """Reopening A* with stale-queue removal; incoming direction is part of state
    when turn costs depend on it. ``neighbors(u)`` yields (v, nonnegative_cost).
    """
    if not math.isfinite(turn_weight) or turn_weight<0:raise ValueError("turn_weight must be finite/nonnegative")
    def h(u):return math.dist(positions[u],positions[goal]) if heuristic else 0.0
    serial=itertools.count();s=(None,start) if turn_weight else start
    best={s:0.};parent={};heap=[(h(start),next(serial),0.,s)];expanded=[]
    while heap:
        _,_,g,state=heapq.heappop(heap)
        if g>best.get(state,math.inf)+1e-10:continue
        u=state[1] if turn_weight else state
        expanded.append(u)
        if u==goal:
            states=[state]
            while state in parent:state=parent[state];states.append(state)
            ids=[q[1] if turn_weight else q for q in reversed(states)]
            return ids,g,expanded
        for v,cost in neighbors(u):
            if not math.isfinite(cost) or cost<0:raise ValueError("A* edge costs must be finite/nonnegative")
            extra=0.
            if turn_weight and state[0] is not None:
                a=np.asarray(positions[u])-positions[state[0]];b=np.asarray(positions[v])-positions[u]
                den=float(np.linalg.norm(a)*np.linalg.norm(b))
                if den>1e-12:extra=turn_weight*math.acos(float(np.clip(np.dot(a,b)/den,-1.,1.)))
            nxt=(u,v) if turn_weight else v;ng=g+cost+extra
            if ng+1e-10<best.get(nxt,math.inf):
                best[nxt]=ng;parent[nxt]=state
                heapq.heappush(heap,(ng+h(v),next(serial),ng,nxt))
    return [],math.inf,expanded


class RoadGraph:
    def __init__(self,world:World):
        self.world=world;self.graph=nx.Graph();self.positions={};self.build_seconds=0.

    def finish(self):
        self.ids=list(self.positions)
        self.array=np.array([self.positions[i] for i in self.ids],float).reshape(-1,2)
        self.tree=cKDTree(self.array) if len(self.ids) else None

    def edge_cost(self,a,b):
        return math.dist(a,b)

    def visible_neighbors(self,p,k=24):
        if self.tree is None:return []
        _,indices=self.tree.query(p,k=min(k,len(self.ids)))
        return [(self.ids[int(i)],self.edge_cost(p,self.positions[self.ids[int(i)]]))
                for i in np.atleast_1d(indices) if self.world.segment_free(p,self.positions[self.ids[int(i)]])]

    def query(self,start,goal,*,heuristic=True,turn_weight=0.):
        t0=time.perf_counter();world=self.world
        if not world.point_free(start) or not world.point_free(goal):
            return SearchResult(False,reason="invalid_endpoint",stats={"query_seconds":time.perf_counter()-t0})
        if math.dist(start,goal)<1e-9:return SearchResult(True,[tuple(start)],stats={"cost":0.,"expanded":0,"query_seconds":0.})
        # Endpoint overlay: no mutation of the reusable graph (important for PRM).
        a,b=-1,-2;pos={**self.positions,a:tuple(start),b:tuple(goal)}
        overlay=defaultdict(list)
        for q,p in ((a,start),(b,goal)):
            for n,c in self.visible_neighbors(p):overlay[q].append((n,c));overlay[n].append((q,c))
        if world.segment_free(start,goal):overlay[a].append((b,self.edge_cost(start,goal)));overlay[b].append((a,self.edge_cost(start,goal)))
        def neighbors(u):
            if u>=0:
                yield from ((v,float(d["weight"])) for v,d in self.graph[u].items())
            yield from overlay[u]
        route,cost,expanded=astar_graph(neighbors,a,b,pos,heuristic,turn_weight)
        stats=dict(query_seconds=time.perf_counter()-t0,expanded=len(expanded),cost=cost if route else None,
                   build_seconds=self.build_seconds,nodes=len(self.positions),edges=self.graph.number_of_edges(),
                   start_attachments=len(overlay[a]),goal_attachments=len(overlay[b]))
        if not route:return SearchResult(False,reason="disconnected_sampled_graph_or_endpoint_attachment",stats=stats)
        path=dedup([pos[i] for i in route])
        if not world.path_free(path):raise PlanningError("Internal graph path failed geometry validation")
        exploration=[]  # Do not duplicate a reusable road graph in every query.
        return SearchResult(True,path,stats=stats,exploration=exploration)


class GridGraph(RoadGraph):
    """Eight-neighbour metric grid; FULL edges checked, no corner tunnelling."""
    def __init__(self,world,resolution=.4,clearance_weight=0.,diagonal=True,max_nodes=150000):
        super().__init__(world);t0=time.perf_counter()
        if not math.isfinite(resolution) or resolution<=0:raise ValueError("Grid resolution must be >0")
        self.resolution=resolution;self.clearance_weight=clearance_weight;self.index={};self.lattice={}
        x0,y0,x1,y1=world.bounds
        nx_=int(math.ceil((x1-x0)/resolution));ny_=int(math.ceil((y1-y0)/resolution))
        if nx_*ny_>max_nodes:raise ValueError(f"Grid would have {nx_*ny_} candidates; increase resolution")
        for iy in range(ny_):
            for ix in range(nx_):
                p=(x0+(ix+.5)*resolution,y0+(iy+.5)*resolution)
                if not world.point_free(p):continue
                n=len(self.positions);self.positions[n]=p;self.index[(ix,iy)]=n;self.lattice[n]=(ix,iy)
                self.graph.add_node(n,pos=p)
        offsets=[(1,0),(0,1)]+([(1,1),(1,-1)] if diagonal else [])
        for (ix,iy),u in self.index.items():
            for dx,dy in offsets:
                v=self.index.get((ix+dx,iy+dy))
                if v is None:continue
                a,b=self.positions[u],self.positions[v]
                if not world.segment_free(a,b):continue
                length=math.dist(a,b)
                self.graph.add_edge(u,v,weight=self.edge_cost(a,b),length=length)
        self.finish();self.build_seconds=time.perf_counter()-t0


    def edge_cost(self,a,b):
        length=math.dist(a,b)
        if not self.clearance_weight:return length
        d=self.world.clearance_at(((a[0]+b[0])/2,(a[1]+b[1])/2))
        return length*(1+self.clearance_weight/max(d,.02))


class PRM(RoadGraph):
    def __init__(self,world,samples=800,k=18,seed=7):
        super().__init__(world);t0=time.perf_counter()
        if samples<1 or k<1:raise ValueError("PRM samples and k must be positive")
        rng=np.random.default_rng(seed)
        for i in range(samples):self.positions[i]=world.sample(rng);self.graph.add_node(i)
        self.finish()
        _,indices=self.tree.query(self.array,k=min(k+1,samples))
        if samples==1:indices=np.asarray(indices).reshape(1,1)
        for u,ns in enumerate(indices):
            for v in np.atleast_1d(ns):
                v=int(v)
                if u==v or self.graph.has_edge(u,v):continue
                a,b=self.positions[u],self.positions[v]
                if world.segment_free(a,b):self.graph.add_edge(u,v,weight=math.dist(a,b))
        self.build_seconds=time.perf_counter()-t0


def _steer(a,b,step):
    a=np.asarray(a,float);b=np.asarray(b,float);d=float(np.linalg.norm(b-a))
    return tuple(a+(b-a)*min(1.,step/d)) if d>1e-12 else tuple(a)


def _unroll(nodes,parents,j):
    route=[]
    while j>=0:route.append(nodes[j]);j=parents[j]
    return list(reversed(route))


def rrt(world,start,goal,*,seed=7,step=.65,iterations=6000,goal_bias=.12,timeout=20.,star=False,direct_connection=False):
    Config(seed=seed,rr_step=step,iterations=iterations,timeout=timeout,goal_bias=goal_bias).validate()
    t0=time.perf_counter()
    if not world.point_free(start) or not world.point_free(goal):return SearchResult(False,reason="invalid_endpoint")
    if math.dist(start,goal)<1e-9:return SearchResult(True,[tuple(start)],stats={"query_seconds":0.,"nodes":1})
    if direct_connection and world.segment_free(start,goal):return SearchResult(True,[tuple(start),tuple(goal)],stats={"direct_visible":True,"nodes":2,"query_seconds":time.perf_counter()-t0})
    rng=np.random.default_rng(seed);nodes=[tuple(start)];parents=[-1];costs=[0.];children=defaultdict(set)
    storage=np.empty((iterations+1,2));storage[0]=start;goal_candidates=[];first=None;iteration=0
    for iteration in range(iterations):
        if time.perf_counter()-t0>timeout:break
        sample=goal if rng.random()<goal_bias else world.sample(rng)
        d2=np.sum((storage[:len(nodes)]-sample)**2,axis=1);near=int(np.argmin(d2))
        new=_steer(nodes[near],sample,step)
        if math.dist(new,nodes[near])<1e-8 or not world.segment_free(nodes[near],new):continue
        parent=near;best=costs[near]+math.dist(nodes[near],new);neighbors=[]
        if star:
            n=len(nodes)+1
            # 2-D shrinking radius, limited by local steering range.
            diameter=math.hypot(world.bounds[2]-world.bounds[0],world.bounds[3]-world.bounds[1])
            radius=min(3*step,2*diameter*math.sqrt(math.log(max(n,2))/n))
            neighbors=np.where(np.sum((storage[:len(nodes)]-new)**2,axis=1)<=radius**2)[0].tolist()
            for j in neighbors:
                alt=costs[j]+math.dist(nodes[j],new)
                if alt+1e-9<best and world.segment_free(nodes[j],new):parent=j;best=alt
        nid=len(nodes);nodes.append(new);storage[nid]=new;parents.append(parent);costs.append(best);children[parent].add(nid)
        if star:
            for j in neighbors:
                if j==0 or j==parent:continue
                alt=best+math.dist(new,nodes[j])
                if alt+1e-9>=costs[j] or not world.segment_free(new,nodes[j]):continue
                # Positive edge costs prohibit rewiring an ancestor through its descendant.
                ancestor=parent;cycle=False
                while ancestor>=0:
                    if ancestor==j:cycle=True;break
                    ancestor=parents[ancestor]
                if cycle:continue
                delta=alt-costs[j];children[parents[j]].discard(j);parents[j]=nid;children[nid].add(j)
                stack=[j]
                while stack:
                    cur=stack.pop();costs[cur]+=delta;stack.extend(children[cur])
        if math.dist(new,goal)<=step and world.segment_free(new,goal):
            goal_candidates.append(nid)
            if first is None:first=time.perf_counter()-t0
            if not star:break
    stats=dict(query_seconds=time.perf_counter()-t0,build_seconds=0.,nodes=len(nodes),iterations=iteration+1,
               first_solution_seconds=first,seed=seed,budget_exhausted=not bool(goal_candidates))
    edges=[[nodes[parents[i]],nodes[i]] for i in range(1,min(len(nodes),3001))]
    if not goal_candidates:return SearchResult(False,reason="sampling_budget_exhausted_not_proof_of_infeasibility",stats=stats,exploration=edges)
    j=min(goal_candidates,key=lambda k:costs[k]+math.dist(nodes[k],goal))
    path=dedup(_unroll(nodes,parents,j)+[tuple(goal)])
    if not world.path_free(path):raise PlanningError("RRT internal path validation failed")
    stats["cost"]=path_length(path)
    return SearchResult(True,path,stats=stats,exploration=edges)


def rrt_connect(world,start,goal,*,seed=7,step=.65,iterations=6000,timeout=20.,**_):
    Config(seed=seed,rr_step=step,iterations=iterations,timeout=timeout).validate()
    t0=time.perf_counter();rng=np.random.default_rng(seed)
    if not world.point_free(start) or not world.point_free(goal):return SearchResult(False,reason="invalid_endpoint")
    if math.dist(start,goal)<1e-9:return SearchResult(True,[tuple(start)],stats={"query_seconds":0.})
    if world.segment_free(start,goal):return SearchResult(True,[tuple(start),tuple(goal)],stats={"direct_visible":True,"query_seconds":time.perf_counter()-t0})
    trees=[([tuple(start)],[-1]),([tuple(goal)],[-1])];joined=None
    def extend(tree,target):
        nodes,parents=tree;j=int(np.argmin(np.sum((np.asarray(nodes)-target)**2,axis=1)))
        new=_steer(nodes[j],target,step)
        if math.dist(nodes[j],new)<1e-9:return j,True
        if not world.segment_free(nodes[j],new):return None,False
        nodes.append(new);parents.append(j)
        return len(nodes)-1,math.dist(new,target)<1e-8
    for k in range(iterations):
        if time.perf_counter()-t0>timeout:break
        a=k%2;b=1-a;sample=world.sample(rng);i,_=extend(trees[a],sample)
        if i is None:continue
        target=trees[a][0][i]
        while time.perf_counter()-t0<=timeout:
            j,reached=extend(trees[b],target)
            if j is None:break
            if reached:
                joined=(i,j) if a==0 else (j,i);break
        if joined:break
    edges=[]
    for nodes,parents in trees:
        edges.extend([[nodes[parents[i]],nodes[i]] for i in range(1,min(len(nodes),1501))])
    stats=dict(query_seconds=time.perf_counter()-t0,iterations=k+1,nodes=sum(len(t[0]) for t in trees),seed=seed)
    if not joined:return SearchResult(False,reason="sampling_budget_exhausted_not_proof_of_infeasibility",stats=stats,exploration=edges)
    a=_unroll(*trees[0],joined[0]);b=_unroll(*trees[1],joined[1]);path=dedup(a+list(reversed(b)))
    if not world.path_free(path):raise PlanningError("RRT-Connect path validation failed")
    stats["cost"]=path_length(path)
    return SearchResult(True,path,stats=stats,exploration=edges)

