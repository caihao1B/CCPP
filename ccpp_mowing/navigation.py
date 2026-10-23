"""Reusable global transit graph; visited grass is never removed from this graph."""
from __future__ import annotations
import hashlib, math, time
import networkx as nx
import numpy as np
from scipy.spatial import cKDTree
from .search import GridGraph, PRM, rrt, rrt_connect
from .models import SearchResult, PlanningError
from .geometry import dedup, path_length

class Navigator:
    def __init__(self,world,cfg,check=lambda:None):
        self.world=world;self.cfg=cfg;self.check=check
        self.road=GridGraph(world,cfg.navigation_resolution,max_nodes=160000)
        self.prm=None;self.logs=[];self.cache={};self.attach_cache={};self.bridge_log=[]
        self._join_straight_components()
        self.component={u:i for i,comp in enumerate(nx.connected_components(self.road.graph)) for u in comp}
        self.components=[set(c) for c in nx.connected_components(self.road.graph)]
        self.reachable=None

    def _join_straight_components(self):
        """Small deterministic bridge search for straight narrow gates.
        Pivot endpoints must be safe; an exact swept rectangle checks the edge.
        Not a complete SE(2) planner. Rejected gaps remain explicitly excluded.
        """
        comps=list(nx.connected_components(self.road.graph))
        if len(comps)<2:return
        for i,group in enumerate(comps):
            self.check()
            if len(comps)>30:break
            a=sorted(group);pa=np.asarray([self.road.positions[u] for u in a]);ta=cKDTree(pa)
            for other in comps[i+1:]:
                b=sorted(other);pb=np.asarray([self.road.positions[v] for v in b]);ds,inds=ta.query(pb)
                for k in np.argsort(ds)[:24]:
                    if ds[k]>6.:continue
                    u,v=a[int(inds[k])],b[int(k)]
                    if self.world.segment_free(self.road.positions[u],self.road.positions[v]):
                        self.road.graph.add_edge(u,v,weight=float(ds[k]),narrow_straight_bridge=True)
                        self.bridge_log.append({'u':u,'v':v,'length':float(ds[k]),'certificate':'rectangular_translation_between_safe_pivots'})
                        break

    def anchor(self,p):
        p=tuple(p)
        if p not in self.attach_cache:
            candidates=self.road.visible_neighbors(p,k=48)
            if self.reachable is not None:candidates=[(u,c) for u,c in candidates if u in self.reachable]
            self.attach_cache[p]=min(candidates,key=lambda uc:(uc[1],uc[0]))[0] if candidates else None
        return self.attach_cache[p]

    def restrict_tasks(self,grid):
        attachments={u:self.anchor(p) for u,p in grid.positions.items()}
        counts={}
        for a in attachments.values():
            if a is not None:counts[self.component[a]]=counts.get(self.component[a],0)+1
        if not counts:return [],[{'id':u,'center':p,'reason':'no_navigation_attachment'} for u,p in grid.positions.items()]
        if self.cfg.start_id is not None:
            a=attachments.get(self.cfg.start_id)
            if a is None:raise PlanningError('Explicit start is not attachable; refusing to relocate it')
            chosen=self.component[a]
        else:chosen=max(counts,key=lambda i:(counts[i],-i))
        self.reachable=self.components[chosen];self.attach_cache.clear()
        keep=[];skipped=[]
        for u,p in grid.positions.items():
            a=attachments[u]
            if a is None:reason='navigation_resolution_no_attachment'
            elif self.component[a]!=chosen:reason='different_certified_navigation_component'
            else:keep.append(u);continue
            skipped.append({'id':u,'center':p,'reason':reason})
        return keep,skipped

    def shortcut(self,path):
        out=[path[0]];i=0
        while i<len(path)-1:
            # Limit the greedy horizon to bound quadratic work on long routes.
            j=min(i+100,len(path)-1)
            while j>i+1 and not self.world.segment_free(path[i],path[j]):j-=1
            out.append(path[j]);i=j
        return dedup(out)

    def route(self,start,goal):
        self.check();p,q=tuple(start),tuple(goal);cfg=self.cfg;t0=time.perf_counter()
        key=(p,q,cfg.planner,cfg.direct_visible)
        record={'requested':cfg.planner,'start':p,'goal':q,'fallback':None}
        if key in self.cache:
            path,actual=self.cache[key];record.update(actual=actual,cache_hit=True,seconds=time.perf_counter()-t0,success=True)
            self.logs.append(record);return path[:]
        if math.dist(p,q)<1e-10:result=SearchResult(True,[p]);actual='identity'
        elif cfg.direct_visible and self.world.segment_free(p,q):result=SearchResult(True,[p,q]);actual='verified_visibility'
        else:
            actual=cfg.planner
            if actual in ('astar','dijkstra'):result=self.road.query(p,q,heuristic=actual=='astar')
            elif actual=='prm':
                if self.prm is None:self.prm=PRM(self.world,cfg.prm_samples,cfg.prm_neighbors,cfg.seed)
                result=self.prm.query(p,q)
            else:
                seed=int.from_bytes(hashlib.sha256(repr((cfg.seed,p,q)).encode()).digest()[:4],'little')
                kw=dict(seed=seed,step=cfg.rr_step,iterations=cfg.iterations,timeout=cfg.timeout)
                if actual=='rrt_connect':result=rrt_connect(self.world,p,q,**kw)
                else:result=rrt(self.world,p,q,goal_bias=cfg.goal_bias,star=actual=='rrt_star',**kw)
            record['primary_stats']=result.stats
            if not result.success:
                record['primary_failure']=result.reason
                if cfg.fallback_policy=='astar_reported' and actual not in ('astar','dijkstra'):
                    result=self.road.query(p,q);record['fallback']='astar';actual='astar_recovery'
        if not result.success:
            record.update(actual=actual,success=False,reason=result.reason,seconds=time.perf_counter()-t0)
            self.logs.append(record);raise PlanningError(f'Connection failed: {result.reason}')
        path=dedup(result.path)
        if cfg.shortcut_transit and len(path)>2:path=self.shortcut(path)
        if not self.world.path_free(path):raise PlanningError('Connector output violates swept-footprint certificate')
        self.cache[key]=(path,actual)
        record.update(actual=actual,success=True,seconds=time.perf_counter()-t0,length_m=path_length(path))
        self.logs.append(record)
        return path[:]
