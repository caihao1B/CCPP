"""Coverage ordering is separate from point-to-point search.

Deterministic connected clustering, endpoint-safe GA/2-opt, metric Christofides,
and an actual edge-valid open MST/Euler walk. Never shortcut away required cells.
"""
from __future__ import annotations
from collections import deque
import math
import numpy as np
import networkx as nx
from scipy.cluster.vq import kmeans2
from .models import PlanningError


def connected_clusters(graph,max_size=48,method="bfs",seed=7):
    if max_size<1:raise ValueError("max_size must be positive")
    groups=[set(graph)]
    if method=="kmeans" and len(graph)>max_size:
        ids=sorted(graph);pts=np.array([graph.nodes[n]["pos"] for n in ids],dtype=float)
        _,labels=kmeans2(pts,int(math.ceil(len(ids)/max_size)),minit="++",seed=seed,iter=30)
        groups=[{ids[i] for i in range(len(ids)) if labels[i]==label} for label in sorted(set(labels))]
    elif method!="bfs" and method!="kmeans":raise ValueError("Unknown clustering method")
    result=[]
    for group in groups:
        unassigned=set(group)
        while unassigned:
            start=min(unassigned);unassigned.remove(start);cluster=[start];queue=deque([start])
            while queue and len(cluster)<max_size:
                u=queue.popleft()
                for v in sorted(graph[u]):
                    if v in unassigned:
                        unassigned.remove(v);cluster.append(v);queue.append(v)
                        if len(cluster)==max_size:break
            result.append(sorted(cluster))
    if set().union(*map(set,result))!=set(graph):raise PlanningError("Clustering lost targets")
    return result


def route_cost(route,D,closed=False):
    if len(route)<2:return 0.
    value=sum(float(D[a,b]) for a,b in zip(route,route[1:]))
    return value+(float(D[route[-1],route[0]]) if closed else 0.)


def two_opt(route,D,closed=False,fixed_goal=False,passes=8):
    """Endpoint-safe 2-opt; O(1) cost deltas for symmetric graph distances.

    The old candidate-by-candidate full route summation was unnecessarily slow.
    Asymmetric costs use an exact full evaluation rather than an invalid delta.
    """
    best=list(route);cost=route_cost(best,D,closed)
    stop=len(best)-1 if fixed_goal else len(best)
    symmetric=bool(np.allclose(D,np.asarray(D).T,atol=1e-12,rtol=1e-12))
    for _ in range(passes):
        improved=False
        for i in range(1,stop-1):
            for j in range(i+2,stop+1):
                if symmetric:
                    a,b,c=best[i-1],best[i],best[j-1]
                    delta=float(D[a,c]-D[a,b])
                    d=best[j] if j<len(best) else best[0] if closed else None
                    if d is not None:delta+=float(D[b,d]-D[c,d])
                    candidate_cost=cost+delta
                else:
                    candidate=best[:i]+list(reversed(best[i:j]))+best[j:]
                    candidate_cost=route_cost(candidate,D,closed)
                if candidate_cost<cost-1e-9:
                    best[i:j]=reversed(best[i:j]);cost=candidate_cost;improved=True
        if not improved:break
    return best


def tsp_order(D,start=0,goal=None,solver="ga",seed=7,population=48,generations=100):
    D=np.asarray(D,float);n=len(D)
    if D.shape!=(n,n) or not np.all(np.isfinite(D)) or np.any(D<0):
        raise ValueError("TSP requires a finite nonnegative square metric/cost matrix")
    if n==0:return []
    if start not in range(n) or (goal is not None and goal not in range(n)):raise ValueError("Invalid TSP endpoints")
    closed=goal==start if goal is not None else False
    if closed:goal=None
    interior=[i for i in range(n) if i!=start and i!=goal]
    def assemble(xs):return [start]+list(xs)+([goal] if goal is not None else [])
    if n<=2:return assemble(interior)
    # A feasible deterministic seed, with endpoints never included in crossover/mutation.
    nn=[];todo=set(interior);cur=start
    while todo:
        nxt=min(todo,key=lambda j:(D[cur,j],j));nn.append(nxt);todo.remove(nxt);cur=nxt
    seed_route=two_opt(assemble(nn),D,closed,goal is not None)
    if solver=="christofides":
        G=nx.Graph();G.add_nodes_from(range(n))
        for i in range(n):
            for j in range(i+1,n):G.add_edge(i,j,weight=float(D[i,j]))
        tour=nx.approximation.christofides(G,weight="weight")[:-1]
        k=tour.index(start);tour=tour[k:]+tour[:k]
        if goal is not None:tour=[x for x in tour if x!=goal]+[goal]
        # Endpoint-constrained open conversion is a heuristic, NOT a 3/2 guarantee.
        candidate=two_opt(tour,D,closed,goal is not None)
        return min((candidate,seed_route),key=lambda x:route_cost(x,D,closed))
    if solver!="ga":raise ValueError("solver must be ga or christofides")
    if population<4 or generations<1:raise ValueError("GA needs population >=4 and generations >=1")
    rng=np.random.default_rng(seed)
    genes=seed_route[1:-1] if goal is not None else seed_route[1:]
    pop=[genes[:]]+[rng.permutation(interior).tolist() for _ in range(population-1)]
    best=genes[:];best_cost=route_cost(assemble(best),D,closed);stale=0
    for generation in range(generations):
        pop.sort(key=lambda g:route_cost(assemble(g),D,closed))
        score=route_cost(assemble(pop[0]),D,closed)
        if score<best_cost-1e-9:best=pop[0][:];best_cost=score;stale=0
        else:stale+=1
        if stale>=20:break
        new=[best[:],pop[0][:]]
        def select():
            candidates=rng.choice(len(pop),size=min(3,len(pop)),replace=False)
            return pop[min(candidates,key=lambda i:route_cost(assemble(pop[int(i)]),D,closed))]
        while len(new)<population:
            a,b=select(),select()
            if len(a)<2:child=a[:]
            else:
                i,j=sorted(rng.choice(len(a)+1,2,replace=False).tolist())
                block=a[i:j];rest=[x for x in b if x not in block];child=rest[:i]+block+rest[i:]
                if rng.random()<.25:
                    u,v=rng.choice(len(child),2,replace=False);child[int(u)],child[int(v)]=child[int(v)],child[int(u)]
            new.append(child)
        pop=new
    candidates=[assemble(best),seed_route]+[assemble(g) for g in pop]
    route=min(candidates,key=lambda r:route_cost(r,D,closed))
    route=two_opt(route,D,closed,goal is not None)
    if set(route)!=set(range(n)) or len(route)!=n or route[0]!=start or (goal is not None and route[-1]!=goal):
        raise PlanningError("TSP endpoint/permutation invariant failed")
    return route


def graph_distance_matrix(graph,ids):
    D=np.zeros((len(ids),len(ids)))
    for i,u in enumerate(ids):
        d=nx.single_source_dijkstra_path_length(graph,u,weight="weight")
        for j,v in enumerate(ids):
            if v not in d:raise PlanningError("Required targets are disconnected in the graph")
            D[i,j]=d[v]
    return D


def mst_euler_walk(graph,start=None,goal=None):
    if not len(graph):return []
    if not nx.is_connected(graph):raise PlanningError("MST coverage graph disconnected; no teleport fallback")
    start=min(graph) if start is None else start
    if start not in graph or (goal is not None and goal not in graph):raise ValueError("MST endpoint not in graph")
    if len(graph)==1:return [start]
    tree=nx.minimum_spanning_tree(graph,weight="weight")
    if goal is None:
        distances=nx.single_source_dijkstra_path_length(tree,start,weight="weight")
        goal=max(distances,key=lambda n:(distances[n],n))
    odd={n for n in tree if tree.degree(n)%2}
    if start!=goal:odd.symmetric_difference_update({start,goal})
    matching_graph=nx.Graph();odd=sorted(odd);path_cache={}
    for i,u in enumerate(odd):
        d,paths=nx.single_source_dijkstra(graph,u,weight="weight")
        for v in odd[i+1:]:matching_graph.add_edge(u,v,weight=d[v]);path_cache[(u,v)]=paths[v]
    augmented=nx.MultiGraph(tree)
    for u,v in nx.min_weight_matching(matching_graph,weight="weight"):
        path=path_cache[(u,v)] if (u,v) in path_cache else list(reversed(path_cache[(v,u)]))
        for a,b in zip(path,path[1:]):augmented.add_edge(a,b,weight=graph[a][b]["weight"])
    walk=list(nx.eulerian_path(augmented,source=start))
    result=[start]+[v for _,v in walk]
    if result[-1]!=goal or not set(graph).issubset(result):raise PlanningError("Euler walk failed endpoint/coverage invariant")
    return result


def expand_order(order,graph):
    if not order:return []
    out=[order[0]]
    for a,b in zip(order,order[1:]):
        try:path=nx.shortest_path(graph,a,b,weight="weight")
        except nx.NetworkXNoPath as e:raise PlanningError("Order cannot be connected without crossing obstacles") from e
        out.extend(path[1:])
    if not set(order).issubset(out):raise PlanningError("Stitching lost a required waypoint")
    return out


def greedy_astar_coverage(graph,positions,start):
    from .search import astar_graph
    needed=set(graph);visited={start};route=[start];expanded=0
    while needed-visited:
        target=min(needed-visited,key=lambda n:(math.dist(positions[route[-1]],positions[n]),n))
        path,_,search=astar_graph(lambda u:((v,d["weight"]) for v,d in graph[u].items()),route[-1],target,positions)
        if not path:raise PlanningError(f"Target {target} is unreachable; it was NOT counted visited")
        route.extend(path[1:]);visited.update(path);expanded+=len(search)
    return route,expanded
