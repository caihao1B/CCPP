import itertools, math
from dataclasses import replace
import numpy as np
import networkx as nx
import pytest
from shapely.geometry import Point
from ccpp_mowing.models import Config,Robot,SOLVERS,INTER_SOLVERS,PlanningError
from ccpp_mowing.maps import generate
from ccpp_mowing.geometry import World
from ccpp_mowing.grids import build_grid
from ccpp_mowing.order_core import connected_clusters,route_cost,two_opt,mst_euler_walk
from ccpp_mowing.ordering import matrix_order,held_karp,local_order,build_order
from ccpp_mowing.search import astar_graph,GridGraph,PRM,rrt,rrt_connect
from ccpp_mowing.navigation import Navigator

@pytest.mark.parametrize('method',SOLVERS)
def test_every_local_algorithm_keeps_endpoints_and_visits(method):
    cfg=Config(cell_area=2.,solver=method,exact_limit=14)
    g=build_grid(World(generate(width=6,height=6,density='none'),Robot()),cfg)
    # A connected 3x3 subgrid is small enough for actual exact TSP.
    nodes=list(nx.bfs_tree(g.graph,min(g.graph)))[:9];G=g.graph.subgraph(nodes)
    start=nodes[0];goal=nodes[-1]
    route=local_order(G,g,cfg,start,goal)
    assert route[0]==start and route[-1]==goal and set(route)==set(nodes)

@pytest.mark.parametrize('method',INTER_SOLVERS)
def test_all_matrix_solvers_and_exact_small_optimum(method):
    points=np.random.default_rng(5).uniform(0,10,(7,2));D=np.linalg.norm(points[:,None]-points[None,:],axis=2)
    result=matrix_order(D,method,start=0,goal=6,cfg=Config())
    assert sorted(result)==list(range(7)) and result[0]==0 and result[-1]==6
    if method=='exact':
        brute=min(route_cost([0,*p,6],D) for p in itertools.permutations(range(1,6)))
        assert route_cost(result,D)==pytest.approx(brute)

@pytest.mark.parametrize('method',['bfs','kmeans'])
def test_partition_complete_and_connected(method):
    G=nx.grid_2d_graph(10,8)
    for u in G:G.nodes[u]['pos']=u
    groups=connected_clusters(G,17,method,3)
    flat=[n for g in groups for n in g]
    assert set(flat)==set(G) and len(flat)==len(set(flat))
    assert all(len(g)<=17 and nx.is_connected(G.subgraph(g)) for g in groups)


def test_directed_2opt_uses_full_cost():
    D=np.random.default_rng(4).uniform(1,10,(9,9));np.fill_diagonal(D,0)
    route=list(range(9));new=two_opt(route,D,fixed_goal=True)
    assert new[0]==0 and new[-1]==8 and route_cost(new,D)<=route_cost(route,D)+1e-8


def test_mst_goal_not_premature_termination():
    G=nx.Graph();G.add_weighted_edges_from([(0,1,1),(1,2,1),(1,3,1),(3,4,1),(3,5,1)])
    route=mst_euler_walk(G,0,2)
    assert route[-1]==2 and set(route)==set(G)
    assert all(G.has_edge(a,b) for a,b in zip(route,route[1:]))


def test_astar_goal_discovery_not_termination():
    adj={0:[(1,2),(2,1)],1:[(3,3)],2:[(3,8)],3:[]};pos={n:(0,0) for n in adj}
    path,cost,_=astar_graph(lambda u:adj[u],0,3,pos,heuristic=False)
    assert path==[0,1,3] and cost==5


def test_astar_incoming_direction_state():
    G=nx.grid_2d_graph(5,5);pos={u:u for u in G}
    path,cost,_=astar_graph(lambda u:((v,1.) for v in G[u]),(0,0),(4,4),pos,turn_weight=2.)
    assert path[0]==(0,0) and path[-1]==(4,4)
    assert cost==pytest.approx(8+math.pi)

@pytest.mark.parametrize('planner',['astar','dijkstra','rrt','rrt_connect','rrt_star','prm'])
def test_nontrivial_obstacle_query(planner):
    m=generate(width=12,height=10,density='none');m.obstacles=[Point(6,5).buffer(1.1)]
    w=World(m,Robot());p,q=(2,5),(10,5)
    assert not w.segment_free(p,q)
    if planner in ('astar','dijkstra'):r=GridGraph(w,.45).query(p,q,heuristic=planner=='astar')
    elif planner=='prm':
        road=PRM(w,500,16,5);before=(len(road.graph),road.graph.number_of_edges())
        r=road.query(p,q);road.query(q,p)
        assert before==(len(road.graph),road.graph.number_of_edges())
    elif planner=='rrt_connect':r=rrt_connect(w,p,q,seed=5,iterations=4000,timeout=4.)
    else:r=rrt(w,p,q,seed=5,iterations=2000,timeout=4.,star=planner=='rrt_star')
    assert r.success and len(r.path)>2 and w.path_free(r.path)


def test_reported_fallback_is_not_rrt_success():
    m=generate(width=12,height=10,density='none');m.obstacles=[Point(6,5).buffer(1.1)]
    cfg=Config(planner='rrt',iterations=1,timeout=.1)
    n=Navigator(World(m,Robot()),cfg);path=n.route((2,5),(10,5))
    assert path and n.logs[-1]['fallback']=='astar' and n.logs[-1]['actual']=='astar_recovery'
    strict=Navigator(World(m,Robot()),replace(cfg,fallback_policy='strict'))
    with pytest.raises(PlanningError):strict.route((2,5),(10,5))


def test_exact_budget_has_explicit_substitution():
    cfg=Config(cell_area=.6,cluster_size=2,cluster_min=1,cluster_solver='exact',exact_limit=3)
    w=World(generate(width=10,height=8,density='none'),Robot());g=build_grid(w,cfg);n=Navigator(w,cfg)
    out=build_order(g,cfg,n)
    assert out.stats['actual_inter_solver']=='nn2opt' and out.stats['warnings']
    with pytest.raises(PlanningError):build_order(g,replace(cfg,fallback_policy='strict'),n)
