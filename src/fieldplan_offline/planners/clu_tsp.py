from __future__ import annotations
import math
from typing import List, Dict, Tuple
import networkx as nx

from fieldplan_offline.models.graph_square import build_cell_graph

def boundary_cells(G_full: nx.Graph, cluster: List[int]) -> List[int]:
    in_cl = set(cluster)
    return [u for u in cluster if any(v not in in_cl for v in G_full[u])]

def min_dist_between_sets(G: nx.Graph, srcs: List[int], tgts: set[int]) -> Tuple[int, Tuple[int, int]]:
    from heapq import heappush, heappop
    seen: Dict[int, Tuple[int,int]] = {}
    heap: List[Tuple[int,int,int]] = []
    for s in srcs:
        seen[s] = (0, s)
        heappush(heap, (0, s, s))
    while heap:
        d, u, origin = heappop(heap)
        if u in tgts:
            return d, (origin, u)
        if d != seen[u][0]:
            continue
        for v in G[u]:
            nd = d + 1
            if v not in seen or nd < seen[v][0]:
                seen[v] = (nd, origin)
                heappush(heap, (nd, v, origin))
    raise RuntimeError("No path between these sets")

def solve_cluster_tsp(grid, clusters: List[List[int]], min_gate_samples: int = 8, tsp_solver: str = "christofides") -> Tuple[List[int], Dict[Tuple[int, int], Tuple[int, int]]]:
    G_full = build_cell_graph(grid)
    Ei: Dict[int,List[int]] = {}
    for cid, cl in enumerate(clusters):
        bd = boundary_cells(G_full, cl)
        if len(bd) > min_gate_samples:
            step = math.ceil(len(bd) / min_gate_samples)
            bd = bd[::step]
        Ei[cid] = bd or cl

    m = len(clusters)
    Gc = nx.Graph()
    Gc.add_nodes_from(range(m))
    gate_pairs: Dict[Tuple[int,int], Tuple[int,int]] = {}

    for i in range(m):
        for j in range(i+1, m):
            dist, pair = min_dist_between_sets(G_full, Ei[i], set(Ei[j]))
            Gc.add_edge(i, j, weight=dist)
            gate_pairs[(i,j)] = gate_pairs[(j,i)] = pair

    if tsp_solver == "christofides":
        try:
            from networkx.algorithms.approximation import christofides
            tour = christofides(Gc, weight="weight")
        except Exception:
            from networkx.algorithms.approximation import greedy_tsp
            tour = greedy_tsp(Gc, weight="weight", source=0)
    else:
        from networkx.algorithms.approximation import greedy_tsp
        tour = greedy_tsp(Gc, weight="weight", source=0)

    if tour[0] == tour[-1]:
        tour = tour[:-1]
    return tour, gate_pairs
