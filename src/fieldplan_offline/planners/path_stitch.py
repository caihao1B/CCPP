from __future__ import annotations
from typing import List, Set, Any
import time, networkx as nx

def _hamilton_dfs(G: nx.Graph, start: Any, goal: Any, timeout: float = 0.05) -> List[Any] | None:
    n = G.number_of_nodes()
    t0 = time.time()
    order: list[Any] = [start]
    visited: set[Any] = {start}

    def _sorted_nbr(u):
        return sorted(G.neighbors(u), key=lambda v: G.degree(v))

    def _dfs(u):
        if time.time() - t0 > timeout:
            return False
        if len(order) == n:
            return u == goal
        for v in _sorted_nbr(u):
            if v in visited:
                continue
            order.append(v); visited.add(v)
            if _dfs(v):
                return True
            order.pop(); visited.remove(v)
        return False

    if _dfs(start):
        return order
    return None

def _penalized_shortest_path(G: nx.Graph, src: Any, dst: Any, visited: Set[Any], gamma: int = 1000) -> List[Any]:
    def w(u, v, d):
        return 1 + (gamma if v in visited else 0)
    return nx.dijkstra_path(G, src, dst, weight=w)

def _local_patch(path: List[Any], G: nx.Graph, gamma: int = 1000, max_iter: int = 6) -> List[Any]:
    for _ in range(max_iter):
        dup_pos = next((i for i, x in enumerate(path) if x in path[:i]), None)
        if dup_pos is None:
            break
        a = path[max(0, dup_pos-2)]
        c = path[min(len(path)-1, dup_pos+3)]
        visited_now = set(path[:dup_pos])
        alt = _penalized_shortest_path(G, a, c, visited_now, gamma)
        old_len = dup_pos+3 - (dup_pos-2)
        if len(alt) < old_len:
            path = path[:dup_pos-2] + alt + path[dup_pos+3:]
    return path

def stitch(node_seq: List[Any], graph: nx.Graph, gamma: int = 1000, patch: bool = True) -> List[Any]:
    if not node_seq:
        return []
    if graph.number_of_nodes() > 300:
        ham = None
    else:
        ham = _hamilton_dfs(graph, node_seq[0], node_seq[-1], timeout=0.05)
    if ham:
        return ham

    full = [node_seq[0]]
    visited = {node_seq[0]}
    for u, v in zip(node_seq, node_seq[1:]):
        seg = _penalized_shortest_path(graph, u, v, visited, gamma)
        full.extend(seg[1:]); visited.update(seg)
    if patch:
        full = _local_patch(full, graph, gamma)
    return full
