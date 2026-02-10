from __future__ import annotations
from typing import List, Dict, Sequence, Any
import networkx as nx

def _build_subgraph(targets: Sequence[Any], neighbor_map: Dict[Any, List[Any]]) -> nx.Graph:
    G = nx.Graph()
    for nid in targets:
        G.add_node(nid)
    for u in targets:
        for v in neighbor_map.get(u, []):
            if v in G and not G.has_edge(u, v):
                G.add_edge(u, v, weight=1)
    return G

def _odd_nodes(G: nx.Graph) -> List[Any]:
    return [n for n in G.nodes if G.degree(n) % 2 == 1]

def _augment_mst_with_matching(mst: nx.Graph, G_full: nx.Graph, odd: List[Any]) -> nx.MultiGraph:
    mst_aug = nx.MultiGraph(mst)
    if len(odd) <= 1:
        return mst_aug
    dist_cache = dict(nx.all_pairs_dijkstra_path_length(G_full, weight="weight"))
    match_graph = nx.Graph()
    for i, u in enumerate(odd):
        for v in odd[i+1:]:
            match_graph.add_edge(u, v, weight=-dist_cache[u][v])
    matching = nx.algorithms.matching.max_weight_matching(match_graph, maxcardinality=True)
    for u, v in matching:
        path_uv = nx.shortest_path(G_full, u, v, weight="weight")
        for a, b in zip(path_uv[:-1], path_uv[1:]):
            w = G_full[a][b].get("weight", 1)
            mst_aug.add_edge(a, b, weight=w)
    return mst_aug

class MSTEulerPlanner:
    def __init__(self, *, neighbor_map: Dict[Any, List[Any]]):
        self.neighbor_map = neighbor_map

    def plan(self, targets: Sequence[Any], start_id: Any | None = None, goal_id: Any | None = None) -> List[Any]:
        ids = list(targets)
        if len(ids) <= 2:
            return ids

        G = _build_subgraph(ids, self.neighbor_map)
        mst = nx.minimum_spanning_tree(G, weight="weight")
        odd = _odd_nodes(mst)
        mst_aug = _augment_mst_with_matching(mst, G, odd)

        if not nx.is_eulerian(mst_aug):
            return ids

        circuit = list(nx.eulerian_circuit(mst_aug))
        epath = [circuit[0][0]] + [v for _, v in circuit]

        if start_id is not None:
            if start_id not in epath:
                raise ValueError("start_id not in euler circuit")
            idx = epath.index(start_id)
            epath = epath[idx:] + epath[1:idx+1]

        if goal_id is not None and goal_id in epath:
            idx = max(i for i, n in enumerate(epath) if n == goal_id)
            epath = epath[:idx + 1]

        return epath
