from __future__ import annotations

import math
from typing import List, Tuple, Dict, Optional

import networkx as nx
import numpy as np
from shapely.geometry import Polygon

from fieldplan_offline.models.graph_square import build_cell_graph

try:
    from sklearn.cluster import KMeans  # optional
except ModuleNotFoundError:
    KMeans = None  # type: ignore

try:
    import pymetis  # optional
except ModuleNotFoundError:
    pymetis = None  # type: ignore


def compute_metrics(field: Polygon, obstacles: List[Polygon]) -> Tuple[float, float]:
    """Return (shape_complexity, obstacle_area_ratio)."""
    c_shape = field.length / (2 * math.sqrt(math.pi * field.area)) if field.area > 0 else 0.0
    c_obs = sum(o.area for o in obstacles) / field.area if field.area > 0 else 0.0
    return c_shape, c_obs


def bfs_growth(G: nx.Graph, max_size: int) -> List[List[int]]:
    """Simple BFS growth clustering that guarantees connectivity."""
    unassigned = set(G.nodes())
    clusters: List[List[int]] = []
    while unassigned:
        seed = unassigned.pop()
        cluster = {seed}
        q = [seed]
        while q and len(cluster) < max_size:
            u = q.pop(0)
            for v in G[u]:
                if v in unassigned:
                    unassigned.remove(v)
                    cluster.add(v)
                    q.append(v)
                    if len(cluster) >= max_size:
                        break
        clusters.append(list(cluster))
    return clusters


def kmeans_clusters(G: nx.Graph, k: int) -> List[List[int]]:
    """KMeans on node positions, then split each label bucket into connected components."""
    if KMeans is None:
        raise RuntimeError("sklearn not installed")
    ids = list(G.nodes())
    if not ids:
        return []
    k = max(1, min(k, len(ids)))
    pts = np.array([G.nodes[n]["pos"] for n in ids])
    labels = KMeans(k, random_state=0).fit_predict(pts)

    buckets: Dict[int, List[int]] = {i: [] for i in range(k)}
    for idx, lbl in enumerate(labels):
        buckets[int(lbl)].append(ids[idx])

    clusters: List[List[int]] = []
    for nodes in buckets.values():
        if not nodes:
            continue
        for comp in nx.connected_components(G.subgraph(nodes)):
            clusters.append(list(comp))
    return clusters


def metis_clusters(G: nx.Graph, k: int) -> List[List[int]]:
    """Metis partitioning on graph structure, then split into connected components."""
    if pymetis is None:
        raise RuntimeError("pymetis not installed")
    node_map = {n: i for i, n in enumerate(G.nodes())}
    inv = {i: n for n, i in node_map.items()}

    adj = [[] for _ in range(len(node_map))]
    for u, v in G.edges():
        iu, iv = node_map[u], node_map[v]
        adj[iu].append(iv)
        adj[iv].append(iu)

    k = max(1, min(k, len(node_map)))
    _, parts = pymetis.part_graph(k, adjacency=adj)

    buckets: Dict[int, List[int]] = {i: [] for i in range(k)}
    for i_part, idx in enumerate(parts):
        buckets[int(idx)].append(inv[i_part])

    clusters: List[List[int]] = []
    for nodes in buckets.values():
        if not nodes:
            continue
        for comp in nx.connected_components(G.subgraph(nodes)):
            clusters.append(list(comp))
    return clusters


def merge_small_clusters(G: nx.Graph, clusters: List[List[int]], min_size: int, max_size: int) -> List[List[int]]:
    """Merge clusters smaller than min_size into best neighboring clusters if possible."""
    cid_of: Dict[int, int] = {}
    for idx, cl in enumerate(clusters):
        for v in cl:
            cid_of[v] = idx

    sizes = [len(c) for c in clusters]
    small = [i for i, s in enumerate(sizes) if s < min_size]

    for s_idx in sorted(small, key=lambda i: sizes[i]):
        if sizes[s_idx] >= min_size:
            continue

        targets: Dict[int, int] = {}
        for v in clusters[s_idx]:
            for nbr in G[v]:
                t_cid = cid_of[nbr]
                if t_cid != s_idx:
                    targets[t_cid] = targets.get(t_cid, 0) + 1

        targets_sorted = sorted(targets.items(), key=lambda kv: (-kv[1], sizes[kv[0]]))
        for tgt, _ in targets_sorted:
            if sizes[tgt] + sizes[s_idx] <= max_size:
                clusters[tgt].extend(clusters[s_idx])
                sizes[tgt] += sizes[s_idx]
                for v in clusters[s_idx]:
                    cid_of[v] = tgt
                sizes[s_idx] = 0
                clusters[s_idx] = []
                break

    clusters = [cl for cl in clusters if cl]
    final: List[List[int]] = []
    for cl in clusters:
        for comp in nx.connected_components(G.subgraph(cl)):
            final.append(list(comp))
    return final


def cluster_grid(
    grid,
    field: Polygon,
    obstacles: List[Polygon],
    *,
    max_size: int = 100,
    target_k: int = 100,
    min_size: int = 5,
    algo: Optional[str] = None,
) -> List[List[int]]:
    """
    Cluster walkable cells into connected clusters.

    algo:
      - None: auto select based on metrics and availability
      - "bfs" | "kmeans" | "metis"
    """
    G = build_cell_graph(grid)
    for cell in grid.cells:
        if cell.walkable and cell.cell_number in G:
            G.nodes[cell.cell_number]["pos"] = cell.center

    c_shape, c_obs = compute_metrics(field, obstacles)

    # Auto choice (rough heuristic)
    if algo is None:
        algo = (
            "bfs" if c_obs >= 0.4 else
            "kmeans" if c_obs < 0.4 and c_shape < 1.5 else
            "metis"
        )

    # Availability fallbacks
    if algo == "kmeans" and KMeans is None:
        print("[cluster] sklearn missing → fallback BFS")
        algo = "bfs"
    if algo == "metis" and pymetis is None:
        print("[cluster] pymetis missing → fallback BFS")
        algo = "bfs"

    print(f"[cluster] C_shape={c_shape:.2f}, C_obs={c_obs:.2f} → {algo.upper()}")

    if algo == "bfs":
        clusters = bfs_growth(G, max_size=max_size)
    elif algo == "kmeans":
        clusters = kmeans_clusters(G, target_k)
    elif algo == "metis":
        clusters = metis_clusters(G, target_k)
    else:
        raise ValueError(f"Unknown algo: {algo}")

    if min_size > 1:
        clusters = merge_small_clusters(G, clusters, min_size, max_size)

    return clusters
