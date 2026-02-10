from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any

import networkx as nx

from shapely.geometry import Polygon

from fieldplan_offline.models.grid_square import build_square_grid
from fieldplan_offline.models.graph_square import build_cell_graph
from fieldplan_offline.planners.clustering import cluster_grid
from fieldplan_offline.planners.clu_tsp import solve_cluster_tsp
from fieldplan_offline.planners.tsp_ga import TSPPlanner
from fieldplan_offline.planners.path_stitch import stitch

from fieldplan_offline.export.plan_schema import Plan, Segment
from fieldplan_offline.postprocess.headings import xy_to_poses

@dataclass
class GridPlanConfig:
    cell_size: float = 3.0
    keep_blocked: bool = True
    min_area_ratio: float = 0.2

    cluster_max: int = 120
    cluster_target: int = 80
    cluster_min: int = 8

    gate_samples: int = 8
    intra_solver: str = "christofides"   # "ga"|"christofides"

    stitch_gamma: int = 1000
    stitch_patch: bool = True

def build_plan_from_grid(field: Polygon, obstacles: List[Polygon], *, cfg: GridPlanConfig) -> Plan:
    grid = build_square_grid(field, obstacles, cell_size=cfg.cell_size, min_area_ratio=cfg.min_area_ratio, keep_blocked=cfg.keep_blocked)

    clusters = cluster_grid(grid, field, obstacles, max_size=cfg.cluster_max, target_k=cfg.cluster_target, min_size=cfg.cluster_min)
    order, gate_pairs = solve_cluster_tsp(grid, clusters, min_gate_samples=cfg.gate_samples)

    G_full = build_cell_graph(grid)
    neighbor_map = {n: list(G_full[n]) for n in G_full}

    tsp = TSPPlanner(population=160, generations=400, mutation=0.06, apply_2opt=True, seed=0)

    # map cell_id -> (x,y)
    coord = {c.cell_number: c.center for c in grid.cells if c.walkable}

    local_path: Dict[int, List[int]] = {}
    bridges: List[List[int]] = []

    # intra-cluster routes
    for idx, cid in enumerate(order):
        prev_c = order[idx - 1]
        next_c = order[(idx + 1) % len(order)]
        u = gate_pairs[(prev_c, cid)][1]
        v = gate_pairs[(cid, next_c)][0]

        cell_set = set(clusters[cid]); cell_set.update([u, v])
        objs = [c for c in grid.cells if c.cell_number in cell_set]

        route = tsp.plan(
            objs,
            dist_mode="graph",
            neighbor_map=neighbor_map,
            start_id=u,
            goal_id=v if u != v else None,
            solver=cfg.intra_solver,
            return_objects=False
        )
        route = stitch(route, G_full, gamma=cfg.stitch_gamma, patch=cfg.stitch_patch)
        local_path[cid] = route

    # build segments: cluster segments + bridge segments
    segments: List[Segment] = []
    seg_id = 0
    for idx, cid in enumerate(order):
        cell_path = local_path[cid]
        xy = [coord[n] for n in cell_path if n in coord]
        poses = xy_to_poses(xy, frame_id="map")
        segments.append(Segment(
            id=f"seg_{seg_id:04d}",
            kind="intra_cluster",
            cluster_id=cid,
            frame_id="map",
            poses=poses,
            meta={"cell_count": len(cell_path)}
        ))
        seg_id += 1

        nxt = order[(idx + 1) % len(order)]
        ex, en = local_path[cid][-1], local_path[nxt][0]
        if ex != en:
            br = nx.shortest_path(G_full, ex, en)[1:-1]
            if br:
                bridges.append(br)
                xy2 = [coord[n] for n in br if n in coord]
                poses2 = xy_to_poses(xy2, frame_id="map")
                segments.append(Segment(
                    id=f"seg_{seg_id:04d}",
                    kind="bridge",
                    cluster_id=-1,
                    frame_id="map",
                    poses=poses2,
                    meta={"cell_count": len(br), "from_cluster": cid, "to_cluster": nxt}
                ))
                seg_id += 1

    plan = Plan(
        frame_id="map",
        field_meta={"cell_size": cfg.cell_size, "grid": "square"},
        clusters=clusters,
        cluster_order=order,
        segments=segments,
        transitions=[]
    )
    return plan
