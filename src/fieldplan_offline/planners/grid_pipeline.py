from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List

import math
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
    # grid build
    cell_size: float = 3.0
    keep_blocked: bool = True
    min_area_ratio: float = 0.2

    # clustering
    cluster_max: int = 120
    cluster_target: int = 80
    cluster_min: int = 8
    cluster_algo: str | None = None   # None|"bfs"|"kmeans"|"metis"

    # cluster-level tsp gates
    gate_samples: int = 8

    # intra-cluster
    intra_solver: str = "christofides"   # "ga"|"christofides"

    # stitching
    stitch_gamma: int = 1000
    stitch_patch: bool = True


def _path_length_xy(xy: List[tuple[float, float]]) -> float:
    if len(xy) < 2:
        return 0.0
    return float(sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(xy, xy[1:])))


def build_plan_from_grid(field: Polygon, obstacles: List[Polygon], *, cfg: GridPlanConfig) -> Plan:
    """
    Square-grid pipeline:
      field/obstacles -> grid -> clustering -> cluster-TSP -> intra-cluster order -> stitch -> segments
    Output is a Plan with segment list suitable for ROS2/Nav2 FollowPath execution.
    """
    grid = build_square_grid(
        field,
        obstacles,
        cell_size=cfg.cell_size,
        min_area_ratio=cfg.min_area_ratio,
        keep_blocked=cfg.keep_blocked,
    )

    clusters = cluster_grid(
        grid,
        field,
        obstacles,
        max_size=cfg.cluster_max,
        target_k=cfg.cluster_target,
        min_size=cfg.cluster_min,
        algo=cfg.cluster_algo,
    )
    order, gate_pairs = solve_cluster_tsp(grid, clusters, min_gate_samples=cfg.gate_samples)

    G_full = build_cell_graph(grid)
    neighbor_map = {n: list(G_full[n]) for n in G_full}

    tsp = TSPPlanner(population=160, generations=400, mutation=0.06, apply_2opt=True, seed=0)

    # Fast lookup
    cell_by_id = {c.cell_number: c for c in grid.cells if c.walkable}
    coord = {cid: cell.center for cid, cell in cell_by_id.items()}

    local_path: Dict[int, List[int]] = {}

    # intra-cluster routes
    for idx, cid in enumerate(order):
        prev_c = order[idx - 1]
        next_c = order[(idx + 1) % len(order)]
        u = gate_pairs[(prev_c, cid)][1]
        v = gate_pairs[(cid, next_c)][0]

        cell_set = set(clusters[cid])
        cell_set.update([u, v])
        objs = [cell_by_id[n] for n in cell_set if n in cell_by_id]

        route = tsp.plan(
            objs,
            dist_mode="graph",
            neighbor_map=neighbor_map,
            start_id=u,
            goal_id=v if u != v else None,
            solver=cfg.intra_solver,
            return_objects=False,
        )

        route = stitch(route, G_full, gamma=cfg.stitch_gamma, patch=cfg.stitch_patch)
        local_path[cid] = route

    # build segments: cluster segments + bridge segments
    segments: List[Segment] = []
    seg_id = 0

    for idx, cid in enumerate(order):
        # cluster segment
        cell_path = local_path[cid]
        xy = [coord[n] for n in cell_path if n in coord]
        poses = xy_to_poses(xy, frame_id="map")
        segments.append(Segment(
            id=f"seg_{seg_id:04d}",
            kind="intra_cluster",
            cluster_id=cid,
            frame_id="map",
            poses=poses,
            meta={
                "cell_count": len(cell_path),
                "length_m": _path_length_xy(xy),
                "start": xy[0] if xy else None,
                "end": xy[-1] if xy else None,
            },
        ))
        seg_id += 1

        # bridge to next cluster
        nxt = order[(idx + 1) % len(order)]
        ex, en = local_path[cid][-1], local_path[nxt][0]
        if ex == en:
            continue

        br_nodes = nx.shortest_path(G_full, ex, en)  # include endpoints
        br_xy = [coord[n] for n in br_nodes if n in coord]

        # Ensure at least 2 points for FollowPath
        if len(br_xy) < 2:
            continue

        poses2 = xy_to_poses(br_xy, frame_id="map")
        segments.append(Segment(
            id=f"seg_{seg_id:04d}",
            kind="bridge",
            cluster_id=-1,
            frame_id="map",
            poses=poses2,
            meta={
                "cell_count": len(br_nodes),
                "length_m": _path_length_xy(br_xy),
                "from_cluster": cid,
                "to_cluster": nxt,
                "start": br_xy[0],
                "end": br_xy[-1],
            },
        ))
        seg_id += 1

    plan = Plan(
        frame_id="map",
        field_meta={"cell_size": cfg.cell_size, "grid": "square"},
        clusters=clusters,
        cluster_order=order,
        segments=segments,
        transitions=[],
    )
    return plan
