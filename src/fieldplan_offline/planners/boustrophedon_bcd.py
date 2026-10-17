from __future__ import annotations

from dataclasses import dataclass, field as dataclass_field
from typing import Any, Dict, List, Optional, Tuple

import math

import networkx as nx
import numpy as np
from shapely import affinity
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union
from shapely.prepared import prep

from fieldplan_offline.io.path_safety import (
    GEOMETRY_TOL, NoSafePathError, covers_segment as _covers_segment, validate_path,
)

from fieldplan_offline.models.graph_square import build_cell_graph
from fieldplan_offline.models.grid_square import build_square_grid


# =============================================================================
# Data structures
# =============================================================================

@dataclass(frozen=True)
class BcdCell:
    """A single boustrophedon cell (in rotated frame)."""
    id: int
    slab_idx: int
    x_range: Tuple[float, float]
    poly_r: Polygon
    centroid_r: Tuple[float, float]


@dataclass
class GridConnector:
    """
    Grid-based connector used for obstacle-avoiding transitions (A* on square grid).
    The graph is built on a rotated frame so everything is consistent with BCD slicing.
    """
    graph: nx.Graph
    node_center: Dict[int, Tuple[float, float]]
    ids: np.ndarray
    centers: np.ndarray
    free_r: Polygon | MultiPolygon

    _attachments: dict = dataclass_field(default_factory=dict, init=False, repr=False)

    def __post_init__(self):
        self._allowed = prep(self.free_r.buffer(GEOMETRY_TOL))

    def nearest_node(self, pt: Tuple[float, float]) -> int:
        """Nearest *visible* node; never silently discard an exact endpoint."""
        pt = tuple(pt)
        if pt in self._attachments:
            return self._attachments[pt]
        if not self._allowed.covers(Point(pt)):
            raise NoSafePathError(f"Connector endpoint outside free space: {pt}")
        p = np.asarray(pt, dtype=float)
        d2 = np.sum((self.centers - p[None, :]) ** 2, axis=1)
        for idx in np.argsort(d2):
            node = int(self.ids[idx])
            center = self.node_center[node]
            edge = Point(pt) if pt == center else LineString([pt, center])
            if self._allowed.covers(edge):
                self._attachments[pt] = node
                return node
        raise NoSafePathError(f"No visible connector grid node from {pt}; refine cell_size.")

    def shortest_path_nodes(self, a: Tuple[float, float], b: Tuple[float, float]) -> List[int]:
        s = self.nearest_node(a)
        g = self.nearest_node(b)

        def h(u: int, v: int) -> float:
            return math.dist(self.node_center[u], self.node_center[v])

        try:
            return nx.astar_path(self.graph, s, g, heuristic=h, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound) as exc:
            raise NoSafePathError(
                f"No safe grid connection: {a} -> {b}; check connectivity or refine cell_size."
            ) from exc

    def shortest_path_length(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        pts = self.shortest_path_xy(a, b)
        return sum(math.dist(u, v) for u, v in zip(pts, pts[1:]))

    def shortest_path_xy(
        self,
        a: Tuple[float, float],
        b: Tuple[float, float],
        *,
        include_endpoints: bool = True,
        tol: float = GEOMETRY_TOL,
    ) -> List[Tuple[float, float]]:
        # Preserve the old grid-only option, but full connections prefer safe straight lines.
        if include_endpoints and _covers_segment(self.free_r, a, b, tol=tol):
            return [a] if a == b else [a, b]
        nodes = self.shortest_path_nodes(a, b)
        pts = [self.node_center[n] for n in nodes]
        if include_endpoints:
            pts = [a] + pts + [b]
        # Preserve exact endpoints, including very short connections.
        pts = _dedup_consecutive(pts, eps=0.0)
        validate_path(self.free_r, pts, context=f"connector {a} -> {b}")
        return pts


# =============================================================================
# Geometry helpers
# =============================================================================

def _long_axis_angle(poly: Polygon) -> float:
    """Angle (deg) of the longest edge of the minimum rotated rectangle."""
    mrr = poly.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)[:-1]
    best = None
    best_len = -1.0
    for a, b in zip(coords, coords[1:] + coords[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = dx * dx + dy * dy
        if L > best_len:
            best_len = L
            best = (dx, dy)
    if best is None:
        return 0.0
    return math.degrees(math.atan2(best[1], best[0]))


def _extract_polygons(geom: Any) -> List[Polygon]:
    if geom is None or getattr(geom, "is_empty", False):
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return [p for p in geom.geoms if isinstance(p, Polygon)]
    if isinstance(geom, GeometryCollection):
        out: List[Polygon] = []
        for g in geom.geoms:
            out.extend(_extract_polygons(g))
        return out
    geoms = getattr(geom, "geoms", None)
    if geoms is not None:
        out: List[Polygon] = []
        for g in geoms:
            out.extend(_extract_polygons(g))
        return out
    return []


def _quantize(x: float, tol: float) -> float:
    if tol <= 0:
        return x
    return round(x / tol) * tol


def _collect_critical_x(free_r: Polygon | MultiPolygon, *, tol: float) -> List[float]:
    """
    Collect critical x-coordinates from the boundary of free-space geometry.
    These serve as the sweep 'events' for BCD-like slicing.
    """
    xs: List[float] = []
    for p in _extract_polygons(free_r):
        xs.extend([c[0] for c in p.exterior.coords])
        for ring in p.interiors:
            xs.extend([c[0] for c in ring.coords])
    if not xs:
        return []
    return sorted({_quantize(x, tol) for x in xs})


def _dedup_consecutive(pts: List[Tuple[float, float]], *, eps: float = 1e-9) -> List[Tuple[float, float]]:
    if not pts:
        return []
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    return out


def _sample_linestring(ls: LineString, step: float) -> List[Tuple[float, float]]:
    """Sample a LineString into points with approximately 'step' spacing."""
    if step <= 0 or ls.length < step * 1.5:
        return list(ls.coords)
    n = max(2, int(math.ceil(ls.length / step)) + 1)
    return [ls.interpolate(i / (n - 1), normalized=True).coords[0] for i in range(n)]


def _straight_connector(a: Tuple[float, float], b: Tuple[float, float], step: float) -> List[Tuple[float, float]]:
    ax, ay = a
    bx, by = b
    d = math.hypot(bx - ax, by - ay)
    if d < 1e-9:
        return [a]
    if step <= 0:
        return [a, b]
    n = max(2, int(math.ceil(d / step)) + 1)
    return [(ax + (bx - ax) * i / (n - 1), ay + (by - ay) * i / (n - 1)) for i in range(n)]


def _simple_turn_connector(
    cell_poly: Polygon,
    end_pt: Tuple[float, float],
    start_pt: Tuple[float, float],
    *,
    end_is_top: bool,
    start_is_top: bool,
    tol: float,
) -> Optional[List[Tuple[float, float]]]:
    """
    Try a cheap in-cell connector that stays inside the cell:
      end -> (end_x, y_conn) -> (start_x, y_conn) -> start
    If top-to-top, pick y_conn = min(end_y, start_y).
    If bottom-to-bottom, pick y_conn = max(end_y, start_y).
    If end/top mismatch, try direct segment.

    Return None if not covered by cell polygon.
    """
    ax, ay = end_pt
    bx, by = start_pt

    if end_is_top and start_is_top:
        y_conn = min(ay, by)
    elif (not end_is_top) and (not start_is_top):
        y_conn = max(ay, by)
    else:
        if _covers_segment(cell_poly, end_pt, start_pt, tol=tol):
            return [end_pt, start_pt]
        return None

    p1 = (ax, y_conn)
    p2 = (bx, y_conn)
    if not _covers_segment(cell_poly, end_pt, p1, tol=tol):
        return None
    if not _covers_segment(cell_poly, p1, p2, tol=tol):
        return None
    if not _covers_segment(cell_poly, p2, start_pt, tol=tol):
        return None
    return [end_pt, p1, p2, start_pt]


# =============================================================================
# BCD decomposition
# =============================================================================

def decompose_bcd(
    free_r: Polygon | MultiPolygon,
    *,
    crit_tol: float = 1e-3,
    min_slab_width: float = 1e-3,
    min_cell_area: float = 1e-4,
    pad: float = 1.0,
) -> Tuple[List[BcdCell], List[float]]:
    """
    BCD-like decomposition in rotated frame:

    1) Collect critical x from free-space boundary.
    2) Build vertical slabs between consecutive x cuts.
    3) Intersect free-space with each slab.
    4) Each connected polygon component inside a slab becomes one cell.

    Returns:
      cells: list of BcdCell
      x_cuts: x cut positions used
    """
    if getattr(free_r, "is_empty", False):
        return [], []

    minx, miny, maxx, maxy = free_r.bounds
    xs = _collect_critical_x(free_r, tol=crit_tol)
    xs = [minx] + [x for x in xs if (x > minx + 1e-9 and x < maxx - 1e-9)] + [maxx]
    xs = sorted(xs)

    x_cuts: List[float] = []
    for x in xs:
        if not x_cuts or abs(x - x_cuts[-1]) > max(crit_tol * 0.5, 1e-12):
            x_cuts.append(x)

    cells: List[BcdCell] = []
    for k in range(len(x_cuts) - 1):
        xa, xb = x_cuts[k], x_cuts[k + 1]
        if (xb - xa) < min_slab_width:
            continue

        strip = box(xa, miny - pad, xb, maxy + pad)
        inter = free_r.intersection(strip)

        for poly in _extract_polygons(inter):
            if poly.area < min_cell_area:
                continue
            cxy = poly.centroid.coords[0]
            cells.append(
                BcdCell(
                    id=len(cells),
                    slab_idx=k,
                    x_range=(xa, xb),
                    poly_r=poly,
                    centroid_r=(float(cxy[0]), float(cxy[1])),
                )
            )

    return cells, x_cuts


def build_cell_adjacency(cells: List[BcdCell], *, tol: float = 1e-6) -> nx.Graph:
    """Adjacency graph based on geometric contact (distance <= tol)."""
    G = nx.Graph()
    for c in cells:
        G.add_node(c.id)

    for i in range(len(cells)):
        pi = cells[i].poly_r
        for j in range(i + 1, len(cells)):
            pj = cells[j].poly_r
            if pi.distance(pj) <= tol:
                G.add_edge(cells[i].id, cells[j].id, weight=1.0)
    return G


def order_cells(
    cells: List[BcdCell],
    G: nx.Graph,
    *,
    mode: str = "dfs",
    connector: Optional[GridConnector] = None,
) -> List[int]:
    """
    Order cells:
      - dfs: depth-first traversal on adjacency graph (classic BCD-style execution).
      - tsp: approximate TSP on cell centroids (weights from connector if provided).
    """
    if not cells:
        return []

    centroid = {c.id: c.centroid_r for c in cells}

    comps = list(nx.connected_components(G)) if G.number_of_nodes() else []
    comps = sorted(comps, key=len, reverse=True)
    ordered: List[int] = []

    def dfs_component(nodes: set[int]) -> List[int]:
        start = min(nodes, key=lambda u: (centroid[u][0], centroid[u][1]))
        out: List[int] = []
        visited: set[int] = set()

        def dfs(u: int) -> None:
            visited.add(u)
            out.append(u)
            nbrs = [v for v in G.neighbors(u) if v in nodes]
            nbrs = sorted(nbrs, key=lambda v: (centroid[v][0], centroid[v][1]))
            for v in nbrs:
                if v not in visited:
                    dfs(v)

        dfs(start)
        for u in sorted(nodes):
            if u not in visited:
                dfs(u)
        return out

    if mode == "dfs" or connector is None:
        for comp in comps:
            ordered.extend(dfs_component(set(comp)))
        return ordered

    # TSP mode
    ids = [c.id for c in cells]
    Gc = nx.Graph()
    Gc.add_nodes_from(ids)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a = centroid[ids[i]]
            b = centroid[ids[j]]
            try:
                d = connector.shortest_path_length(a, b)
            except Exception:
                d = math.hypot(a[0] - b[0], a[1] - b[1])
            Gc.add_edge(ids[i], ids[j], weight=float(d))

    try:
        from networkx.algorithms.approximation import christofides
        tour = christofides(Gc, weight="weight")
    except Exception:
        from networkx.algorithms.approximation import greedy_tsp
        tour = greedy_tsp(Gc, weight="weight", source=ids[0])

    if tour and tour[0] == tour[-1]:
        tour = tour[:-1]
    return list(tour)


# =============================================================================
# Coverage inside one cell (boustrophedon motion)
# =============================================================================

def cover_cell_boustrophedon(
    cell_poly_r: Polygon,
    *,
    swath_spacing: float,
    sample_step: float,
    connector: Optional[GridConnector] = None,
    connector_step: Optional[float] = None,
    tol: float = 1e-6,
) -> List[Tuple[float, float]]:
    """
    Generate a serpentine coverage path inside one BCD cell.

    In rotated frame, cells are approximately x-monotone => we generate vertical swaths (x=const).
    Between consecutive swaths, we try:
      1) simple in-cell connector (guaranteed inside if it passes coverage test),
      2) grid-based connector (A* on free-space grid),
      3) explicit failure if neither connection is safe.

    Returns a single polyline (list of xy).
    """
    if swath_spacing <= 0:
        raise ValueError("swath_spacing must be positive")

    minx, miny, maxx, maxy = cell_poly_r.bounds
    eps = max(1e-6, swath_spacing * 1e-4)

    xs: List[float] = []
    x = minx + eps
    while x <= maxx - eps + 1e-9:
        xs.append(x)
        x += swath_spacing
    if not xs:
        xs = [(minx + maxx) * 0.5]

    if connector_step is None:
        connector_step = sample_step

    path: List[Tuple[float, float]] = []
    direction_up = True
    prev_end_is_top: Optional[bool] = None

    for x in xs:
        line = LineString([(x, miny - 1.0), (x, maxy + 1.0)])
        inter = cell_poly_r.intersection(line)

        parts: List[LineString] = []
        if isinstance(inter, LineString):
            if inter.length > 1e-6:
                parts = [inter]
        else:
            for g in getattr(inter, "geoms", []):
                if isinstance(g, LineString) and g.length > 1e-6:
                    parts.append(g)
        if not parts:
            continue

        seg = max(parts, key=lambda g: g.length)
        sampled = _sample_linestring(seg, sample_step)

        sampled = sorted(sampled, key=lambda p: p[1])
        if not direction_up:
            sampled = list(reversed(sampled))

        start_pt = sampled[0]
        end_pt = sampled[-1]
        start_is_top = start_pt[1] > end_pt[1]
        end_is_top = end_pt[1] > start_pt[1]

        if not path:
            path.append(start_pt)
        else:
            simple = _simple_turn_connector(
                cell_poly_r,
                path[-1],
                start_pt,
                end_is_top=bool(prev_end_is_top),
                start_is_top=bool(start_is_top),
                tol=tol,
            )
            if simple is not None:
                conn_pts = simple
            elif connector is not None:
                conn_pts = connector.shortest_path_xy(path[-1], start_pt, include_endpoints=True, tol=tol)
            else:
                raise NoSafePathError(f"No safe in-cell connection: {path[-1]} -> {start_pt}")

            # densify connector
            if len(conn_pts) >= 2 and connector_step and connector_step > 0:
                densified: List[Tuple[float, float]] = []
                for a, b in zip(conn_pts, conn_pts[1:]):
                    densified.extend(_straight_connector(a, b, connector_step)[:-1])
                densified.append(conn_pts[-1])
                conn_pts = densified

            for p in conn_pts[1:]:
                if p != path[-1]:
                    path.append(p)

        for p in sampled[1:]:
            if p != path[-1]:
                path.append(p)

        prev_end_is_top = end_is_top
        direction_up = not direction_up

    path = _dedup_consecutive(path, eps=0.0)
    if path:
        validate_path(connector.free_r if connector else cell_poly_r, path, context="cell coverage")
    return path


# =============================================================================
# High-level planner: field -> segments
# =============================================================================

def build_grid_connector(
    field_r: Polygon,
    obstacles_r: List[Polygon],
    *,
    cell_size: float,
    min_area_ratio: float = 0.2,
) -> GridConnector:
    """
    Build a square-grid connector in rotated frame.

    Important detail:
    - we additionally require the cell center to be inside free-space, so path vertices
      (centers) won't drift outside field boundary in concave boundaries.
    """
    if not math.isfinite(cell_size) or cell_size <= 0:
        raise ValueError("connector cell_size must be positive and finite")
    grid = build_square_grid(
        field_r,
        obstacles_r,
        cell_size=cell_size,
        min_area_ratio=min_area_ratio,
        keep_blocked=True,
        prune_isolated=False,
    )

    free_r = field_r
    if obstacles_r:
        free_r = field_r.difference(unary_union(obstacles_r))

    # Check centers AND complete graph edges (concave boundaries can cut an edge
    # even when both centers lie inside). Keep all components for honest failures.
    allowed = prep(free_r.buffer(GEOMETRY_TOL))
    for c in grid.cells:
        if c.walkable and not allowed.covers(Point(c.center)):
            c.walkable = False
    G = build_cell_graph(grid)
    node_center = {c.cell_number: (float(c.center[0]), float(c.center[1]))
                   for c in grid.cells if c.walkable and c.cell_number in G}
    for u, v in list(G.edges):
        if not allowed.covers(LineString([node_center[u], node_center[v]])):
            G.remove_edge(u, v)
        else:
            G[u][v]["weight"] = math.dist(node_center[u], node_center[v])
    if not node_center:
        raise NoSafePathError("No walkable connector grid nodes; check margin/cell_size.")

    ids = np.array(list(node_center.keys()), dtype=int)
    centers = np.array(list(node_center.values()), dtype=float)
    return GridConnector(graph=G, node_center=node_center, ids=ids, centers=centers, free_r=free_r)


def plan_boustrophedon_bcd(
    field: Polygon,
    obstacles: List[Polygon],
    *,
    swath_spacing: float,
    margin: float = 0.0,
    sweep_angle_deg: Optional[float] = None,
    sample_step: float = 1.0,
    connector_cell_size: Optional[float] = None,
    connector_step: Optional[float] = None,
    cell_order_mode: str = "dfs",   # "dfs" | "tsp"
    crit_tol: Optional[float] = None,
    min_slab_width: Optional[float] = None,
    min_cell_area: float = 1e-3,
    adjacency_tol: Optional[float] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """
    Full BCD-style boustrophedon planner:

    1) margin: field shrink + obstacle inflation
    2) rotate to BCD frame
    3) decompose into cells (slabs between critical x)
    4) order cells (dfs or tsp)
    5) cover each cell with vertical swaths
    6) connect cells with obstacle-avoiding grid paths
    7) rotate back; return segments and meta

    Returns:
      segments: list of dict {kind: "row"/"turn", points: [(x,y)...], meta: {...}}
      meta: planner-level metadata
    """
    if not math.isfinite(swath_spacing) or swath_spacing <= 0:
        raise ValueError("swath_spacing must be positive and finite")
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("margin must be nonnegative and finite")
    if sweep_angle_deg is None:
        sweep_angle_deg = _long_axis_angle(field)

    # margin
    field_m = field.buffer(-margin) if margin != 0 else field
    if field_m.is_empty:
        raise ValueError("Field became empty after margin. Reduce margin.")

    obstacles_m = [o.buffer(margin) if margin != 0 else o for o in obstacles]
    obstacles_m = [o for o in obstacles_m if not o.is_empty]

    free = field_m
    if obstacles_m:
        free = free.difference(unary_union(obstacles_m))
    if free.is_empty:
        raise ValueError("Free space is empty (field minus obstacles).")

    origin = free.centroid.coords[0]
    field_r = affinity.rotate(field_m, -sweep_angle_deg, origin=origin)
    obstacles_r = [affinity.rotate(o, -sweep_angle_deg, origin=origin) for o in obstacles_m]
    free_r = affinity.rotate(free, -sweep_angle_deg, origin=origin)

    # heuristics for tolerances
    if crit_tol is None:
        crit_tol = max(1e-3, swath_spacing * 1e-3)
    if min_slab_width is None:
        min_slab_width = max(1e-3, swath_spacing * 1e-2)
    if adjacency_tol is None:
        adjacency_tol = max(1e-6, swath_spacing * 1e-4)

    # connector grid resolution
    if connector_cell_size is None:
        connector_cell_size = max(0.2, min(0.5, swath_spacing * 0.5))

    connector = build_grid_connector(field_r, obstacles_r, cell_size=connector_cell_size)

    cells, x_cuts = decompose_bcd(
        free_r,
        crit_tol=crit_tol,
        min_slab_width=min_slab_width,
        min_cell_area=min_cell_area,
        pad=1.0,
    )
    if not cells:
        raise RuntimeError("BCD produced no cells. Check min_cell_area / geometry / margin.")

    Gcells = build_cell_adjacency(cells, tol=adjacency_tol)
    cell_order = order_cells(
        cells,
        Gcells,
        mode=cell_order_mode,
        connector=connector if cell_order_mode == "tsp" else None,
    )

    # precompute per-cell paths
    cell_path_r: Dict[int, List[Tuple[float, float]]] = {}
    for cid in cell_order:
        poly = cells[cid].poly_r
        path = cover_cell_boustrophedon(
            poly,
            swath_spacing=swath_spacing,
            sample_step=sample_step,
            connector=connector,
            connector_step=connector_step,
            tol=adjacency_tol,
        )
        if len(path) >= 2:
            cell_path_r[cid] = path

    segments_r: List[Dict[str, Any]] = []
    prev_end: Optional[Tuple[float, float]] = None
    prev_cid: Optional[int] = None

    for cid in cell_order:
        path = cell_path_r.get(cid)
        if not path:
            continue

        # choose orientation to reduce connection cost
        if prev_end is not None:
            def connection_cost(endpoint):
                try:
                    return connector.shortest_path_length(prev_end, endpoint)
                except NoSafePathError:
                    return math.inf
            cost_fwd = connection_cost(path[0])
            cost_rev = connection_cost(path[-1])
            if math.isinf(cost_fwd) and math.isinf(cost_rev):
                raise NoSafePathError(f"No safe connection from cell {prev_cid} to cell {cid}")
            if cost_rev + 1e-9 < cost_fwd:
                path = list(reversed(path))

        if prev_end is not None:
            conn = connector.shortest_path_xy(prev_end, path[0], include_endpoints=True, tol=adjacency_tol)
            if len(conn) >= 2:
                segments_r.append(
                    {"kind": "turn", "points": conn, "meta": {"type": "inter_cell", "from_cell": prev_cid, "to_cell": cid}}
                )

        segments_r.append(
            {
                "kind": "row",
                "points": path,
                "meta": {
                    "type": "cell_coverage",
                    "cell_id": cid,
                    "slab_idx": cells[cid].slab_idx,
                    "x_range": cells[cid].x_range,
                    "cell_area": float(cells[cid].poly_r.area),
                },
            }
        )

        prev_end = path[-1]
        prev_cid = cid

    # rotate back
    def rot_back(pt: Tuple[float, float]) -> Tuple[float, float]:
        p = Point(pt)
        pr = affinity.rotate(p, sweep_angle_deg, origin=origin)
        return (float(pr.x), float(pr.y))

    segments_out: List[Dict[str, Any]] = []
    for s in segments_r:
        pts = [rot_back(p) for p in s["points"]]
        segments_out.append({"kind": s["kind"], "points": pts, "meta": s.get("meta", {})})

    # Flattening also checks every seam that a JSON/CSV consumer will traverse.
    if segments_out:
        validate_path(free, [p for s in segments_out for p in s["points"]], context="BCD plan")

    meta: Dict[str, Any] = {
        "planner": "boustrophedon_bcd",
        "sweep_angle_deg": float(sweep_angle_deg),
        "margin": float(margin),
        "swath_spacing": float(swath_spacing),
        "sample_step": float(sample_step),
        "connector_cell_size": float(connector_cell_size),
        "cell_order_mode": cell_order_mode,
        "cell_count": len(cells),
        "x_cut_count": len(x_cuts),
        "crit_tol": float(crit_tol),
        "min_slab_width": float(min_slab_width),
        "min_cell_area": float(min_cell_area),
        "adjacency_tol": float(adjacency_tol),
    }
    return segments_out, meta