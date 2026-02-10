from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional

import math
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.ops import unary_union
from shapely import affinity

@dataclass
class Swath:
    line: LineString
    index: int

def _long_axis_angle(poly: Polygon) -> float:
    # angle of the longest edge of minimum rotated rectangle
    mrr = poly.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)[:-1]
    best = None
    best_len = -1.0
    for a, b in zip(coords, coords[1:]+coords[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        L = dx*dx + dy*dy
        if L > best_len:
            best_len = L
            best = (dx, dy)
    if best is None:
        return 0.0
    return math.degrees(math.atan2(best[1], best[0]))

def compute_boustrophedon_segments(
    field: Polygon,
    obstacles: List[Polygon],
    *,
    swath_spacing: float,
    margin: float = 0.0,
    sweep_angle_deg: Optional[float] = None,
    sample_step: float = 1.0,
) -> List[List[Tuple[float, float]]]:
    """
    Generate continuous serpentine (swath) segments:
      - shrink field by margin, expand obstacles by margin
      - generate parallel lines, intersect with free space
      - output polyline segments (each a list of (x,y)) in boustrophedon order
    """
    if sweep_angle_deg is None:
        sweep_angle_deg = _long_axis_angle(field)

    free = field.buffer(-margin) if margin != 0 else field
    if obstacles:
        obs = unary_union([o.buffer(margin) if margin != 0 else o for o in obstacles])
        free = free.difference(obs)
    if free.is_empty:
        return []

    # rotate to make sweep lines horizontal
    origin = free.centroid.coords[0]
    free_r = affinity.rotate(free, -sweep_angle_deg, origin=origin)

    minx, miny, maxx, maxy = free_r.bounds
    ys = []
    y = miny - swath_spacing
    while y <= maxy + swath_spacing:
        ys.append(y)
        y += swath_spacing

    segs: List[List[Tuple[float, float]]] = []
    direction = 1
    for i, y in enumerate(ys):
        line = LineString([(minx-10*swath_spacing, y), (maxx+10*swath_spacing, y)])
        inter = free_r.intersection(line)
        parts: List[LineString] = []
        if isinstance(inter, LineString):
            if inter.length > 1e-6:
                parts = [inter]
        elif isinstance(inter, MultiLineString):
            parts = [g for g in inter.geoms if g.length > 1e-6]
        else:
            parts = []
        # order parts left->right by x
        parts.sort(key=lambda ls: ls.centroid.x)
        if direction < 0:
            parts = list(reversed(parts))

        for p in parts:
            coords = list(p.coords)
            # sample step along the line if needed
            if sample_step > 0 and p.length > sample_step*1.5:
                n = max(2, int(math.ceil(p.length / sample_step)))
                coords = [p.interpolate(t/(n-1), normalized=True).coords[0] for t in range(n)]
            # rotate back
            p_back = affinity.rotate(LineString(coords), sweep_angle_deg, origin=origin)
            segs.append(list(p_back.coords))
        direction *= -1
    return segs
