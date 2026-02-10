from __future__ import annotations
from typing import List, Tuple
import math

def downsample(points: List[Tuple[float, float]], ds: float = 1.0, keep_ends: bool = True) -> List[Tuple[float, float]]:
    """Keep a point if it's at least ds away from last kept point."""
    if len(points) <= 2 or ds <= 0:
        return points
    out = [points[0]] if keep_ends else []
    last = points[0]
    for p in points[1:-1]:
        if math.hypot(p[0]-last[0], p[1]-last[1]) >= ds:
            out.append(p)
            last = p
    if keep_ends:
        out.append(points[-1])
    return out


def downsample_keep_corners(
    points: List[Tuple[float, float]],
    *,
    ds: float = 1.0,
    corner_deg: float = 25.0,
) -> List[Tuple[float, float]]:
    """
    Downsample by distance while preserving corners.

    - Always keep first/last point
    - Always keep points where turning angle >= corner_deg
    - Otherwise keep points at least ds apart
    """
    if len(points) <= 2 or ds <= 0:
        return points

    def angle(a, b, c) -> float:
        # angle at b between ba and bc
        bax, bay = a[0]-b[0], a[1]-b[1]
        bcx, bcy = c[0]-b[0], c[1]-b[1]
        na = math.hypot(bax, bay)
        nc = math.hypot(bcx, bcy)
        if na < 1e-9 or nc < 1e-9:
            return 0.0
        dot = (bax*bcx + bay*bcy) / (na*nc)
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot))

    keep = [False] * len(points)
    keep[0] = True
    keep[-1] = True

    # mark corners
    for i in range(1, len(points)-1):
        ang = angle(points[i-1], points[i], points[i+1])
        if ang >= corner_deg:
            keep[i] = True

    out = [points[0]]
    last = points[0]
    for i in range(1, len(points)-1):
        p = points[i]
        if keep[i]:
            out.append(p)
            last = p
            continue
        if math.hypot(p[0]-last[0], p[1]-last[1]) >= ds:
            out.append(p)
            last = p

    out.append(points[-1])
    # remove accidental duplicates
    dedup = [out[0]]
    for p in out[1:]:
        if math.hypot(p[0]-dedup[-1][0], p[1]-dedup[-1][1]) > 1e-9:
            dedup.append(p)
    return dedup
