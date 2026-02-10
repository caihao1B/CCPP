from __future__ import annotations
from typing import List, Tuple
import math

def downsample(points: List[Tuple[float,float]], ds: float = 1.0, keep_ends: bool = True) -> List[Tuple[float,float]]:
    """
    Keep a point if it's at least ds away from last kept point.
    """
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
