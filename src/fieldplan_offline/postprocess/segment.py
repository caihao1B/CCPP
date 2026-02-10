from __future__ import annotations
from typing import List, Tuple
import math

def split_by_turn(points: List[Tuple[float,float]], turn_thresh_deg: float = 35.0, min_points: int = 3) -> List[List[Tuple[float,float]]]:
    """
    Split polyline into segments when heading change exceeds threshold.
    Points are (x,y). Very lightweight; good enough for producing row/turn chunks.
    """
    if len(points) < 3:
        return [points]
    segs: List[List[Tuple[float,float]]] = []
    cur = [points[0], points[1]]
    def heading(a,b):
        return math.atan2(b[1]-a[1], b[0]-a[0])
    last_h = heading(points[0], points[1])
    for i in range(2, len(points)):
        h = heading(points[i-1], points[i])
        ang = abs((h - last_h + math.pi) % (2*math.pi) - math.pi)
        if math.degrees(ang) >= turn_thresh_deg and len(cur) >= min_points:
            cur.append(points[i-1])
            segs.append(cur)
            cur = [points[i-1], points[i]]
        else:
            cur.append(points[i])
        last_h = h
    if cur:
        segs.append(cur)
    return segs

def split_by_length(points: List[Tuple[float,float]], max_len: float = 30.0) -> List[List[Tuple[float,float]]]:
    if len(points) < 2:
        return [points]
    out: List[List[Tuple[float,float]]] = []
    cur = [points[0]]
    acc = 0.0
    for i in range(1, len(points)):
        a,b = points[i-1], points[i]
        d = math.hypot(b[0]-a[0], b[1]-a[1])
        if acc + d > max_len and len(cur) >= 2:
            out.append(cur)
            cur = [a, b]
            acc = d
        else:
            cur.append(b)
            acc += d
    if cur:
        out.append(cur)
    return out
