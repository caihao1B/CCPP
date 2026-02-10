from __future__ import annotations
from typing import Iterable, List
from shapely.geometry import Polygon

def make_valid_polygons(polys: Iterable[Polygon]) -> List[Polygon]:
    """
    Attempt to fix invalid polygons (self-intersections, etc.).
    Requires shapely>=2.0 for make_valid; falls back to buffer(0) otherwise.
    """
    out: List[Polygon] = []
    for p in polys:
        if p.is_valid:
            out.append(p)
            continue
        try:
            from shapely import make_valid
            pv = make_valid(p)
            if isinstance(pv, Polygon):
                out.append(pv)
            else:
                # MultiPolygon -> keep the largest piece
                parts = list(getattr(pv, "geoms", []))
                if parts:
                    out.append(max(parts, key=lambda g: g.area))
        except Exception:
            out.append(p.buffer(0))
    return out
