from __future__ import annotations
import json
from pathlib import Path
from typing import List, Tuple, Dict, Any, Optional

from shapely.geometry import shape, Polygon

def load_field_geojson(path: str | Path) -> Tuple[Polygon, List[Polygon], Dict[str, Any]]:
    """
    Load field + obstacles from a GeoJSON FeatureCollection.

    Accepts either:
      - properties.kind == "field"/"obstacle"
      - properties.type == "field"/"obstacle"

    Returns:
      field_polygon, obstacle_polygons, meta
    """
    p = Path(path)
    gj = json.loads(p.read_text(encoding="utf-8"))
    feats = gj.get("features", [])
    field = None
    obstacles: List[Polygon] = []
    for f in feats:
        props = f.get("properties", {}) or {}
        k = props.get("kind", props.get("type", "")).lower()
        geom = shape(f.get("geometry"))
        if not isinstance(geom, Polygon):
            # ignore non-polygons for now
            continue
        if k == "field":
            field = geom
        elif k == "obstacle":
            obstacles.append(geom)
    if field is None:
        raise ValueError("No field polygon found in geojson (properties.kind/type=='field').")
    meta = {
        "source": str(p),
        "feature_count": len(feats),
        "obstacle_count": len(obstacles),
    }
    return field, obstacles, meta
