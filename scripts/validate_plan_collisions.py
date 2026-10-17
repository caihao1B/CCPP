#!/usr/bin/env python3
"""Audit complete Plan JSON edges, including the gaps between segments.

Exit 0: covered by the configured free space; 1: geometric violations;
2: input error. Raw obstacle intersections include boundary contact and are
reported separately from violations of free-space coverage (tolerance 1e-8 m).
This is a point-path audit, not a footprint/curvature/dynamics certificate.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path

from shapely.geometry import LineString, Point, shape
from shapely.ops import unary_union

GEOMETRY_TOL = 1e-8


def audit_plan(plan: dict, geojson: dict, margin: float | None = None) -> dict:
    if not plan.get('segments'):
        raise ValueError('Plan contains no segments')
    field = None
    obstacles = []
    for feature in geojson['features']:
        props = feature.get('properties') or {}
        kind = props.get('kind', props.get('type', '')).lower()
        if kind == 'field':
            field = shape(feature['geometry'])
        elif kind == 'obstacle':
            obstacles.append((props.get('obstacle_id', len(obstacles)), shape(feature['geometry'])))
    if field is None or not field.is_valid or any(not o.is_valid for _, o in obstacles):
        raise ValueError('A valid field and valid obstacle polygons are required.')
    if margin is None:
        margin = float(plan.get('field_meta', {}).get('margin', 0.0))
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('margin must be finite and nonnegative')
    free = field.buffer(-margin) if margin else field
    if obstacles:
        free = free.difference(unary_union([o.buffer(margin) if margin else o for _, o in obstacles]))
    allowed = free.buffer(GEOMETRY_TOL)
    counts = Counter()
    hit_segments = set()
    hit_turns = set()
    details = []
    previous = None
    length = 0.0

    def check(a, b, *, segment, kind, edge, seam=False):
        nonlocal length
        line = Point(a) if a == b else LineString([a, b])
        hits = [oid for oid, obs in obstacles if line.intersects(obs)]
        interior = [oid for oid, obs in obstacles if line.intersects(obs) and not line.touches(obs)]
        outside = not allowed.covers(line)
        prefix = 'seam' if seam else 'path'
        counts[f'{prefix}_edges'] += 1
        length += line.length
        if hits:
            counts[f'{prefix}_obstacle_edges'] += 1
            if not seam:
                hit_segments.add(segment)
                if kind == 'turn':
                    hit_turns.add(segment)
                    counts['turn_obstacle_edges'] += 1
        if interior:
            counts[f'{prefix}_obstacle_interior_edges'] += 1
        if outside:
            counts[f'{prefix}_unsafe_edges'] += 1
        if hits or outside:
            details.append(dict(segment_id=segment, kind=kind, edge_index=edge, seam=seam,
                                a=a, b=b, obstacle_ids=hits, interior_obstacle_ids=interior,
                                outside_free_space=outside))

    for seg in plan['segments']:
        pts = [(float(p['x']), float(p['y'])) for p in seg['poses']]
        if not pts or any(not all(math.isfinite(v) for v in p) for p in pts):
            raise ValueError(f"Empty or non-finite segment: {seg['id']}")
        counts['segments'] += 1
        counts[seg['kind'] + '_segments'] += 1
        counts['unsafe_points'] += sum(not allowed.covers(Point(p)) for p in pts)
        if previous is not None:
            gap = math.dist(previous[1], pts[0])
            if gap > GEOMETRY_TOL:
                counts['discontinuous_seams'] += 1
            check(previous[1], pts[0], segment=f"{previous[0]} -> {seg['id']}",
                  kind='seam', edge=-1, seam=True)
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            check(a, b, segment=seg['id'], kind=seg['kind'], edge=i)
        previous = seg['id'], pts[-1]
    keys = ['segments', 'row_segments', 'turn_segments', 'path_edges', 'path_obstacle_edges',
            'turn_obstacle_edges', 'path_obstacle_interior_edges', 'path_unsafe_edges',
            'seam_edges', 'seam_obstacle_edges', 'seam_obstacle_interior_edges',
            'seam_unsafe_edges', 'discontinuous_seams', 'unsafe_points']
    return {
        'safe': counts['path_unsafe_edges'] + counts['seam_unsafe_edges'] + counts['unsafe_points'] == 0,
        'continuous': counts['discontinuous_seams'] == 0,
        'margin': margin, 'geometry_tolerance': GEOMETRY_TOL,
        'obstacles': len(obstacles), **{key: counts[key] for key in keys},
        'obstacle_segments': len(hit_segments), 'obstacle_turn_segments': len(hit_turns),
        'obstacle_turn_ids': sorted(hit_turns), 'length_including_seams': length,
        'violations': details,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--geojson', type=Path, required=True)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--margin', type=float, default=None, help='Default: plan field_meta.margin')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()
    try:
        result = audit_plan(json.loads(args.plan.read_text(encoding='utf-8')),
                            json.loads(args.geojson.read_text(encoding='utf-8')), args.margin)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        ap.error(str(exc))
    result['plan'] = str(args.plan)
    if args.out:
        args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'violations'}, indent=2))
    raise SystemExit(0 if result['safe'] else 1)


if __name__ == '__main__':
    main()
