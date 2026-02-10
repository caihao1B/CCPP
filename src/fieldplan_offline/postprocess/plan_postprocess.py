from __future__ import annotations
from typing import Optional, List, Tuple

import math

from fieldplan_offline.export.plan_schema import Plan, Segment
from fieldplan_offline.postprocess.segment import split_by_turn, split_by_length
from fieldplan_offline.postprocess.downsample import downsample_keep_corners
from fieldplan_offline.postprocess.headings import xy_to_poses


def _segment_length(xy: List[Tuple[float, float]]) -> float:
    if len(xy) < 2:
        return 0.0
    return float(sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(xy, xy[1:])))


def postprocess_plan(
    plan: Plan,
    *,
    split_turn_deg: Optional[float] = None,
    split_max_len: Optional[float] = None,
    downsample_ds: Optional[float] = None,
    corner_deg: float = 25.0,
    min_points: int = 2,
) -> Plan:
    """
    Apply segmentation and downsampling to all plan segments.

    - split_turn_deg: split on heading change (deg). If None, no split.
    - split_max_len: split long polylines into chunks by length (meters). If None, no split.
    - downsample_ds: keep points at least ds apart; always keep corners. If None, no downsample.
    - corner_deg: turning angle threshold used to preserve corners during downsample.
    - min_points: ensure each output segment has >= min_points points.
    """
    new_segments: List[Segment] = []
    for seg in plan.segments:
        xy = [(p.x, p.y) for p in seg.poses]

        chunks = [xy]
        if split_turn_deg is not None:
            chunks = []
            for c in [xy]:
                chunks.extend(split_by_turn(c, turn_thresh_deg=split_turn_deg, min_points=max(min_points, 3)))

        if split_max_len is not None:
            chunks2: List[List[Tuple[float,float]]] = []
            for c in chunks:
                chunks2.extend(split_by_length(c, max_len=split_max_len))
            chunks = chunks2

        # Downsample
        if downsample_ds is not None and downsample_ds > 0:
            chunks = [downsample_keep_corners(c, ds=downsample_ds, corner_deg=corner_deg) for c in chunks]

        # Rebuild segments
        if len(chunks) == 1:
            c = chunks[0]
            if len(c) >= min_points:
                poses = xy_to_poses(c, frame_id=seg.frame_id)
                seg.poses = poses
                seg.meta = dict(seg.meta)
                seg.meta.update({"length_m": _segment_length(c)})
                new_segments.append(seg)
            continue

        for i, c in enumerate(chunks):
            if len(c) < min_points:
                continue
            poses = xy_to_poses(c, frame_id=seg.frame_id)
            meta = dict(seg.meta)
            meta.update({
                "parent_id": seg.id,
                "chunk_idx": i,
                "chunk_count": len(chunks),
                "length_m": _segment_length(c),
                "start": c[0],
                "end": c[-1],
            })
            new_segments.append(Segment(
                id=f"{seg.id}_{i:02d}",
                kind=seg.kind,
                cluster_id=seg.cluster_id,
                frame_id=seg.frame_id,
                poses=poses,
                meta=meta,
            ))

    plan.segments = new_segments
    return plan
