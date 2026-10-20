"""Unmodified pure-function excerpt from ROS2_BT- plan_io.py.
Do not fix legacy behavior here: this is the audit oracle. See provenance.json.
"""
import math
from typing import Any, Dict, List, Tuple

def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])

def compute_segment_length_m(poses: List[Dict[str, Any]]) -> float:
    if len(poses) < 2:
        return 0.0
    total = 0.0
    prev = (float(poses[0]["x"]), float(poses[0]["y"]))
    for p in poses[1:]:
        cur = (float(p["x"]), float(p["y"]))
        total += _dist(prev, cur)
        prev = cur
    return total

def merge_short_segments(
    segments: List[Dict[str, Any]],
    length_threshold_m: float = 1.0,
    min_poses: int = 3,
) -> List[Dict[str, Any]]:
    """Merge short segments into the previous segment.

    Rule (guard rail):
    - if segment_length < length_threshold_m OR poses < min_poses -> merge into previous
    """
    if not segments:
        return segments

    merged: List[Dict[str, Any]] = []
    for seg in segments:
        poses = seg.get("poses", [])
        seg_len = float(seg.get("meta", {}).get("length_m", compute_segment_length_m(poses)))
        too_short = (seg_len < length_threshold_m) or (len(poses) < min_poses)

        if too_short and merged:
            prev = merged[-1]
            prev_poses = prev.get("poses", [])

            # Avoid duplicating the junction pose if identical
            if prev_poses and poses:
                last = prev_poses[-1]
                first = poses[0]
                if float(last.get("x", 0.0)) == float(first.get("x", 0.0)) and float(last.get("y", 0.0)) == float(first.get("y", 0.0)):
                    poses = poses[1:]

            prev_poses.extend(poses)
            prev["poses"] = prev_poses

            # Update meta (best-effort)
            prev.setdefault("meta", {})
            prev["meta"]["merged"] = True
            prev["meta"].setdefault("merged_from", [])
            prev["meta"]["merged_from"].append(seg.get("id", ""))

            prev_len = float(prev["meta"].get("length_m", compute_segment_length_m(prev_poses)))
            prev["meta"]["length_m"] = prev_len + seg_len
        else:
            merged.append(seg)
    return merged
