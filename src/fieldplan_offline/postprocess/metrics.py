from __future__ import annotations
from typing import Dict, Tuple, List
import math
import numpy as np

def path_metrics(path: List[int], coord_map: Dict[int, Tuple[float, float]], turn_thresh_deg: float = 30.0, span: int = 2):
    length = len(path)
    unique = len(set(path))
    repeat_ratio = 1 - unique / length if length else 0.0
    if length < span + 1:
        return length, 0, repeat_ratio, 0.0, 0.0
    pts = np.asarray([coord_map[n] for n in path], dtype=float)
    vecs = pts[span:] - pts[:-span]
    seg_len = np.linalg.norm(vecs, axis=1)
    dirs = vecs / np.maximum(seg_len[:, None], 1e-9)
    turns = sharp90 = 0
    for d1, d2 in zip(dirs[:-1], dirs[1:]):
        ang = math.degrees(math.acos(np.clip(float(np.dot(d1, d2)), -1.0, 1.0)))
        if ang > turn_thresh_deg:
            turns += 1
            if ang >= 90.0:
                sharp90 += 1
    avg_seg = float(seg_len.mean()) if len(seg_len) else 0.0
    sharp_ratio = sharp90 / turns if turns else 0.0
    return length, turns, repeat_ratio, avg_seg, sharp_ratio
