from __future__ import annotations
import csv
from pathlib import Path
from fieldplan_offline.export.plan_schema import Plan

def save_flat_path_csv(plan: Plan, out_path: str | Path) -> Path:
    """
    Flatten all segments into one CSV with x,y,qx,qy,qz,qw,frame_id.
    """
    p = Path(out_path)
    with p.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x","y","qx","qy","qz","qw","frame_id","segment_id","kind","cluster_id"])
        for seg in plan.segments:
            for pose in seg.poses:
                w.writerow([pose.x, pose.y, pose.qx, pose.qy, pose.qz, pose.qw, pose.frame_id, seg.id, seg.kind, seg.cluster_id])
    return p
