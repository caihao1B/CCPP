from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import math

from fieldplan_offline.export.plan_schema import Pose2D

def yaw_to_quat(yaw: float) -> Tuple[float,float,float,float]:
    # roll=pitch=0
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw/2.0)
    qw = math.cos(yaw/2.0)
    return (qx,qy,qz,qw)

def xy_to_poses(points: List[Tuple[float,float]], frame_id: str = "map") -> List[Pose2D]:
    if not points:
        return []
    poses: List[Pose2D] = []
    for i, (x,y) in enumerate(points):
        if i < len(points) - 1:
            x2,y2 = points[i+1]
            yaw = math.atan2(y2-y, x2-x)
        else:
            x1,y1 = points[i-1] if i>0 else (x,y)
            yaw = math.atan2(y-y1, x-x1) if i>0 else 0.0
        qx,qy,qz,qw = yaw_to_quat(yaw)
        poses.append(Pose2D(x=x, y=y, yaw=yaw, qx=qx, qy=qy, qz=qz, qw=qw, frame_id=frame_id))
    return poses
