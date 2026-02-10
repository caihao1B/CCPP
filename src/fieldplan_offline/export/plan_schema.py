from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Literal

SegmentKind = Literal["intra_cluster", "bridge", "row", "turn", "patch"]

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    qx: float
    qy: float
    qz: float
    qw: float
    frame_id: str = "map"

@dataclass
class Segment:
    id: str
    kind: SegmentKind
    cluster_id: int
    frame_id: str
    poses: List[Pose2D]
    meta: Dict[str, Any]

@dataclass
class Plan:
    frame_id: str
    field_meta: Dict[str, Any]
    clusters: List[List[int]]
    cluster_order: List[int]
    segments: List[Segment]
    transitions: List[Dict[str, Any]]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
