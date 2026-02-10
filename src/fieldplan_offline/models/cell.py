from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
from shapely.geometry import Polygon

@dataclass
class Cell:
    """
    Minimal cell structure used by grid/graph planners.

    - cell_number: unique id
    - points: polygon exterior coords (without repeated last point)
    - center: (x,y)
    - walkable: True if free space
    """
    cell_number: int
    points: List[Tuple[float, float]]
    center: Tuple[float, float]
    walkable: bool

    @property
    def poly(self) -> Polygon:
        return Polygon(self.points)
