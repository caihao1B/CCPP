from __future__ import annotations
import math
from typing import List

import networkx as nx
from shapely.geometry import Polygon, box
from shapely.strtree import STRtree

from fieldplan_offline.models.cell import Cell
from fieldplan_offline.models.graph_square import build_cell_graph

class BaseGrid:
    """
    Base grid:
      - build cells and mark walkable
      - prune isolated components (keep only largest connected walkable component)
    """

    def __init__(
        self,
        field: Polygon,
        obstacles: List[Polygon],
        cell_size: float,
        min_area_ratio: float = 0.2,
        keep_blocked: bool = True,
    ):
        self.field = field
        self.obstacles = obstacles
        self.cell_size = cell_size
        self.min_area_ratio = min_area_ratio
        self.keep_blocked = keep_blocked

        self._obs_tree = STRtree(obstacles) if obstacles else None
        self.cells: List[Cell] = []
        self._build()
        self._prune_isolated()

    def _is_free(self, poly: Polygon) -> bool:
        if not self.field.intersects(poly):
            return False
        if (self.field.intersection(poly).area / poly.area) < self.min_area_ratio:
            return False
        if self._obs_tree:
            for cand in self._obs_tree.query(poly):
                geom = cand if isinstance(cand, Polygon) else self.obstacles[int(cand)]
                if geom.intersects(poly):
                    return False
        return True

    def _build(self) -> None:
        raise NotImplementedError

    def _prune_isolated(self) -> None:
        G: nx.Graph = build_cell_graph(self)
        if G.number_of_nodes() == 0:
            return
        main = max(nx.connected_components(G), key=len)
        for cell in self.cells:
            if cell.walkable and cell.cell_number not in main:
                cell.walkable = False

class SquareGrid(BaseGrid):
    def _build(self) -> None:
        w = self.cell_size
        half = w / 2
        minx, miny, maxx, maxy = self.field.bounds

        start_x = math.floor((minx - w) / w) * w
        start_y = math.floor((miny - w) / w) * w
        nx_cells = int(math.ceil((maxx - start_x) / w)) + 1
        ny_cells = int(math.ceil((maxy - start_y) / w)) + 1

        idx = 1
        for i in range(nx_cells):
            for j in range(ny_cells):
                cx = start_x + i * w + half
                cy = start_y + j * w + half
                poly = box(cx - half, cy - half, cx + half, cy + half)
                ok = self._is_free(poly)
                if ok or self.keep_blocked:
                    self.cells.append(Cell(
                        idx,
                        list(poly.exterior.coords)[:-1],
                        (cx, cy),
                        ok
                    ))
                    idx += 1

        self.start_x = start_x
        self.start_y = start_y
        self.grid_type = "square"

def build_square_grid(
    field: Polygon,
    obstacles: List[Polygon],
    cell_size: float = 3.0,
    min_area_ratio: float = 0.2,
    keep_blocked: bool = True,
) -> SquareGrid:
    return SquareGrid(field, obstacles, cell_size, min_area_ratio, keep_blocked)
