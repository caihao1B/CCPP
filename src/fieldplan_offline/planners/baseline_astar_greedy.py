from __future__ import annotations
import heapq, math
from dataclasses import dataclass
from typing import List, Dict, Set, Tuple, Optional, Any
import networkx as nx

from fieldplan_offline.models.grid_square import BaseGrid
from fieldplan_offline.models.graph_square import build_cell_graph

@dataclass
class Node:
    id: int
    g: float
    h: float
    f: float
    parent: Optional["Node"] = None
    def __lt__(self, other): 
        return self.f < other.f

class ASPlanner:
    def __init__(self, grid: BaseGrid, start: int | None = None):
        self.grid = grid
        self.G = build_cell_graph(self.grid)
        self.coords = {c.cell_number: c.center for c in grid.cells if c.walkable}
        self.start = start or self._find_start()

    def _find_start(self) -> int:
        cells = [c for c in self.grid.cells if c.walkable]
        if not cells:
            raise ValueError("No walkable cells")
        return min(cells, key=lambda c: (c.center[1], c.center[0])).cell_number

    def _dist(self, a: int, b: int) -> float:
        x1, y1 = self.coords[a]; x2, y2 = self.coords[b]
        return math.hypot(x2-x1, y2-y1)

    def _astar(self, start: int, goal: int) -> List[int]:
        if start == goal:
            return [start]
        open_set: List[Node] = []
        heapq.heappush(open_set, Node(start, 0, self._dist(start, goal), self._dist(start, goal)))
        closed: Set[int] = set()
        g_cost: Dict[int, float] = {start: 0}
        parent: Dict[int, int] = {}

        while open_set:
            node = heapq.heappop(open_set)
            cur = node.id
            if cur == goal:
                path = []
                while cur in parent:
                    path.append(cur); cur = parent[cur]
                path.append(start)
                return path[::-1]
            closed.add(cur)
            for nb in self.G.neighbors(cur):
                if nb in closed:
                    continue
                g_new = g_cost[cur] + self._dist(cur, nb)
                if nb not in g_cost or g_new < g_cost[nb]:
                    parent[nb] = cur
                    g_cost[nb] = g_new
                    h = self._dist(nb, goal)
                    f = g_new + h
                    heapq.heappush(open_set, Node(nb, g_new, h, f))
        return []

    def plan(self) -> Tuple[List[int], Dict[str, Any]]:
        all_cells = {c.cell_number for c in self.grid.cells if c.walkable}
        if not all_cells:
            return [], {"error": "no walkable cells"}
        visited = {self.start}
        path = [self.start]
        cur = self.start
        repeats = 0

        while len(visited) < len(all_cells):
            unvisited = list(all_cells - visited)
            # greedy pick nearest unvisited
            target = min(unvisited, key=lambda t: self._dist(cur, t))
            seg = self._astar(cur, target)
            if not seg:
                visited.add(target)
                continue
            for cell in seg[1:]:
                path.append(cell)
                if cell in visited:
                    repeats += 1
                else:
                    visited.add(cell)
            cur = target

        stats = {
            "total_cells": len(all_cells),
            "visited_cells": len(visited),
            "path_length": len(path),
            "repeat_count": repeats,
        }
        return path, stats
