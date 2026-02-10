from __future__ import annotations
from typing import Dict, Tuple
import math
import networkx as nx

EPS = 1e-6

def build_cell_graph(grid) -> nx.Graph:
    """
    Square grid 4-neighborhood graph over walkable cells.
    Edge weight=1.
    """
    G = nx.Graph()
    w  = grid.cell_size
    sx = grid.start_x
    sy = grid.start_y

    pos2id: Dict[Tuple[int,int], int] = {}
    for c in grid.cells:
        if not c.walkable:
            continue
        cx, cy = c.center
        gx = int(math.floor(((cx - sx) / w) - 0.5 + EPS))
        gy = int(math.floor(((cy - sy) / w) - 0.5 + EPS))
        pos2id[(gx, gy)] = c.cell_number
        G.add_node(c.cell_number)

    for (gx, gy), cid in pos2id.items():
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nid = pos2id.get((gx+dx, gy+dy))
            if nid is not None:
                G.add_edge(cid, nid, weight=1)
    return G

def grid_index(grid) -> Dict[int, Tuple[int,int]]:
    """
    cell_id -> (row, col) index for square grid.
    """
    idx: Dict[int, Tuple[int,int]] = {}
    w  = grid.cell_size
    sx = grid.start_x
    sy = grid.start_y
    for c in grid.cells:
        if not c.walkable:
            continue
        gx = int(math.floor(((c.center[0] - sx) / w) - 0.5 + EPS))
        gy = int(math.floor(((c.center[1] - sy) / w) - 0.5 + EPS))
        idx[c.cell_number] = (gy, gx)
    return idx
