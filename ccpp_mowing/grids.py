"""Regular square / equilateral-triangle decomposition on the SAME physical map.

The graph is shared-EDGE adjacency: <=4 neighbours for squares, <=3 for
triangles. Clipped boundary cells keep their original regular-grid centres.
No connected component is silently dropped. Physical area is a separate metric.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from collections import defaultdict
import networkx as nx
from shapely.geometry import Polygon, Point, mapping
from .models import Config, PlanningError, Point2
from .geometry import World

@dataclass
class Cell:
    id: int
    vertices: list[Point2]
    vertex_keys: list[tuple[int,int]]
    polygon: object
    task_polygon: object
    center: Point2
    area_fraction: float
    traversable: bool
    reason: str
    row: int
    column: int
    orientation: str

@dataclass
class Grid:
    cells: list[Cell]
    graph: nx.Graph
    positions: dict[int,Point2]
    side: float
    area: float
    circumradius: float
    grid_type: str
    metadata: dict

    @property
    def by_id(self): return {c.id:c for c in self.cells}

    def to_geojson(self):
        return {"type":"FeatureCollection","metadata":self.metadata,"features":[
            {"type":"Feature","properties":{"id":c.id,"traversable":c.traversable,
             "reason":c.reason,"center":c.center,"row":c.row,"column":c.column,
             "orientation":c.orientation,"area_fraction":c.area_fraction},
             "geometry":mapping(c.polygon)} for c in self.cells]}


def dimensions(cfg:Config):
    cfg.validate()
    tri=cfg.grid_type=="triangle"
    if cfg.scale_mode=="equal_area":
        side=math.sqrt(4*cfg.cell_area/math.sqrt(3)) if tri else math.sqrt(cfg.cell_area)
    elif cfg.scale_mode=="equal_side": side=cfg.cell_side
    else: side=.98*cfg.tool_radius*(math.sqrt(3) if tri else math.sqrt(2))
    area=math.sqrt(3)/4*side**2 if tri else side**2
    radius=side/math.sqrt(3) if tri else side/math.sqrt(2)
    return side,area,radius


def build_grid(world:World,cfg:Config)->Grid:
    side,area,radius=dimensions(cfg)
    x0,y0,x1,y1=world.map.field.bounds
    estimate=((x1-x0)*(y1-y0)/area)+8*((x1-x0+y1-y0)/side)+32
    if estimate>cfg.max_cells*1.3: raise PlanningError("Grid is too fine: increase cell_area/cell_side or max_cells")
    cells=[];edge_owners=defaultdict(list)
    def add(keys,verts,row,col,orientation):
        poly=Polygon(verts)
        if poly.intersection(world.map.field).area<1e-10:return
        target=poly.intersection(world.target)
        fraction=float(target.area/area)
        center=(sum(x for x,y in verts)/len(verts),sum(y for x,y in verts)/len(verts))
        free=world.point_free(center)
        allowed=bool(fraction>=cfg.min_area_ratio and target.area>1e-10 and free)
        reason="free" if allowed else ("no_task_area" if target.area<1e-10 else "insufficient_task_fraction" if fraction<cfg.min_area_ratio else "center_collision")
        cell=Cell(len(cells),verts,keys,poly,target,center,fraction,allowed,reason,row,col,orientation)
        cells.append(cell)
        if allowed:
            for a,b in zip(keys,keys[1:]+keys[:1]):edge_owners[tuple(sorted((a,b)))].append(cell.id)
    if cfg.grid_type=="square":
        for j in range(math.floor(y0/side)-1,math.ceil(y1/side)+1):
            for i in range(math.floor(x0/side)-1,math.ceil(x1/side)+1):
                keys=[(i,j),(i+1,j),(i+1,j+1),(i,j+1)]
                add(keys,[(i*side,j*side) for i,j in keys],j,i,"square")
    else:
        height=math.sqrt(3)*side/2
        for j in range(math.floor(y0/height)-1,math.ceil(y1/height)+1):
            lo=math.floor(x0/side-j/2)-2;hi=math.ceil(x1/side-j/2)+2
            for i in range(lo,hi):
                for keys,name in [([(i,j),(i+1,j),(i,j+1)],"up"),
                                  ([(i+1,j),(i+1,j+1),(i,j+1)],"down")]:
                    verts=[(side*(ii+jj/2),height*jj) for ii,jj in keys]
                    add(keys,verts,j,i,name)
    if len(cells)>cfg.max_cells:raise PlanningError("Too many grid cells; increase grid size")
    positions={c.id:c.center for c in cells if c.traversable}
    if not positions:raise PlanningError("No feasible task-cell centres; reduce cell size or robot clearance")
    graph=nx.Graph()
    for i,p in positions.items():graph.add_node(i,pos=p)
    rejected=0
    for owners in edge_owners.values():
        if len(owners)>2:raise PlanningError("Non-manifold tessellation: more than 2 cells share an edge")
        if len(owners)!=2:continue
        a,b=owners
        if world.segment_free(positions[a],positions[b]):
            graph.add_edge(a,b,weight=math.dist(positions[a],positions[b]))
        else:rejected+=1
    maximum=3 if cfg.grid_type=="triangle" else 4
    if max(dict(graph.degree()).values())>maximum:raise PlanningError("Invalid shared-edge neighbourhood")
    metadata={"grid_type":cfg.grid_type,"scale_mode":cfg.scale_mode,"side_m":side,"regular_cell_area_m2":area,
              "cell_circumradius_m":radius,"tool_radius_m":cfg.tool_radius,
              "tool_disk_covers_entire_regular_cell_at_center":bool(cfg.tool_radius+1e-10>=radius),
              "total_cells":len(cells),"free_cells":len(positions),"blocked_or_excluded_cells":len(cells)-len(positions),
              "grid_edges":graph.number_of_edges(),"max_degree":maximum,"actual_max_degree":max(dict(graph.degree()).values()),
              "graph_components":nx.number_connected_components(graph),"rejected_center_connections":rejected,
              "discarded_components":0,"centre_policy":"regular_centroid_inside_clearance_free_space",
              "boundary_policy":"keep regular centre; task area is clipped; no silent denominator change"}
    return Grid(cells,graph,positions,side,area,radius,cfg.grid_type,metadata)
