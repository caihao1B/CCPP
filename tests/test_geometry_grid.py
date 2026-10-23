import copy, math
import pytest
from shapely.geometry import Point, box, mapping
from shapely.ops import unary_union
from ccpp_mowing.geometry import FieldMap,World,rectangle,translation_sweep,rotation_sweep
from ccpp_mowing.maps import generate,SHAPES,DENSITIES
from ccpp_mowing.models import Robot,Config,PlanningError
from ccpp_mowing.grids import build_grid,dimensions
from ccpp_mowing.navigation import Navigator

@pytest.mark.parametrize('shape_name',SHAPES)
@pytest.mark.parametrize('density',DENSITIES)
def test_metric_map_contract(shape_name,density):
    m=generate(shape_name,18,14,density,'uniform',3)
    assert m.field.is_valid
    w=World(m,Robot())
    assert w.original_task.symmetric_difference(w.target.union(w.manual_predeclared)).area<1e-8
    assert w.target.intersection(w.manual_predeclared).area<1e-8
    assert m.metadata['seed']==3

@pytest.mark.parametrize('grid_type',['square','triangle'])
@pytest.mark.parametrize('mode',['equal_area','equal_side','tool_cover'])
def test_tessellation(grid_type,mode):
    cfg=Config(grid_type=grid_type,scale_mode=mode,cell_area=.5,cell_side=.7)
    g=build_grid(World(generate(width=10,height=8,density='none'),Robot()),cfg)
    assert max(dict(g.graph.degree()).values()) <= (4 if grid_type=='square' else 3)
    if mode=='equal_area':assert g.area==pytest.approx(.5)
    if mode=='equal_side':assert g.side==pytest.approx(.7)
    if mode=='tool_cover':assert g.circumradius<=.35
    if grid_type=='triangle':
        for c in g.cells:
            lengths=[math.dist(a,b) for a,b in zip(c.vertices,c.vertices[1:]+c.vertices[:1])]
            assert max(lengths)-min(lengths)<1e-9
    polies=[c.polygon for c in g.cells]
    assert sum(p.area for p in polies)-unary_union(polies).area<1e-6


def test_whole_rectangle_gate_not_false_circle_failure():
    f=unary_union([box(0,0,8,3),box(3.3,3,4.7,5),box(0,5,8,8)])
    m=FieldMap.from_geojson({'type':'FeatureCollection','features':[{'type':'Feature','properties':{'kind':'field'},'geometry':mapping(f)}]})
    r=Robot();w=World(m,r);p,q=(4,1.5),(4,6.5)
    from shapely.geometry import LineString
    assert w.point_free(p) and w.point_free(q)
    assert not w.cspace.covers(LineString([p,q]))
    assert w.segment_free(p,q)
    assert w.free.covers(translation_sweep(p,q,r,r.clearance_m))


def test_rotating_corners_and_line_middle_collisions():
    r=Robot();p=(0,0);pocket=box(-.7,-.7,.7,.7)
    assert pocket.covers(rectangle(p,0,r.length_m,r.width_m))
    assert pocket.covers(rectangle(p,math.pi/2,r.length_m,r.width_m))
    assert not pocket.covers(rectangle(p,math.pi/4,r.length_m,r.width_m))
    assert not pocket.covers(rotation_sweep(p,r))
    m=generate(width=12,height=10,density='none');m.obstacles=[Point(6,5).buffer(.6)]
    w=World(m,r)
    assert w.point_free((2,5)) and w.point_free((10,5))
    assert not w.segment_free((2,5),(10,5))


def test_unknown_obstacle_type_and_geographic_crs_rejected():
    data=generate(density='none').original_geojson
    bad=copy.deepcopy(data);bad['features'][0]['properties']['kind']='mystery'
    with pytest.raises(ValueError):FieldMap.from_geojson(bad)
    bad=copy.deepcopy(data);bad['crs']='EPSG:4326'
    with pytest.raises(ValueError):FieldMap.from_geojson(bad)


def test_no_silent_connected_component_drop():
    cfg=Config(cell_area=.8);w=World(generate('split',14,10,'none'),Robot());g=build_grid(w,cfg)
    n=Navigator(w,cfg);keep,skip=n.restrict_tasks(g)
    assert keep and skip
    assert len(keep)+len(skip)==len(g.positions)
    assert all(x['reason']=='different_certified_navigation_component' for x in skip)


def test_grid_budget_guard():
    with pytest.raises(PlanningError):build_grid(World(generate(),Robot()),Config(cell_area=.0001,max_cells=100))
