import math

import networkx as nx
import pytest
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from fieldplan_offline.io.path_safety import NoSafePathError, validate_path
from fieldplan_offline.planners.boustrophedon_bcd import (
    build_grid_connector, cover_cell_boustrophedon, plan_boustrophedon_bcd,
)


def assert_covered(free, points):
    assert points
    for a, b in zip(points, points[1:]):
        assert free.buffer(1e-8).covers(Point(a) if a == b else LineString([a, b]))


U_FIELD = Polygon([(0, 0), (8, 0), (8, 8), (6, 8), (6, 2), (2, 2), (2, 8), (0, 8)])


@pytest.mark.parametrize('field,obstacles,a,b,cell', [
    (box(0, 0, 10, 10), [], (1, 1), (9, 9), 0.5),
    (box(0, 0, 10, 10), [box(4, 3, 6, 7)], (1, 5), (9, 5), 0.5),
    # Only a 0.6 m passage joins the two halves; the direct diagonal hits walls.
    (box(0, 0, 10, 10), [box(4, 0, 6, 4.7), box(4, 5.3, 6, 10)], (1, 2), (9, 8), 0.1),
    (U_FIELD, [], (1, 7), (7, 7), 0.5),
])
def test_connector_preserves_endpoints_and_complete_edges(field, obstacles, a, b, cell):
    free = field.difference(unary_union(obstacles))
    connector = build_grid_connector(field, obstacles, cell_size=cell)
    points = connector.shortest_path_xy(a, b)
    assert points[0] == a
    assert points[-1] == b
    assert_covered(free, points)
    if free.covers(LineString([a, b])):
        assert points == [a, b]
    else:
        assert len(points) > 2


def test_endpoint_uses_visible_node_instead_of_nearest_across_wall():
    # Nearest center to a lies across the narrow notch; another visible node exists.
    field = box(0, 0, 4, 4).difference(box(0.8, 1, 0.9, 4))
    connector = build_grid_connector(field, [], cell_size=0.5)
    a, b = (0.91, 3.7), (0.5, 3.5)
    nearest = min(connector.node_center, key=lambda n: math.dist(a, connector.node_center[n]))
    assert not field.covers(LineString([a, connector.node_center[nearest]]))
    points = connector.shortest_path_xy(a, b)
    assert points[0] == a and points[-1] == b
    assert_covered(field, points)


def test_grid_edges_cannot_cross_thin_concave_notch():
    field = box(0, 0, 4, 4).difference(box(0.99, 1, 1.01, 4))
    connector = build_grid_connector(field, [], cell_size=0.5)
    for u, v in connector.graph.edges:
        assert field.covers(LineString([connector.node_center[u], connector.node_center[v]]))
    points = connector.shortest_path_xy((0.75, 3.5), (1.25, 3.5))
    assert min(p[1] for p in points) <= 1
    assert_covered(field, points)


def test_only_checking_vertices_misses_collision():
    free = box(0, 0, 10, 10).difference(box(4, 4, 6, 6))
    points = [(1, 5), (9, 5)]
    assert all(free.covers(Point(p)) for p in points)
    with pytest.raises(NoSafePathError, match='unsafe edge'):
        validate_path(free, points)


def test_disconnected_regions_fail_but_smaller_component_is_retained():
    field, obstacles = box(0, 0, 10, 10), [box(3, 0, 4, 10)]
    connector = build_grid_connector(field, obstacles, cell_size=0.5)
    assert nx.number_connected_components(connector.graph) == 2
    # The smaller component remains routable, including grid-only use.
    points = connector.shortest_path_xy((1, 1), (2, 9), include_endpoints=False)
    assert all(p[0] < 3 for p in points)
    with pytest.raises(NoSafePathError, match='No safe grid connection'):
        connector.shortest_path_xy((1, 5), (9, 5))


def test_coarse_grid_may_fail_narrow_passage_without_straight_fallback():
    connector = build_grid_connector(box(0, 0, 10, 10),
        [box(4, 0, 6, 4.7), box(4, 5.3, 6, 10)], cell_size=0.5)
    with pytest.raises(NoSafePathError):
        connector.shortest_path_xy((1, 2), (9, 8))


def test_outside_endpoint_and_empty_grid_fail_explicitly():
    connector = build_grid_connector(box(0, 0, 2, 2), [], cell_size=0.5)
    with pytest.raises(NoSafePathError):
        connector.shortest_path_xy((-1, 1), (1, 1))
    with pytest.raises(NoSafePathError, match='No walkable'):
        build_grid_connector(box(0, 0, 2, 2), [box(0, 0, 2, 2)], cell_size=0.5)


def test_missing_endpoint_attachment_fails_instead_of_omitting_endpoint():
    field = box(0, 0, 8, 4)
    # Left room is too small to contain any unblocked grid cell.
    obs = [box(0.2, 0, 1.5, 4)]
    connector = build_grid_connector(field, obs, cell_size=0.5)
    with pytest.raises(NoSafePathError, match='No visible'):
        connector.shortest_path_xy((0.1, 2), (7, 2))


def test_zero_length_path_and_large_tolerance_do_not_bypass_collision():
    connector = build_grid_connector(box(0, 0, 10, 10), [box(4, 4, 6, 6)], cell_size=0.5)
    assert connector.shortest_path_xy((1, 1), (1, 1)) == [(1, 1)]
    pts = connector.shortest_path_xy((1, 5), (9, 5), tol=10)
    assert len(pts) > 2
    assert_covered(connector.free_r, pts)


def test_cell_without_connector_must_fail_on_unsafe_turn():
    # A triangular notch makes the cheap horizontal connection cross the exterior.
    field = Polygon([(0, 0), (6, 0), (6, 6), (3, 2), (0, 6)])
    with pytest.raises(NoSafePathError):
        cover_cell_boustrophedon(field, swath_spacing=4, sample_step=0.5)


@pytest.mark.parametrize('order', ['dfs', 'tsp'])
def test_bcd_unreachable_cells_fail(order):
    with pytest.raises(NoSafePathError):
        plan_boustrophedon_bcd(box(0, 0, 10, 6), [box(4, 0, 6, 6)],
            swath_spacing=1, sweep_angle_deg=0, connector_cell_size=0.5,
            cell_order_mode=order)


@pytest.mark.parametrize('angle', [17, 63])
def test_bcd_rotation_margin_and_intra_cell_connectors(angle):
    field, obs = box(0, 0, 12, 8), [box(4, 2, 7, 5)]
    segments, _ = plan_boustrophedon_bcd(field, obs, swath_spacing=1, margin=0.2,
        sweep_angle_deg=angle, connector_cell_size=0.25)
    points = [p for s in segments for p in s['points']]
    assert_covered(field.buffer(-0.2).difference(obs[0].buffer(0.2)), points)
    for a, b in zip(segments, segments[1:]):
        assert math.dist(a['points'][-1], b['points'][0]) < 1e-8
