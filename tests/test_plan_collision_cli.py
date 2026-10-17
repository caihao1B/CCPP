import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest
from shapely.geometry import Polygon, box, mapping

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'validate_plan_collisions.py'
spec = importlib.util.spec_from_file_location('collision_audit', SCRIPT)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def make_field(path, field, obstacles):
    geo = {'type': 'FeatureCollection', 'features': [
        {'type': 'Feature', 'properties': {'kind': 'field'}, 'geometry': mapping(field)},
        *[{'type': 'Feature', 'properties': {'kind': 'obstacle', 'obstacle_id': i},
           'geometry': mapping(o)} for i, o in enumerate(obstacles)],
    ]}
    path.write_text(json.dumps(geo))
    return geo


def run_cli(geojson, output, method, *extra):
    return subprocess.run([sys.executable, '-m', 'fieldplan_offline.cli.plan_boustrophedon',
        '--geojson', str(geojson), '--method', method, '--spacing', '1', '--step', '1',
        '--margin', '0.2', '--out', str(output), '--out_csv', str(output.with_suffix('.csv')),
        *extra], capture_output=True, text=True)


SCENES = [
    (box(0, 0, 10, 8), []),
    (box(0, 0, 10, 8), [box(4, 2, 6, 6)]),
    (Polygon([(0, 0), (8, 0), (8, 8), (6, 8), (6, 2), (2, 2), (2, 8), (0, 8)]), []),
    (box(0, 0, 10, 10), [box(4, 0, 6, 4.5), box(4, 5.5, 6, 10)]),
]


@pytest.mark.parametrize('method', ['continuous', 'bcd'])
@pytest.mark.parametrize('scene', SCENES, ids=['straight', 'obstacle', 'concave', 'narrow'])
def test_cli_exports_safe_json_and_csv(tmp_path, method, scene):
    source, output = tmp_path / 'field.geojson', tmp_path / 'plan.json'
    geo = make_field(source, *scene)
    result = run_cli(source, output, method, '--connect_cell', '0.1')
    assert result.returncode == 0, result.stderr
    plan = json.loads(output.read_text())
    report = audit.audit_plan(plan, geo)
    assert report['safe'], report
    assert report['continuous']
    assert report['path_obstacle_edges'] == report['seam_obstacle_edges'] == 0
    assert set(plan) == {'frame_id', 'field_meta', 'clusters', 'cluster_order', 'segments', 'transitions'}
    rows = list(csv.DictReader(output.with_suffix('.csv').open()))
    assert list(rows[0]) == ['x', 'y', 'qx', 'qy', 'qz', 'qw', 'frame_id', 'segment_id', 'kind', 'cluster_id']
    xy_json = [(p['x'], p['y']) for s in plan['segments'] for p in s['poses']]
    assert [(float(r['x']), float(r['y'])) for r in rows] == xy_json


@pytest.mark.parametrize('method', ['continuous', 'bcd'])
def test_cli_unreachable_produces_no_outputs(tmp_path, method):
    source, output = tmp_path / 'field.geojson', tmp_path / 'plan.json'
    make_field(source, box(0, 0, 10, 8), [box(4, 0, 6, 8)])
    result = run_cli(source, output, method)
    assert result.returncode != 0
    assert 'Planning failed:' in result.stderr
    assert not output.exists()
    assert not output.with_suffix('.csv').exists()


def test_continuous_none_preserves_separate_rows(tmp_path):
    source, output = tmp_path / 'field.geojson', tmp_path / 'plan.json'
    make_field(source, box(0, 0, 10, 8), [box(4, 0, 6, 8)])
    result = run_cli(source, output, 'continuous', '--connect', 'none')
    assert result.returncode == 0, result.stderr
    plan = json.loads(output.read_text())
    assert all(s['kind'] == 'row' for s in plan['segments'])
    assert plan['field_meta']['connect'] == 'none'


@pytest.mark.parametrize('margin', ['0', '0.2'])
@pytest.mark.parametrize('method', ['continuous', 'bcd'])
def test_repository_field_regression(tmp_path, method, margin):
    output = tmp_path / 'plan.json'
    result = run_cli(ROOT / 'field.geojson', output, method, '--margin', margin)
    assert result.returncode == 0, result.stderr
    report = audit.audit_plan(json.loads(output.read_text()), json.loads((ROOT / 'field.geojson').read_text()))
    assert report['safe'] and report['continuous'], report
    if margin != '0':
        assert report['path_obstacle_edges'] == report['seam_obstacle_edges'] == 0


def test_historical_example_has_23_turns_and_62_crossing_edges():
    report = audit.audit_plan(json.loads((ROOT / 'plan_boustrophedon.json').read_text()),
                              json.loads((ROOT / 'field.geojson').read_text()))
    assert (report['segments'], report['turn_segments'], report['obstacles']) == (101, 50, 17)
    assert (report['obstacle_turn_segments'], report['turn_obstacle_edges']) == (23, 62)
    assert any(v['segment_id'] == 'seg_0005' and 0 in v['obstacle_ids'] for v in report['violations'])
    assert not report['safe']


def test_auditor_checks_whole_edges_and_implicit_seams(tmp_path):
    source = tmp_path / 'field.geojson'
    geo = make_field(source, box(0, 0, 10, 10), [box(4, 4, 6, 6)])
    def segment(sid, pts):
        return {'id': sid, 'kind': 'turn', 'poses': [dict(x=x, y=y) for x, y in pts]}
    plan = {'segments': [segment('a', [(1, 5), (9, 5)])]}
    result = audit.audit_plan(plan, geo)
    assert result['unsafe_points'] == 0
    assert result['path_unsafe_edges'] == 1
    plan = {'segments': [segment('a', [(1, 1), (1, 5)]), segment('b', [(9, 5), (9, 9)])]}
    result = audit.audit_plan(plan, geo)
    assert result['path_unsafe_edges'] == 0
    assert result['seam_unsafe_edges'] == 1
    output = tmp_path / 'plan.json'
    output.write_text(json.dumps(plan))
    process = subprocess.run([sys.executable, str(SCRIPT), '--geojson', str(source), '--plan', str(output)],
                             capture_output=True, text=True)
    assert process.returncode == 1
