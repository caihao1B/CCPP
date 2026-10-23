from pathlib import Path
import sys,subprocess,json
from dataclasses import replace
import pytest
from ccpp_mowing.cli import main
from ccpp_mowing.maps import generate
from ccpp_mowing.models import Config,Robot
from ccpp_mowing.geometry import World
from ccpp_mowing.grids import build_grid
from ccpp_mowing.ordering import build_order
from ccpp_mowing.navigation import Navigator
ROOT=Path(__file__).resolve().parents[1]


def test_generate_seed_zero_not_replaced(tmp_path):
    out=tmp_path/'map.geojson'
    assert main(['generate','--seed','0','--out',str(out)])==0
    assert json.loads(out.read_text())['metadata']['seed']==0


def test_main_from_unrelated_directory_and_spaces(tmp_path):
    out=tmp_path/'result with spaces'
    p=subprocess.run([sys.executable,str(ROOT/'main.py'),'--width','8','--height','7','--density','none','--cell-area','1',
                      '--out',str(out),'--no-plots','--no-samples'],cwd=tmp_path,capture_output=True,text=True,timeout=30)
    assert p.returncode==0,p.stdout+p.stderr
    assert json.loads((out/'validation.json').read_text())['valid']
    # The same folder must not be silently overwritten.
    assert main(['plan','--out',str(out)])==2


def test_bad_config_is_saved_failure(tmp_path):
    config=tmp_path/'bad.json';config.write_text('{"cell_area": -2}')
    out=tmp_path/'failure'
    assert main(['plan','--config',str(config),'--out',str(out)])==2
    assert json.loads((out/'status.json').read_text())['status']=='failed'


def test_pre_cancel_flag_cannot_be_complete(tmp_path):
    out=tmp_path/'cancel';out.mkdir();(out/'cancel.flag').touch()
    assert main(['plan','--out',str(out),'--no-plots'])==130
    assert json.loads((out/'status.json').read_text())['status']=='cancelled'


def test_streaming_cluster_budget_logged():
    cfg=Config(cell_area=.6,cluster_size=3,cluster_min=1,max_cluster_matrix=2)
    w=World(generate(width=8,height=7,density='none'),Robot());g=build_grid(w,cfg);n=Navigator(w,cfg)
    o=build_order(g,cfg,n)
    assert o.stats['actual_inter_solver']=='nearest_streaming'
    assert o.stats['warnings'][0]['reason']=='dense_matrix_budget_cap'
    assert set(o.order)==set(g.positions)


def test_navigation_graph_inter_cost():
    cfg=Config(cell_area=.6,cluster_size=10,inter_cost='navigation_graph')
    w=World(generate(width=8,height=7,density='none'),Robot());g=build_grid(w,cfg);n=Navigator(w,cfg)
    o=build_order(g,cfg,n)
    assert set(o.order)==set(g.positions) and o.stats['medoid_cost_model']=='navigation_graph'
