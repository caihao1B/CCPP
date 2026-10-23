"""Independent audit regressions, added after testing the supplied v2.0.0."""
from dataclasses import replace
import copy,json
from pathlib import Path
import pytest
from ccpp_mowing.models import Config,Robot
from ccpp_mowing.maps import generate
from ccpp_mowing.pipeline import run,save_result,validate_saved
from ccpp_mowing.validation import validate
from ccpp_mowing.curve_validation import validate_curves
from ccpp_mowing.snapshot import load_view

@pytest.fixture(scope='module')
def curve_result():
    cfg=Config(trajectory_method='bezier_slsqp',cell_area=.36,cluster_size=16,
               min_turn_radius_m=.1,allow_partial=False)
    return run(generate(width=7,height=6,density='none'),cfg)

def test_disabled_pivot_policy_is_rechecked(curve_result):
    r=curve_result
    assert any(e['kind']=='TURN' for e in r.events)
    report,_,_=validate_curves(r.world,r.robot,r.events,r.required,
                             replace(r.config,curve_allow_pivot_fallback=False))
    assert not report['valid']
    assert any('pivot fallback is disabled' in e for e in report['errors'])

@pytest.mark.parametrize('mode',['shorten','delete'])
def test_tool_transition_wait_is_rechecked(curve_result,mode):
    r=curve_result;events=copy.deepcopy(r.events)
    assert events[0]['reason']=='assumed_tool_spinup'
    if mode=='shorten':events[0]['duration']=.001
    else:events.pop(0)
    t=0
    for i,e in enumerate(events):
        e['id']=i;e['t0']=t;t+=e['duration'];e['t1']=t
    report,_,_=validate_curves(r.world,r.robot,events,r.required,r.config)
    assert not report['valid']
    assert any('tool transition' in e for e in report['errors'])

@pytest.mark.parametrize('field,value',[
    ('trajectory_duration_s',1.),('declared_area_coverage_ratio',1.),
    ('original_area_coverage_ratio',1.),('event_counts',{}),
    ('curve_primitive_counts',{}),('pivot_fallbacks',999),
    ('centre_completion_ratio',.2),('trajectory_duration_s',float('nan')),
])
def test_displayed_metric_cannot_be_forged(tmp_path,curve_result,field,value):
    save_result(curve_result,tmp_path,plots=False,samples=False)
    f=tmp_path/'metrics.json';m=json.loads(f.read_text());m[field]=value;f.write_text(json.dumps(m))
    report=validate_saved(tmp_path)
    assert not report['valid'],(field,report)
    assert any(field in e for e in report['errors'])

def test_strict_saved_ledger_cannot_move_target_to_skipped(tmp_path,curve_result):
    save_result(curve_result,tmp_path,plots=False,samples=False)
    f=tmp_path/'required_centres.json';req=json.loads(f.read_text());key=next(iter(req));pt=req.pop(key)
    f.write_text(json.dumps(req));(tmp_path/'skipped_tasks.json').write_text(json.dumps([
        {'id':int(key),'center':pt,'reason':'fabricated_skip'}]))
    m=json.loads((tmp_path/'metrics.json').read_text());m['serviced_centres']=len(req)
    m['centre_completion_ratio']=len(req)/m['candidate_centres'];m['unserviced_candidate_centres']=1
    (tmp_path/'metrics.json').write_text(json.dumps(m))
    report=validate_saved(tmp_path)
    assert not report['valid']
    assert any('Strict task policy' in e for e in report['errors'])

def test_zero_tool_wait_bounds_remain_supported():
    rb=Robot(tool_spinup_s=0.,tool_spindown_s=0.)
    cfg=Config(trajectory_method='circular_fillet',cell_area=.64)
    r=run(generate(width=7,height=6,density='none'),cfg,rb)
    assert r.validation['valid']

@pytest.mark.parametrize('filename',['demo_01','demo_02','demo_03','demo_04','demo_05','demo_06','demo_07'])
def test_supplied_demonstrations_pass_strengthened_revalidation(filename):
    folder=Path(__file__).resolve().parents[1]/'examples'/filename
    assert validate_saved(folder)['valid']


def test_legacy_gui_loader_rechecks_metrics(tmp_path):
    r=run(generate(width=6,height=6,density='none'),Config(cell_area=1.0))
    save_result(r,tmp_path,plots=False,samples=False)
    m=json.loads((tmp_path/'metrics.json').read_text());m['trajectory_duration_s']=1
    (tmp_path/'metrics.json').write_text(json.dumps(m))
    with pytest.raises(Exception,match='revalidation'):
        load_view(tmp_path)


@pytest.mark.parametrize('wrapped',[False,True])
def test_cli_custom_robot_roundtrip(tmp_path,wrapped):
    from ccpp_mowing.cli import main
    robot=Robot(length_m=.65,width_m=.48,track_spacing_m=.42)
    data=robot.metadata() if wrapped else robot.metadata()['simulation_parameters']
    path=tmp_path/'robot.json';path.write_text(json.dumps(data),encoding='utf-8')
    cfgpath=tmp_path/'config.json';cfgpath.write_text(json.dumps(Config(cell_area=1.0).to_dict()))
    out=tmp_path/'run'
    code=main(['plan','--robot',str(path),'--config',str(cfgpath),'--width','6','--height','6',
               '--density','none','--out',str(out),'--no-plots','--no-samples'])
    assert code==0
    loaded=json.loads((out/'robot.json').read_text())
    assert loaded['simulation_parameters']['length_m']==.65
    assert loaded['reference'].startswith('Custom research vehicle')
    assert validate_saved(out)['valid']


def test_default_and_custom_robot_reference_are_distinct():
    assert 'Agria' in Robot().metadata()['reference']
    assert Robot(length_m=.65,width_m=.48).metadata()['reference'].startswith('Custom research vehicle')
