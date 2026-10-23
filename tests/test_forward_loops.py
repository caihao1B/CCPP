"""v2.1: fixed visit order, physical heading-reversal loops and fail-closed behavior."""
from dataclasses import replace
import math
import numpy as np
import pytest
from ccpp_mowing.models import Config,Robot,PlanningError
from ccpp_mowing.geometry import World
from ccpp_mowing.maps import generate
from ccpp_mowing.curve_motion import build_curve_events,evaluate_curve_event
from ccpp_mowing.curve_validation import validate_curves
from ccpp_mowing.motion import evaluate_events
from ccpp_mowing.curves import wrap


def fixture_case(policy='loops_only',radius=.3):
    rb=Robot(length_m=.65,width_m=.48,track_spacing_m=.30)
    w=World(generate(width=12,height=10,density='none'),rb)
    ps=[(3,4),(7,4),(3,4)]
    legs=[{'p0':a,'p1':b,'kind':'WORK','cluster':0,'tasks':[],'reason':'regression_reversal'} for a,b in zip(ps,ps[1:])]
    cfg=Config(trajectory_method='bezier_slsqp',turn_policy=policy,min_turn_radius_m=radius,return_to_start=True)
    return w,rb,legs,cfg

@pytest.mark.parametrize('method',['circular_fillet','bezier_g2','bezier_slsqp','dubins'])
@pytest.mark.parametrize('model',['differential','bounded_curvature'])
def test_reversal_uses_a_moving_loop_not_a_hidden_pivot(method,model):
    w,rb,legs,cfg=fixture_case();cfg=replace(cfg,trajectory_method=method,motion_model=model)
    ev,report=build_curve_events((3,4),legs,w,rb,cfg)
    assert report['heading_loops']==1 and report['pivot_fallbacks']==0
    assert not any(e['kind']=='TURN' for e in ev)
    group=[e for e in ev if e.get('turn_id')=='heading_1']
    assert len(group)>=3
    samples=np.vstack([evaluate_events(ev,np.linspace(e['t0'],e['t1'],40),rb) for e in group])
    assert np.max(np.linalg.norm(samples[:,1:3]-[7,4],axis=1))>.5
    assert np.min(samples[:,4])>=-1e-10
    assert abs(wrap(group[-1]['yaw1']-group[0]['yaw0']))==pytest.approx(math.pi)
    r,_,_=validate_curves(w,rb,ev,{0:(3,4),1:(7,4)},cfg)
    assert r['valid'],r['errors']

@pytest.mark.parametrize('policy,expected_pivot',[('legacy_pivot',True),('loop_then_pivot',False),('loops_only',False)])
def test_three_policies_are_actual_behaviors(policy,expected_pivot):
    w,rb,legs,cfg=fixture_case(policy)
    ev,r=build_curve_events((3,4),legs,w,rb,cfg)
    assert any(e['kind']=='TURN' for e in ev)==expected_pivot

@pytest.mark.parametrize('policy',['loop_then_pivot','loops_only'])
def test_failed_loop_never_silently_changes_radius(policy):
    w,rb,legs,cfg=fixture_case(policy,radius=10.)
    if policy=='loops_only':
        with pytest.raises(PlanningError,match='No safe'):build_curve_events((3,4),legs,w,rb,cfg)
    else:
        ev,r=build_curve_events((3,4),legs,w,rb,cfg)
        assert r['pivot_fallbacks']==1


def test_tampered_pivot_is_rejected_under_loops_only():
    w,rb,legs,cfg=fixture_case('legacy_pivot')
    ev,_=build_curve_events((3,4),legs,w,rb,cfg)
    r,_,_=validate_curves(w,rb,ev,{0:(3,4),1:(7,4)},replace(cfg,turn_policy='loops_only'))
    assert not r['valid'] and any('disabled' in e for e in r['errors'])


def test_event_seams_have_continuous_position_and_heading():
    w,rb,legs,cfg=fixture_case();ev,_=build_curve_events((3,4),legs,w,rb,cfg)
    for a,b in zip(ev,ev[1:]):
        p=evaluate_events(ev,[a['t1']-1e-6,a['t1']+1e-6],rb)
        assert np.linalg.norm(p[0,1:3]-p[1,1:3])<1e-6
        assert abs(wrap(p[0,3]-p[1,3]))<1e-6
    # Analytic forward nonholonomic law, not just small frame jumps.
    for e in ev:
        if 'curve' not in e:continue
        t=np.linspace(.01,.99,31)*e['duration'];h=1e-5
        a=evaluate_curve_event(e,t,rb)
        d=(evaluate_curve_event(e,t+h,rb)['pos']-evaluate_curve_event(e,t-h,rb)['pos'])/(2*h)
        assert np.max(abs(d-a['v'][:,None]*np.c_[np.cos(a['yaw']),np.sin(a['yaw'])]))<2e-6
        assert min(a['v'])>=0

@pytest.mark.parametrize('policy',['bad','teleport',''])
def test_invalid_policies_rejected(policy):
    with pytest.raises(ValueError):Config(turn_policy=policy).validate()


def test_stop_turn_go_cannot_pretend_to_be_loops_only():
    with pytest.raises(ValueError):Config(turn_policy='loops_only').validate()


def test_rejected_optimized_shape_uses_certified_g2_before_long_loop():
    w,rb,legs,cfg=fixture_case()
    ps=[(3,3),(3.6,3),(3.6,3.6)]
    legs=[{'p0':a,'p1':b,'kind':'WORK','cluster':0,'tasks':[],'reason':'right_angle'} for a,b in zip(ps,ps[1:])]
    cfg=replace(cfg,min_turn_radius_m=.18,return_to_start=False)
    ev,r=build_curve_events((3,3),legs,w,rb,cfg)
    assert r['curve_counts'].get('BEZIER5')==1
    assert r['heading_loops']==0
