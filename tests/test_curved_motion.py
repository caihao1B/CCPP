"""v2 tests: analytical geometry, physical limits, negative cases and saved files."""
from dataclasses import replace
import copy,json,math
import numpy as np
import pytest
from shapely.geometry import Point,box,mapping
from ccpp_mowing.models import Config,Robot,PlanningError,Cancelled
from ccpp_mowing.maps import generate
from ccpp_mowing.geometry import World,FieldMap
from ccpp_mowing.curves import (line,arc,bezier,at,bounds,curvature,flatten,endpoints,
                                certify_collision,wrap)
from ccpp_mowing.dubins import candidates
from ccpp_mowing.curve_motion import build_curve_events,evaluate_curve_event,physical_bounds,CurveTimeOptimizer
from ccpp_mowing.curve_validation import validate_curves
from ccpp_mowing.pipeline import run,save_result,validate_saved
from ccpp_mowing.motion import evaluate_events


def make_world():return World(generate(width=12,height=10,density='none'),Robot())


def example_legs():
    ps=[(2,2),(6,2),(6,6),(3,6)]
    return [{'p0':a,'p1':b,'kind':'WORK','cluster':0,'reason':'test','tasks':[]} for a,b in zip(ps,ps[1:])]


@pytest.mark.parametrize('seed',range(20))
def test_six_dubins_families_endpoints_and_radius(seed):
    rng=np.random.default_rng(seed)
    q0=(*rng.uniform(-3,3,2),rng.uniform(-math.pi,math.pi))
    q1=(*rng.uniform(-3,3,2),rng.uniform(-math.pi,math.pi));R=float(rng.uniform(.1,1.5))
    rows=candidates(q0,q1,R)
    assert rows
    for row in rows:
        a,_,ya,_=endpoints(row['curves'][0]);_,b,_,yb=endpoints(row['curves'][-1])
        assert math.dist(a,q0[:2])<1e-6 and math.dist(b,q1[:2])<1e-6
        assert abs(wrap(ya-q0[2]))<1e-6 and abs(wrap(yb-q1[2]))<1e-6
        for g in row['curves']:assert max(abs(curvature(g,np.linspace(0,1,100))))<=1/R+1e-7


def test_same_point_new_heading_requires_a_moving_loop():
    rows=candidates((3,3,0),(3,3,math.pi/2),.5)
    assert rows and rows[0]['length']>1.
    xy=np.vstack([at(g,np.linspace(0,1,30))[0] for g in rows[0]['curves']])
    assert max(np.linalg.norm(xy-[3,3],axis=1))>.3


@pytest.mark.parametrize('h',[.22,.27,.30,.34])
def test_g2_bezier_curvature_bounds_and_endpoint_continuity(h):
    cp=[[-1,0],[-1+h,0],[-1+2*h,0],[0,1-2*h],[0,1-h],[0,1]]
    g=bezier(cp);p,d,a,*_=at(g,[0,1]);bd=bounds(g)
    assert np.max(abs(a))<1e-12
    assert np.max(abs(curvature(g,np.linspace(0,1,10001))))<=bd['curvature']+1e-8
    assert bd['minimum_parameter_speed']>0
    us,xy=flatten(g,tolerance=.001,max_step=.03)
    from shapely.geometry import LineString
    trace=LineString(xy)
    assert max(trace.distance(Point(q)) for q in at(g,np.linspace(0,1,500))[0])<=.001+1e-8


def test_between_sample_arc_collision_is_not_missed():
    # Arc's endpoints are safe; an obstacle lies at its mid-curve, far from chord.
    g=arc((5,5),2,0,math.pi)
    obstacle=Point(5,7).buffer(.20)
    data={'type':'FeatureCollection','features':[
        {'type':'Feature','properties':{'kind':'field'},'geometry':mapping(box(0,0,12,10))},
        {'type':'Feature','properties':{'kind':'obstacle'},'geometry':mapping(obstacle)}]}
    w=World(FieldMap.from_geojson(data),Robot())
    a,b,_,_=endpoints(g)
    assert w.point_free(a) and w.point_free(b)
    assert not certify_collision(g,w,Robot())[0]


@pytest.mark.parametrize('method',['circular_fillet','bezier_g2','bezier_slsqp','dubins'])
@pytest.mark.parametrize('model',['differential','bounded_curvature'])
def test_curve_motion_limits_and_nonholonomic_derivatives(method,model):
    cfg=Config(trajectory_method=method,motion_model=model,min_turn_radius_m=.4,return_to_start=False)
    w=make_world();ev,opt=build_curve_events((2,2),example_legs(),w,Robot(),cfg)
    report,_,_=validate_curves(w,Robot(),ev,{0:(6,2),1:(6,6),2:(3,6)},cfg)
    assert report['valid'],report['errors']
    assert sum(opt['curve_counts'].get(k,0) for k in ('ARC','BEZIER5'))>0
    if model=='bounded_curvature':assert not any(e['kind']=='TURN' for e in ev)
    for e in ev:
        if 'curve' not in e:continue
        ts=np.linspace(.02,.98,33)*e['duration'];v=evaluate_curve_event(e,ts,Robot())
        actual=physical_bounds(bounds(e['curve']),__import__('ccpp_mowing.motion',fromlist=['Profile']).Profile(**e['profile']),Robot(),cfg)
        for key,val in [('v',v['v']),('a',v['a']),('j',v['j']),('w',v['omega']),('alpha',v['alpha']),('yaw_jerk',v['yaw_jerk'])]:
            assert max(abs(val))<=actual[key]+1e-6,(key,method)
        h=1e-5
        plus=evaluate_curve_event(e,ts+h,Robot());minus=evaluate_curve_event(e,ts-h,Robot())
        vel=(plus['pos']-minus['pos'])/(2*h)
        expected=v['v'][:,None]*np.column_stack([np.cos(v['yaw']),np.sin(v['yaw'])])
        assert np.max(abs(vel-expected))<2e-6
        dy=np.array([wrap(a-b) for a,b in zip(plus['yaw'],minus['yaw'])])/(2*h)
        assert max(abs(dy-v['omega']))<2e-6
        assert max(abs((plus['v']-minus['v'])/(2*h)-v['a']))<2e-6
    times=np.linspace(0,ev[-1]['t1'],103);np.random.default_rng(1).shuffle(times)
    together=evaluate_events(ev,times,Robot())
    separate=np.vstack([evaluate_events(ev,[t],Robot())[0] for t in times])
    assert np.allclose(together,separate,atol=1e-10)


@pytest.mark.parametrize('radius',[.25,.45,.65])
def test_radius_changes_actual_arc_shape(radius):
    cfg=Config(trajectory_method='circular_fillet',min_turn_radius_m=radius,return_to_start=False)
    ev,_=build_curve_events((2,2),example_legs(),make_world(),Robot(),cfg)
    arcs=[e['curve'] for e in ev if e.get('curve',{}).get('type')=='ARC']
    assert arcs
    assert all(g['radius']>=radius-1e-7 for g in arcs)
    assert arcs[0]['radius']==pytest.approx(radius)


def test_no_forbidden_fallback_for_impossible_car_turn():
    cfg=Config(motion_model='bounded_curvature',trajectory_method='bezier_g2',min_turn_radius_m=5.,return_to_start=False)
    with pytest.raises(PlanningError,match='No safe'):
        build_curve_events((2,2),example_legs(),make_world(),Robot(),cfg)


def test_cancel_in_curve_generation():
    def stop():raise Cancelled('stop')
    with pytest.raises(Cancelled):build_curve_events((2,2),example_legs(),make_world(),Robot(),Config(trajectory_method='bezier_g2'),stop)


@pytest.mark.parametrize('key,value',[
    ('min_turn_radius_m',0),('min_turn_radius_m',-1),('curve_step_m',float('nan')),
    ('curve_tolerance_m',.2),('lateral_acceleration_mps2',0),('steering_rate_radps',0),
    ('curve_service_rule','secret_drop_targets')])
def test_invalid_curve_parameters_rejected(key,value):
    with pytest.raises(ValueError):Config(**{key:value}).validate()


def test_car_cannot_choose_legacy_pivot_method():
    with pytest.raises(ValueError):Config(motion_model='bounded_curvature').validate()


@pytest.mark.parametrize('mutation',['radius','unsafe_control','timing','drop_target','tool','fake_metric','steering_jump'])
def test_independent_saved_curve_tampering_rejected(tmp_path,mutation):
    cfg=Config(trajectory_method='bezier_slsqp',motion_model='bounded_curvature',min_turn_radius_m=.25,
               manual_setback_m=2.,cell_area=1.,partition='none')
    result=run(generate(width=10,height=8,density='none'),cfg)
    save_result(result,tmp_path,plots=False,samples=False)
    assert validate_saved(tmp_path)['valid']
    read=lambda n:json.loads((tmp_path/n).read_text())
    def write(n,d):(tmp_path/n).write_text(json.dumps(d))
    if mutation=='radius':
        c=read('config.json');c['min_turn_radius_m']=5.;write('config.json',c)
    elif mutation=='drop_target':
        d=read('required_centres.json');d.pop(next(iter(d)));write('required_centres.json',d)
    elif mutation=='fake_metric':
        d=read('metrics.json');d['path_length_m']+=3;write('metrics.json',d)
    else:
        es=read('events.json');e=next(e for e in es if e.get('curve',{}).get('type')=='BEZIER5')
        if mutation=='unsafe_control':e['curve']['control'][2]=[-100.,-100.]
        elif mutation=='timing':e['profile']['peak']*=20
        elif mutation=='tool':e['tool_on']=False
        else:e['steer0']+=.5
        write('events.json',es)
    assert not validate_saved(tmp_path)['valid']


def test_optimizer_records_a_real_objective_and_continuous_certificate():
    c=Config(trajectory_method='bezier_slsqp',min_turn_radius_m=.4,return_to_start=False)
    es,opt=build_curve_events((2,2),example_legs(),make_world(),Robot(),c)
    shapes=[r for r in opt['records'] if 'objective_before' in r]
    assert shapes and any(r['solver_success'] for r in shapes)
    assert all(r['objective_after']<=r['objective_before']+1e-8 for r in shapes)
    assert opt['actual_duration_s']<=opt['baseline_duration_s']+1e-7
    assert all(max(r['continuous_bound_ratios'].values())<=1+1e-7 for r in opt['time_records'])

@pytest.mark.parametrize('radius',[0.,-1.,float('nan'),float('inf')])
def test_malformed_saved_arc_radius_rejected(radius):
    g={'type':'ARC','radius':radius,'center':[0,0],'angle0':0.,'sweep':1.}
    with pytest.raises(ValueError):bounds(g)


def test_GUI_snapshot_does_not_trust_forged_success_flag(tmp_path):
    from ccpp_mowing.snapshot import load_view
    cfg=Config(trajectory_method='circular_fillet',cell_area=.8,cluster_size=12)
    r=run(generate(width=8,height=7,density='none'),cfg);save_result(r,tmp_path,plots=False,samples=False)
    d=json.loads((tmp_path/'config.json').read_text());d['min_turn_radius_m']=5.
    (tmp_path/'config.json').write_text(json.dumps(d))
    with pytest.raises(PlanningError,match='revalidation'):load_view(tmp_path)
