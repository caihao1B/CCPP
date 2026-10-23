from dataclasses import replace
import copy,json,math
from pathlib import Path
import numpy as np
import pytest
from ccpp_mowing.models import Config,Robot,Cancelled,PlanningError
from ccpp_mowing.motion import Profile,TimeOptimizer,evaluate_events,build_events
from ccpp_mowing.maps import generate
from ccpp_mowing.pipeline import run,save_result,validate_saved
from ccpp_mowing.validation import validate
from ccpp_mowing.geometry import FieldMap

@pytest.mark.parametrize('distance',[.0001,.02,.2,.63,2.,9.,60.,math.pi/2])
@pytest.mark.parametrize('mode',['conservative','slsqp'])
def test_time_profiles_global_extrema_and_integrals(distance,mode):
    opt=TimeOptimizer(mode);p,base=opt.solve(distance,.8,.35,.7)
    t=np.linspace(0,p.duration,1001);s,v,a,j=p.at(t)
    assert s[0]==pytest.approx(0) and s[-1]==pytest.approx(distance,abs=1e-7)
    assert min(np.diff(s))>=-1e-10
    assert max(v)<=.8+1e-8 and max(abs(a))<=.35+1e-8 and max(abs(j))<=.7+1e-8
    assert max(abs(v[[0,-1]]))<1e-10 and max(abs(a[[0,-1]]))<1e-10
    assert p.duration<=base+1e-7


def test_profile_derivatives():
    p=Profile.feasible(4,.8,.35,.7);t=np.linspace(.001,p.duration-.001,501);h=1e-5
    sp,vp,ap,_=p.at(t+h);sm,vm,am,_=p.at(t-h);s,v,a,j=p.at(t)
    assert max(abs((sp-sm)/(2*h)-v))<1e-7
    assert max(abs((vp-vm)/(2*h)-a))<1e-7
    assert max(abs((ap-am)/(2*h)-j))<1e-6

@pytest.mark.parametrize('grid_type',['square','triangle'])
@pytest.mark.parametrize('partition',['none','bfs','kmeans'])
def test_complete_small_pipeline_and_track_limits(grid_type,partition,tmp_path):
    r=run(generate(width=10,height=8,density='low'),Config(grid_type=grid_type,cell_area=.6,partition=partition,cluster_size=18))
    assert r.validation['valid'] and r.metrics['centre_completion_ratio']==1
    a=evaluate_events(r.events,np.linspace(0,r.events[-1]['t1'],4001),r.robot);b=r.robot.track_spacing_m
    assert max(abs((a[:,8]+a[:,9])/2-a[:,4]))<1e-9
    assert max(abs((a[:,9]-a[:,8])/b-a[:,5]))<1e-9
    assert max(abs(a[:,8:10]).ravel())<=r.robot.track_speed_mps+1e-7
    assert max(abs(a[:,10:12]).ravel())<=r.robot.track_acceleration_mps2+1e-7
    assert r.metrics['covered_area_m2']+r.metrics['manual_handover_area_m2']==pytest.approx(r.world.original_task.area)
    save_result(r,tmp_path,plots=False,samples=False)
    assert validate_saved(tmp_path)['valid']


def test_partial_is_not_hidden_as_full_coverage():
    r=run(generate('split',14,10,'none'),Config(cell_area=.7))
    assert r.skipped and r.metrics['centre_completion_ratio']<1
    assert r.metrics['status']=='verified_with_manual_residual'
    assert r.metrics['manual_handover_area_m2']>40
    with pytest.raises(PlanningError):run(generate('split',14,10,'none'),Config(cell_area=.7,allow_partial=False))


def test_cancelled_is_not_partial_success():
    with pytest.raises(Cancelled):run(generate(),Config(),cancel=lambda:True)

@pytest.mark.parametrize('mutation',['shift_time','remove_required','fake_area','sideways','unsafe_profile','status'])
def test_saved_tampering_rejected(mutation,tmp_path):
    r=run(generate(width=8,height=7,density='none'),Config(cell_area=1.))
    save_result(r,tmp_path,plots=False,samples=False)
    def read(n):return json.loads((tmp_path/n).read_text())
    def write(n,d):(tmp_path/n).write_text(json.dumps(d))
    if mutation=='shift_time':
        es=read('events.json')
        for e in es:e['t0']+=50;e['t1']+=50
        write('events.json',es)
    elif mutation=='remove_required':
        d=read('required_centres.json');d.pop(next(iter(d)));write('required_centres.json',d)
    elif mutation=='fake_area':
        d=read('metrics.json');d['covered_area_m2']+=10;write('metrics.json',d)
    elif mutation=='sideways':
        es=read('events.json');e=next(e for e in es if e['kind']=='WORK');e['yaw0']+=.5;write('events.json',es)
    elif mutation=='unsafe_profile':
        es=read('events.json');e=next(e for e in es if 'profile' in e);e['profile']['peak']*=100;write('events.json',es)
    else:
        d=read('status.json');d['status']='running';write('status.json',d)
        with pytest.raises(PlanningError):validate_saved(tmp_path)
        return
    assert not validate_saved(tmp_path)['valid']

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-1,0])
def test_invalid_parameter_guard(value):
    with pytest.raises(ValueError):Config(cell_area=value).validate()


def test_unknown_keys_and_profile_model_mixing_rejected():
    with pytest.raises(ValueError):Config.from_dict({'ros_frame_id':'map'})
    with pytest.raises(ValueError):run(generate(),Config(tool_radius=.9))


def test_event_sampling_preserves_unsorted_input_and_scalar_results():
    from ccpp_mowing.motion import evaluate_events
    cfg=Config(cell_area=1.,cluster_size=8)
    result=run(generate(width=8,height=7,density='none'),cfg)
    ts=np.linspace(-1,result.events[-1]['t1']+1,83)
    np.random.default_rng(9).shuffle(ts)
    together=evaluate_events(result.events,ts,result.robot)
    separate=np.vstack([evaluate_events(result.events,[t],result.robot)[0] for t in ts])
    assert np.allclose(together,separate,atol=1e-12)
    assert evaluate_events(result.events,[],result.robot).shape==(0,14)
    with pytest.raises(ValueError):evaluate_events(result.events,[float('nan')],result.robot)
