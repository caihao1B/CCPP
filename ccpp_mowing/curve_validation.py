"""Independent v2 trajectory revalidation from curve descriptors and map bytes.

No solver success flag or plot is accepted as evidence. Curvature and motion
bounds are recomputed. Whole-curve body enclosures, continuity, finite steering,
original task ledger and conservative cutter coverage are separately checked.
"""
from __future__ import annotations
import math
import numpy as np
from shapely.geometry import Point,LineString
from shapely.ops import unary_union
from .curves import endpoints,bounds,flatten,length,curvature,certify_collision,wrap
from .curve_motion import constraint_ratios
from .motion import Profile
from .geometry import rotation_sweep
from .validation import tool_transition_errors


def coverage_geometry(events,robot,world,cfg):
    pieces=[];exact_trace=[];work=0.;transit=0.
    for e in events:
        if 'curve' in e:
            g=e['curve'];L=length(g)
            if e['kind']=='WORK':
                work+=L
                if g['type']=='LINE':
                    trace=LineString([g['p0'],g['p1']]);reserve=0.;exact_trace.append(trace)
                else:
                    _,xy=flatten(g,cfg.curve_tolerance_m,cfg.curve_step_m)
                    trace=LineString(xy);reserve=cfg.curve_tolerance_m
                    # Known exact endpoints also count for centre-exact mode.
                    a,b,_,_=endpoints(g);exact_trace.extend([Point(a),Point(b)])
                pieces.append(trace.buffer(robot.cutter_radius-reserve,quad_segs=24))
            elif e['kind']=='TRANSIT':transit+=L
        elif e['kind']=='SERVICE':
            pieces.append(Point(e['p0']).buffer(robot.cutter_radius,quad_segs=24));exact_trace.append(Point(e['p0']))
    swept_union=unary_union(pieces);swept=swept_union.intersection(world.original_task)
    original=world.original_task.difference(swept);declared=world.target.difference(swept)
    pre=world.manual_predeclared.difference(swept)
    passed_area=sum(p.intersection(world.original_task).area for p in pieces)
    ratio=lambda total,remaining:1-remaining.area/total.area if total.area else None
    geoms={'swept':swept,'original_remaining':original,'declared_remaining':declared,'predeclared_remaining':pre}
    metrics={'original_task_area_m2':world.original_task.area,'declared_task_area_m2':world.target.area,
        'predeclared_manual_area_m2':world.manual_predeclared.area,'covered_area_m2':swept.area,
        'original_area_coverage_ratio':ratio(world.original_task,original),
        'declared_area_coverage_ratio':ratio(world.target,declared),
        'manual_handover_area_m2':original.area,'declared_residual_area_m2':declared.area,
        'predeclared_manual_remaining_m2':pre.area,'repeated_sweep_area_m2':max(0.,passed_area-swept.area),
        'repeated_sweep_definition':'sum of conservative WORK sweep areas minus their union; endpoint overlap included',
        'work_length_m':work,'transit_length_m':transit,'path_length_m':work+transit,
        'coverage_interpretation':'certified conservative cutter sweep lower bound; rounded corners do not imply centre passage',
        'curve_coverage_error_reserve_m':cfg.curve_tolerance_m,'service_rule':cfg.curve_service_rule}
    return geoms,metrics,swept_union,unary_union(exact_trace)


def validate_curves(world,robot,events,required_positions,cfg):
    errors=tool_transition_errors(events,robot);cert={'curved_body_enclosures':0,'straight_body_sweeps':0,'pivot_envelopes':0,
                    'normalized_profiles':0,'steering_events':0};peak_ratios={};kmax=0.;enclosures=0
    def error(s):
        if len(errors)<100:errors.append(s)
    if not events:error('No trajectory events')
    if events and abs(events[0]['t0'])>1e-9:error('Time does not start at zero')
    for i,e in enumerate(events):
        prefix=f'event {i}: '
        try:
            values=[e['t0'],e['t1'],e['duration'],*e['p0'],*e['p1'],e['yaw0'],e['yaw1'],e.get('steer0',0.),e.get('steer1',0.)]
            if not all(math.isfinite(float(v)) for v in values):raise ValueError('nonfinite state')
            if e['id']!=i:error(prefix+'event id mismatch')
            if e['duration']<=0 or abs(e['t1']-e['t0']-e['duration'])>1e-6:error(prefix+'duration mismatch')
            if e['kind'] not in ('WORK','TRANSIT','TURN','WAIT','SERVICE','STEER'):error(prefix+'unknown action kind')
            if not e.get('reason'):error(prefix+'missing action explanation')
            if e['tool_on']!=(e['kind'] in ('WORK','SERVICE')):error(prefix+'tool-state mismatch')
            if i:
                old=events[i-1]
                if abs(old['t1']-e['t0'])>1e-6 or math.dist(old['p1'],e['p0'])>2e-6 or abs(wrap(old['yaw1']-e['yaw0']))>2e-5:
                    error(prefix+'discontinuous time/position/heading')
                if cfg.motion_model=='bounded_curvature' and abs(old.get('steer1',0.)-e.get('steer0',0.))>2e-5:
                    error(prefix+'instantaneous steering discontinuity')
            if e['kind'] in ('WORK','TRANSIT'):
                if 'curve' not in e:raise ValueError('moving event has no analytical curve')
                g=e['curve'];a,b,ya,yb=endpoints(g)
                if math.dist(a,e['p0'])>2e-6 or math.dist(b,e['p1'])>2e-6 or abs(wrap(ya-e['yaw0']))>2e-5 or abs(wrap(yb-e['yaw1']))>2e-5:
                    error(prefix+'curve descriptor disagrees with event endpoint/tangent')
                okay,n=certify_collision(g,world,robot,cfg.curve_tolerance_m);enclosures+=n
                if not okay:error(prefix+'whole-curve body clearance not certified')
                cert['straight_body_sweeps' if g['type']=='LINE' else 'curved_body_enclosures']+=1
                bd=bounds(g);kmax=max(kmax,bd['curvature'])
                if bd['curvature']>1/cfg.min_turn_radius_m+1e-7:error(prefix+'minimum turning radius violated')
                pr=Profile(**e['profile'])
                if not all(math.isfinite(v) for v in (pr.distance,pr.peak,pr.ramp,pr.cruise)) or min(pr.distance,pr.peak,pr.ramp)<=0 or pr.cruise<0:
                    raise ValueError('invalid normalized curve profile')
                if e.get('parameter')!='u' or abs(pr.distance-1)>1e-8 or abs(pr.peak*(pr.ramp+pr.cruise)-1)>1e-7:
                    error(prefix+'normalized parameter integral mismatch')
                if abs(pr.duration-e['duration'])>1e-6:error(prefix+'curve timing mismatch')
                ratios=constraint_ratios(bd,pr,robot,cfg,e['kind'])
                for k,v in ratios.items():peak_ratios[k]=max(peak_ratios.get(k,0.),v)
                if max(ratios.values())>1+1e-6:error(prefix+'continuous speed/acceleration/jerk/steering bound violated')
                cert['normalized_profiles']+=1
                if cfg.motion_model=='bounded_curvature':
                    ka,kb=curvature(g,[0,1])
                    if abs(e['steer0']-math.atan(cfg.wheelbase_m*ka))>2e-5 or abs(e['steer1']-math.atan(cfg.wheelbase_m*kb))>2e-5:
                        error(prefix+'steering inconsistent with curve curvature')
            else:
                if 'curve' in e:error(prefix+'stationary state contains a translating curve')
                if math.dist(e['p0'],e['p1'])>1e-9:error(prefix+'stationary action translates')
                if not world.free.covers(rotation_sweep(e['p0'],robot,robot.clearance_m)):
                    error(prefix+'stationary body unsafe')
                if e['kind']=='TURN':
                    if cfg.motion_model=='bounded_curvature':error(prefix+'bounded-curvature vehicle pivots')
                    if not cfg.curve_allow_pivot_fallback or cfg.turn_policy=='loops_only':error(prefix+'pivot fallback is disabled by configuration')
                    pr=Profile(**e['profile']);d=abs(e['yaw1']-e['yaw0']);b=robot.track_spacing_m
                    lim=(min(robot.yaw_rate_radps,2*robot.track_speed_mps/b),
                         min(robot.yaw_acceleration_radps2,2*robot.track_acceleration_mps2/b),
                         min(robot.yaw_jerk_radps3,2*robot.track_jerk_mps3/b))
                    cert['pivot_envelopes']+=1
                elif e['kind']=='STEER':
                    if abs(e['yaw1']-e['yaw0'])>1e-9:error(prefix+'steering actuation rotates chassis')
                    pr=Profile(**e['steer_profile']);d=abs(e['steer1']-e['steer0']);lim=(cfg.steering_rate_radps,.8,1.6)
                    cert['steering_events']+=1
                else:
                    if abs(e['yaw1']-e['yaw0'])>1e-9 or abs(e.get('steer1',0.)-e.get('steer0',0.))>1e-9:error(prefix+'waiting/service changes pose or steering')
                    if 'profile' in e or 'steer_profile' in e:error(prefix+'waiting/service has motion profile')
                    continue
                if not all(math.isfinite(v) for v in (pr.distance,pr.peak,pr.ramp,pr.cruise)) or min(pr.distance,pr.peak,pr.ramp)<=0 or pr.cruise<0:
                    raise ValueError('invalid stationary-actuator profile')
                if abs(pr.distance-d)>1e-6 or abs(pr.peak*(pr.ramp+pr.cruise)-d)>1e-6 or abs(pr.duration-e['duration'])>1e-6:
                    error(prefix+'stationary-actuator integral/time mismatch')
                if pr.peak>lim[0]+1e-7 or pr.max_acc>lim[1]+1e-7 or pr.max_jerk>lim[2]+1e-7:error(prefix+'stationary actuator bounds violated')
        except (KeyError,ValueError,TypeError,OverflowError,ZeroDivisionError) as exc:
            error(prefix+str(exc))
    try:
        geoms,metrics,sweep,exact=coverage_geometry(events,robot,world,cfg)
        if cfg.curve_service_rule=='cutter_footprint':
            missing=[u for u,p in required_positions.items() if not sweep.covers(Point(p))]
        else:missing=[u for u,p in required_positions.items() if exact.distance(Point(p))>1e-6]
        if missing:error(f'{len(missing)} original required targets fail the declared service rule')
        area_error=abs(geoms['predeclared_remaining'].area+geoms['declared_remaining'].area-geoms['original_remaining'].area)
        if area_error>1e-5:error('Manual area accounting is inconsistent')
    except (KeyError,ValueError,TypeError,OverflowError,ZeroDivisionError) as exc:
        error('Coverage could not be recomputed: '+str(exc));geoms={};metrics={};missing=list(required_positions);area_error=None
    if cfg.return_to_start and events and math.dist(events[0]['p0'],events[-1]['p1'])>2e-6:error('Requested position return was not completed')
    report={'valid':not errors,'errors':errors,'certificates':cert,'enclosure_count':enclosures,
        'mandatory_centres':len(required_positions),'missing_centre_ids':missing,'service_rule':cfg.curve_service_rule,
        'maximum_certified_curvature_inv_m':kmax,'minimum_certified_moving_radius_m':1/kmax if kmax>1e-12 else None,
        'requested_minimum_radius_m':cfg.min_turn_radius_m,'maximum_continuous_bound_ratios':peak_ratios,
        'manual_area_partition_error_m2':area_error,
        'scope':'analytical curve reference replay, whole-body conservative enclosures and continuous derivative bounds; no tracking-error or hardware guarantee',
        'limitations':['every primitive starts/ends at rest; this is not global time optimality',
                      'curved bodies use an orientation-independent circumscribed disk (conservative)',
                      'coverage is a conservative inner estimate, not a 100% field claim',
                      'return to start constrains position, not final heading or steering']}
    return report,geoms,metrics
