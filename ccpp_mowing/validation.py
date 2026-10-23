"""Independent certificates. Never infer validity from a solver success flag."""
from __future__ import annotations
import math
import numpy as np
from shapely.geometry import Point, LineString
from shapely.ops import unary_union
from .models import PlanningError
from .motion import Profile, KJ, wrap
from .geometry import translation_sweep,rotation_sweep


def coverage_geometry(events,robot,world):
    pieces=[];work_length=0.;transit_length=0.
    for e in events:
        if e['kind']=='WORK':
            line=LineString([e['p0'],e['p1']]);work_length+=line.length;pieces.append(line.buffer(robot.cutter_radius,quad_segs=24))
        elif e['kind']=='SERVICE':pieces.append(Point(e['p0']).buffer(robot.cutter_radius,quad_segs=24))
        elif e['kind']=='TRANSIT':transit_length+=math.dist(e['p0'],e['p1'])
    raw_union=unary_union(pieces)
    swept=raw_union.intersection(world.original_task)
    original_remaining=world.original_task.difference(swept)
    declared_remaining=world.target.difference(swept)
    predeclared_remaining=world.manual_predeclared.difference(swept)
    sum_pass_area=sum(p.intersection(world.original_task).area for p in pieces)
    ratio=lambda total,rem:1-rem.area/total.area if total.area else None
    return {'swept':swept,'original_remaining':original_remaining,'declared_remaining':declared_remaining,
            'predeclared_remaining':predeclared_remaining}, {
        'original_task_area_m2':world.original_task.area,'declared_task_area_m2':world.target.area,
        'predeclared_manual_area_m2':world.manual_predeclared.area,'covered_area_m2':swept.area,
        'original_area_coverage_ratio':ratio(world.original_task,original_remaining),
        'declared_area_coverage_ratio':ratio(world.target,declared_remaining),
        'manual_handover_area_m2':original_remaining.area,'declared_residual_area_m2':declared_remaining.area,
        'predeclared_manual_remaining_m2':predeclared_remaining.area,
        'repeated_sweep_area_m2':max(0.,sum_pass_area-swept.area),
        'repeated_sweep_definition':'sum of work primitive footprint areas minus their union; includes shared endpoint disks',
        'work_length_m':work_length,'transit_length_m':transit_length,'path_length_m':work_length+transit_length}


def tool_transition_errors(events, robot):
    """Recompute tool-state timing; coverage credit is not physical blade speed.

    Explicit WAIT events enact nonzero startup/shutdown delays. With a zero
    configured delay a state change may occur directly at an event boundary.
    This checks the declared simulation contract, not real actuator safety.
    """
    errors = []
    state = False
    for i, event in enumerate(events):
        kind = event.get('kind')
        reason = event.get('reason', '')
        if kind == 'WAIT' and reason in ('assumed_tool_spinup', 'assumed_tool_spindown'):
            on = reason == 'assumed_tool_spinup'
            minimum = robot.tool_spinup_s if on else robot.tool_spindown_s
            duration = event.get('duration', float('nan'))
            if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration + 1e-9 < minimum:
                errors.append(f'event {i}: tool transition wait shorter than declared bound')
            state = on
        elif kind != 'WAIT':
            requested = kind in ('WORK', 'SERVICE')
            if requested != state:
                minimum = robot.tool_spinup_s if requested else robot.tool_spindown_s
                if minimum > 1e-9:
                    errors.append(f'event {i}: missing tool transition wait')
                state = requested
    if events and state and robot.tool_spindown_s > 1e-9:
        errors.append('Missing final tool shutdown wait')
    return errors


def validate(world,robot,events,required_positions,return_to_start=True,cfg=None):
    if cfg is not None and cfg.trajectory_method != 'stop_turn_go':
        from .curve_validation import validate_curves
        return validate_curves(world,robot,events,required_positions,cfg)
    if any('curve' in e or e.get('trajectory_v2') for e in events):
        raise ValueError('Curve events require their saved v2 Config for independent validation')
    errors=tool_transition_errors(events,robot);certificates={'translations':0,'rotations':0,'profiles':0};clearance=float('inf')
    def error(s):
        if len(errors)<100:errors.append(s)
    if not events:error('No motion/service events')
    if events and abs(events[0]['t0'])>1e-9:error('Time must begin at zero')
    for i,e in enumerate(events):
        vals=[e['t0'],e['t1'],e['duration'],*e['p0'],*e['p1'],e['yaw0'],e['yaw1']]
        if not all(math.isfinite(v) for v in vals):error(f'event {i}: nonfinite numbers');continue
        if e['kind'] not in ('WORK','TRANSIT','TURN','WAIT','SERVICE'):error(f'event {i}: unknown kind')
        if e['t1']<=e['t0'] or abs(e['t1']-e['t0']-e['duration'])>1e-6:error(f'event {i}: duration mismatch')
        if e['tool_on'] != (e['kind'] in ('WORK','SERVICE')):error(f'event {i}: tool accounting mismatch')
        if not e.get('reason'):error(f'event {i}: missing action reason')
        if i:
            a=events[i-1]
            if abs(a['t1']-e['t0'])>1e-6 or math.dist(a['p1'],e['p0'])>1e-7 or abs(wrap(a['yaw1']-e['yaw0']))>1e-7:error(f'event {i}: discontinuous join')
        if e['kind'] in ('WORK','TRANSIT'):
            sweep=translation_sweep(e['p0'],e['p1'],robot)
            padded=translation_sweep(e['p0'],e['p1'],robot,robot.clearance_m)
            if not world.free.covers(padded):error(f'event {i}: translating body collision/clearance')
            clearance=min(clearance,sweep.distance(world.free.boundary));certificates['translations']+=1
            d=math.dist(e['p0'],e['p1'])
            yaw=math.atan2(e['p1'][1]-e['p0'][1],e['p1'][0]-e['p0'][0])
            if abs(wrap(yaw-e['yaw0']))>1e-7 or abs(e['yaw1']-e['yaw0'])>1e-7:error(f'event {i}: sideways/reverse or changing heading on straight')
            vmax=min(robot.work_speed_mps if e['kind']=='WORK' else robot.transit_speed_mps,robot.track_speed_mps)
            amax=min(robot.linear_acceleration_mps2,robot.track_acceleration_mps2)
            jmax=min(robot.linear_jerk_mps3,robot.track_jerk_mps3)
        elif e['kind']=='TURN':
            sweep=rotation_sweep(e['p0'],robot);padded=rotation_sweep(e['p0'],robot,robot.clearance_m)
            if not world.free.covers(padded):error(f'event {i}: intermediate rotation collision/clearance')
            clearance=min(clearance,sweep.distance(world.free.boundary));certificates['rotations']+=1
            if math.dist(e['p0'],e['p1'])>1e-9:error(f'event {i}: pivot translates')
            d=abs(e['yaw1']-e['yaw0']);b=robot.track_spacing_m
            vmax=min(robot.yaw_rate_radps,2*robot.track_speed_mps/b)
            amax=min(robot.yaw_acceleration_radps2,2*robot.track_acceleration_mps2/b)
            jmax=min(robot.yaw_jerk_radps3,2*robot.track_jerk_mps3/b)
        else:
            if math.dist(e['p0'],e['p1'])>1e-9 or abs(e['yaw1']-e['yaw0'])>1e-9:error(f'event {i}: waiting state moved')
            if not world.free.covers(rotation_sweep(e['p0'],robot,robot.clearance_m)):error(f'event {i}: parked body unsafe')
            if 'profile' in e:error(f'event {i}: waiting with profile')
            continue
        if 'profile' not in e:error(f'event {i}: no motion law');continue
        pr=Profile(**e['profile']);certificates['profiles']+=1
        if not all(math.isfinite(v) for v in (pr.peak,pr.ramp,pr.cruise,pr.distance)) or min(pr.peak,pr.ramp,pr.distance)<=0 or pr.cruise<0:
            error(f'event {i}: invalid profile');continue
        if abs(pr.distance-d)>1e-6 or abs(pr.peak*(pr.ramp+pr.cruise)-d)>1e-6:error(f'event {i}: distance integral mismatch')
        if abs(pr.duration-e['duration'])>1e-6:error(f'event {i}: motion/time mismatch')
        if pr.peak>vmax+1e-7 or pr.max_acc>amax+1e-7 or pr.max_jerk>jmax+1e-7:error(f'event {i}: speed/acceleration/jerk bound violated')
    geoms,metrics=coverage_geometry(events,robot,world)
    # All mandatory centres must have been passed with WORK/SERVICE enabled.
    centre_lines=[LineString([e['p0'],e['p1']]) for e in events if e['kind']=='WORK']
    centre_lines += [Point(e['p0']) for e in events if e['kind']=='SERVICE']
    centre_union=unary_union(centre_lines)
    missing=[u for u,p in required_positions.items() if centre_union.distance(Point(p))>1e-6]
    if missing:error(f'{len(missing)} mandatory centres not reached with service enabled')
    if return_to_start and events and math.dist(events[0]['p0'],events[-1]['p1'])>1e-7:error('Did not return to the requested start')
    area_error=abs(geoms['predeclared_remaining'].area+geoms['declared_remaining'].area-geoms['original_remaining'].area)
    if area_error>1e-5:error('Manual area accounting inconsistent')
    report={'valid':not errors,'errors':errors,'certificates':certificates,'mandatory_centres':len(required_positions),
            'missing_centre_ids':missing,'minimum_conservative_body_clearance_m':clearance if math.isfinite(clearance) else None,
            'scope':'whole straight/rotation primitives + analytic time extrema; not closed-loop or hardware validation',
            'manual_area_partition_error_m2':area_error}
    return report,geoms,metrics
