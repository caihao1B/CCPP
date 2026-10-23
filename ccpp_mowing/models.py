"""Typed experiment contracts. SI units throughout; no hardware-control interface."""
from __future__ import annotations
from dataclasses import asdict, dataclass, field
import math
from numbers import Real, Integral

Point2 = tuple[float, float]

class PlanningError(RuntimeError):
    """An explicit budget, connectivity, geometry, or validation failure."""

class Cancelled(PlanningError):
    """A cooperative cancellation; not a successful experiment."""

@dataclass
class SearchResult:
    success: bool
    path: list = field(default_factory=list)
    reason: str = ""
    stats: dict = field(default_factory=dict)
    exploration: list = field(default_factory=list)

SOLVERS = ('serpentine', 'column_sweep', 'greedy', 'nearest', 'nn2opt',
           'dfs', 'mst_euler', 'double_tree', 'ga', 'sa', 'christofides', 'exact')
INTER_SOLVERS = ('nearest', 'nn2opt', 'ga', 'sa', 'christofides', 'exact')
PLANNERS = ('astar', 'dijkstra', 'rrt', 'rrt_connect', 'rrt_star', 'prm')
MOTION_MODELS = ('differential', 'bounded_curvature')
TURN_POLICIES = ('legacy_pivot', 'loop_then_pivot', 'loops_only')
TRAJECTORY_METHODS = ('stop_turn_go', 'circular_fillet', 'bezier_g2', 'bezier_slsqp', 'dubins')

@dataclass(frozen=True)
class Robot:
    """Agria 9500-70 dimensions; ALL dynamic bounds below are assumptions.

    Differential-drive kinematics is only an effective tracked-drive model:
    no terrain contact, slip, power, braking certification or autonomous product
    claim. The footprint is a centred rectangle; cutter offset is unverified.
    """
    length_m: float = 1.10
    width_m: float = 1.05
    cutting_width_m: float = 0.70
    track_spacing_m: float = 0.80
    clearance_m: float = 0.10
    geometry_reserve_m: float = 0.01
    work_speed_mps: float = 0.50
    transit_speed_mps: float = 0.80
    linear_acceleration_mps2: float = 0.35
    linear_jerk_mps3: float = 0.70
    yaw_rate_radps: float = 0.60
    yaw_acceleration_radps2: float = 0.40
    yaw_jerk_radps3: float = 0.80
    track_speed_mps: float = 1.00
    track_acceleration_mps2: float = 0.40
    track_jerk_mps3: float = 0.70
    tool_spinup_s: float = 2.0
    tool_spindown_s: float = 3.0

    @property
    def radius(self):
        return math.hypot(self.length_m, self.width_m) / 2

    @property
    def cutter_radius(self):
        return self.cutting_width_m / 2

    @property
    def planning_radius(self):
        return self.radius + self.clearance_m + self.geometry_reserve_m

    def validate(self):
        for k, v in asdict(self).items():
            if isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v):
                raise ValueError(f'Robot {k}: finite number required')
            if v < 0 or (k not in ('clearance_m','geometry_reserve_m','tool_spinup_s','tool_spindown_s') and v == 0):
                raise ValueError(f'Robot {k}: invalid bound')
        return self

    def metadata(self):
        return {
            'reference': ('Agria 9500-70; dimensional reference, not an autonomous digital twin'
                if (self.length_m,self.width_m,self.cutting_width_m)==(1.10,1.05,.70)
                else 'Custom research vehicle; dimensions and dynamics are simulation assumptions'),
            'manufacturer_facts_for_reference_model_only': {'length_m': 1.10, 'width_m': 1.05, 'cutting_width_m': .70,
                'maximum_published_speed_kmh': 5.8, 'zero_turn_supported': True,
                'source': 'https://agria.de/en/products/remote-controlled-devices/agria-9500-70',
                'verified_date': '2026-09-14'},
            'simulation_parameters': asdict(self),
            'assumptions': ['centred rectangular body and circular cutter',
                f'{self.track_spacing_m:.2f} m effective track spacing is NOT manufacturer verified',
                'speed/acceleration/jerk bounds and tool delays are research settings',
                'tool OFF means no coverage credit; no physical blade-stop certification',
                'flat rigid map, no slip, no crop discrimination, no people, no hardware commands']}

@dataclass
class Config:
    grid_type: str = 'square'
    scale_mode: str = 'equal_area'
    cell_area: float = 0.12
    cell_side: float = 0.35
    min_area_ratio: float = 0.15
    manual_setback_m: float = 0.80
    solver: str = 'serpentine'
    partition: str = 'bfs'
    cluster_size: int = 48
    cluster_min: int = 6
    cluster_solver: str = 'nn2opt'
    inter_cost: str = 'euclidean_medoid'
    planner: str = 'astar'
    seed: int = 7
    rr_step: float = 0.70
    iterations: int = 2200
    timeout: float = 0.75
    goal_bias: float = 0.20
    prm_samples: int = 600
    prm_neighbors: int = 18
    navigation_resolution: float = 0.60
    direct_visible: bool = True
    shortcut_transit: bool = True
    fallback_policy: str = 'astar_reported'
    allow_partial: bool = True
    return_to_start: bool = True
    preserve_mst_walk: bool = True
    timing: str = 'slsqp'
    max_cells: int = 30000
    max_metric_nodes: int = 180
    max_cluster_matrix: int = 500
    ga_population: int = 24
    ga_generations: int = 35
    sa_iterations: int = 600
    exact_limit: int = 11
    tool_radius: float = .35
    start_id: int | None = None
    max_events: int = 160000
    sample_dt_s: float = 0.25
    max_samples: int = 250000

    # v2: defaults keep v1 config/API behavior; GUI presets enable curves.
    motion_model: str = 'differential'
    trajectory_method: str = 'stop_turn_go'
    min_turn_radius_m: float = 0.10
    blend_distance_m: float = 1.50
    curve_tolerance_m: float = 0.002
    curve_step_m: float = 0.05
    curve_work_speed_mps: float = 0.50
    curve_transit_speed_mps: float = 0.80
    curve_acceleration_mps2: float = 0.35
    curve_jerk_mps3: float = 0.70
    lateral_acceleration_mps2: float = 0.35
    wheelbase_m: float = 0.65
    steering_rate_radps: float = 0.50
    # v2.1: legacy inputs retain their behavior; new presets use loops_only.
    turn_policy: str = 'legacy_pivot'
    curve_allow_pivot_fallback: bool = True
    curve_service_rule: str = 'cutter_footprint'
    curve_shape_weight: float = 0.08
    max_curve_primitives: int = 40000

    def validate(self):
        enums = {'grid_type': ('square','triangle'), 'scale_mode': ('equal_area','equal_side','tool_cover'),
            'solver': SOLVERS, 'cluster_solver': INTER_SOLVERS, 'partition': ('none','bfs','kmeans'),
            'planner': PLANNERS, 'timing': ('slsqp','conservative'),
            'inter_cost': ('euclidean_medoid','navigation_graph'),
            'fallback_policy': ('astar_reported','strict'),
            'motion_model': MOTION_MODELS, 'trajectory_method': TRAJECTORY_METHODS,
            'turn_policy': TURN_POLICIES,
            'curve_service_rule': ('cutter_footprint','centre_exact')}
        for k, choices in enums.items():
            if getattr(self,k) not in choices: raise ValueError(f'{k} must be one of {choices}')
        for k in ('cell_area','cell_side','tool_radius','rr_step','timeout','navigation_resolution','sample_dt_s',
                  'min_turn_radius_m','blend_distance_m','curve_tolerance_m','curve_step_m',
                  'curve_work_speed_mps','curve_transit_speed_mps','curve_acceleration_mps2',
                  'curve_jerk_mps3','lateral_acceleration_mps2','wheelbase_m','steering_rate_radps'):
            v=getattr(self,k)
            if isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) or v<=0:
                raise ValueError(f'{k} must be finite and > 0')
        for k in ('manual_setback_m','curve_shape_weight'):
            v=getattr(self,k)
            if isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) or v<0: raise ValueError(k)
        for k in ('min_area_ratio','goal_bias'):
            v=getattr(self,k)
            if not isinstance(v,Real) or not math.isfinite(v) or not 0<=v<1:raise ValueError(k)
        for k in ('seed','cluster_size','cluster_min','iterations','prm_samples','prm_neighbors','max_cells',
                  'max_metric_nodes','max_cluster_matrix','ga_population','ga_generations','sa_iterations',
                  'exact_limit','max_events','max_samples','max_curve_primitives'):
            v=getattr(self,k)
            if isinstance(v,bool) or not isinstance(v,Integral) or v<(0 if k=='seed' else 1):raise ValueError(k)
        for k in ('direct_visible','shortcut_transit','allow_partial','return_to_start','preserve_mst_walk','curve_allow_pivot_fallback'):
            if not isinstance(getattr(self,k),bool):raise ValueError(k)
        if self.turn_policy=='loops_only' and self.trajectory_method=='stop_turn_go':
            raise ValueError('loops_only requires a curve method, not stop_turn_go')
        if self.motion_model=='bounded_curvature' and self.trajectory_method=='stop_turn_go':
            raise ValueError('A bounded-curvature robot cannot use stop_turn_go. Select a curve method; pivots are never permitted in this mode.')
        if not 0.0002<=self.curve_tolerance_m<=0.02:raise ValueError('curve_tolerance_m must be 0.0002..0.02 m')
        if not 0.005<=self.curve_step_m<=0.25:raise ValueError('curve_step_m must be 0.005..0.25 m')
        if self.max_curve_primitives>100000:raise ValueError('Curve primitive budget capped at 100000')
        if not 4<=self.ga_population<=200:raise ValueError('GA population must be 4..200')
        if self.cluster_size>180:raise ValueError('Cluster size above 180 is deliberately not supported')
        if self.exact_limit>14:raise ValueError('Exact Held-Karp limit above 14 would be unsafe on a desktop')
        if self.max_cluster_matrix>1000:raise ValueError('Cluster matrix is deliberately capped at 1000')
        if self.start_id is not None and (isinstance(self.start_id,bool) or not isinstance(self.start_id,Integral)):
            raise ValueError('start_id must be an integer or null')
        return self

    def to_dict(self): return asdict(self)

    @classmethod
    def from_dict(cls,data):
        unknown=set(data)-set(cls.__dataclass_fields__)
        if unknown:raise ValueError(f'Unknown configuration keys: {sorted(unknown)}. Do not use an older project configuration.')
        return cls(**data).validate()
