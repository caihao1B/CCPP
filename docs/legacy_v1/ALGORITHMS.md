# Algorithms: theory mapped to this implementation

This document defines the delivered algorithms, including their restrictions. Python implementations live in this repository; OMPL is a conceptual reference, not a dependency.

## Three different optimizations

**Task order** chooses which grass obligations come next. **Connection planning** chooses collision-free travel between obligations. **Time-profile optimization** determines finite-rate execution of the resulting straight/pivot primitives. A shortest individual A* connection does not make the complete tour shortest. A smooth speed law does not prove coverage; coverage is checked separately.

## Discretization

For square side a: area a², four edge neighbours. For equilateral-triangle side s: area √3 s²/4, three edge neighbours. The latter uses a real triangular lattice, not a square grid with a diagonal decoration. `equal_area` uses matching nominal areas, not matching edges; `tool_cover` makes the circumradius slightly smaller than the cutter radius. Buffer/footprint area is still evaluated on the original continuous map. A clipped edge cell may be discarded under the explicit fraction/pivot rules and must appear in the residual accounting.

## Connected clustering

BFS region growth has a hard capacity. K-means uses centre coordinates but its labels need not be connected: split each label into graph components, enforce the capacity and merge compatible small adjacent groups. `cluster_size` always means nodes per cluster. The 6-node minimum is a soft goal; thin/disconnected regions can remain smaller. `none` is still subject to local matrix budget and connectivity; any subdivision is logged.

## Intra-cluster methods

| Name | Implementation and intended interpretation |
|---|---|
| `serpentine` | Alternating row order over the actual task lattice; not continuous BCD |
| `column_sweep` | Column-oriented counterpart |
| `greedy` | Greedily selects graph-near outstanding goals; routes in the graph |
| `nearest` | Nearest-neighbour order using local graph metric |
| `nn2opt` | Nearest seed followed by improving route reversals |
| `dfs` | Depth-first graph traversal, with recorded revisits |
| `mst_euler` | Spanning tree, endpoint-aware odd-vertex augmentation and full Euler trail; never cut off early upon first reaching the endpoint |
| `double_tree` | Double-tree/Euler traversal with endpoint handling; repeats may be preserved |
| `ga` | Fixed-endpoint population permutation search, mutation/crossover and 2-opt; tests protect the mandatory permutation |
| `sa` | Seeded simulated annealing on local order |
| `christofides` | Christofides cycle from a metric graph, adapted to requested open endpoints; no 3/2 guarantee claimed for this adaptation |
| `exact` | Held–Karp dynamic programming within the configured small-node limit |

Graph distances are metric lengths rather than an arbitrary edge count. The 2-opt evaluator handles asymmetric matrices with a full changed-segment cost, instead of wrongly applying symmetric boundary-only deltas. When exact solving would exceed its limit, normal mode creates smaller exact-local clusters; strict or impossible resource requests can fail. Required/actual method and altered partition are exported.

## Inter-cluster methods and gates

Nearest, NN+2-opt, GA, SA, Christofides and Held–Karp operate on a **cluster representative** cost matrix. Default cost is Euclidean distance between medoids (a routing proxy); `navigation_graph` builds actual navigation-graph medoid distances. Neither includes all internal cluster trajectories. Entry/exit candidate scoring is a bounded geometric gate heuristic, and safe route feasibility is checked later. There is no global GTSP/vehicle-time optimality claim. A return query reverses endpoint roles rather than reusing a forward gate unchanged.

An oversized cluster matrix triggers `nearest_streaming` only in recovery mode and logs that this is not the requested solver. Exact cluster-level optimization above the limit is similarly reported. Never compare an exact-vs-GA label without inspecting `ordering.json`.

## A* and Dijkstra

A* uses f=g+h with Euclidean lower bound for geometric navigation costs; stale queue records are skipped and improved g values are reopened. Dijkstra is h=0. Node/edge checks are shared. In the educational turn-penalty variant, previous direction is part of the state. A* optimality is over its particular graph and cost, not every continuous curve and not the full coverage tour.

## Sampling planners

RRT repeatedly samples, finds a nearest tree node, steers a bounded step and checks the whole connection. RRT-Connect grows two trees and reconstructs the correct orientation. RRT* chooses a cost-improving parent and rewires neighbours with descendant cost propagation. A finite iteration/time budget is never an optimality guarantee. PRM builds a reusable sampled roadmap; query endpoint nodes are temporary and do not contaminate future queries. Failed construction/query is explicit.

Shared visibility checks can finish easy queries immediately; they are recorded as `verified_visibility`, not as proof of random exploration. Primary comparisons use a blocked direct route or disable visibility. Normal mode can recover with A*: `queries.json` retains the failed method, reason and real method. Disabling recovery does not prove an environment impossible when sampling times out.

## Collision and motion semantics

Robot body is a centred 1.10×1.05 m rectangle. Cutting tool is an assumed centred disk, radius .35 m. Safe pivot centres use the circumscribed body radius plus margin. Complete straight motion uses the convex hull of endpoint body rectangles, padded for clearance. Complete pivot rotation uses an enclosing regular polygon with radius adjusted so it is an **outer**, not inner, circle approximation. The extra planning reserve prevents geometric roundoff from consuming the verification margin.

A conservative all-heading centre-space failure can be bypassed by an exactly checked straight body sweep between safe endpoints; this permits some narrow gates without promising a full SE(2) planner. The robot cannot pivot at an unsafe endpoint merely because its start/end rectangles fit.

## Actual numerical optimization

No unconstrained XY spline is used. The motion family is a forward straight or a pivot, starting/ending at rest. Use the normalized velocity ramp

`h(z)=10 z³−15 z⁴+6 z⁵`, 0≤z≤1.

For peak speed V, ramp time r, cruise c and displacement d:

- acceleration: `1.875 V/r` maximum;
- jerk: `(10/√3) V/r²` maximum;
- displacement: `d=V(r+c)`;
- duration: `T=2r+c=d/V+r`.

For each distinct primitive, SciPy SLSQP actually optimizes `(V,r)`:

```
minimize d/V + r
subject to 0 < V <= Vmax
           V*r <= d
           1.875*V/r <= Amax
           (10/sqrt(3))*V/r**2 <= Jmax
```

The same formulation applies to positive absolute yaw displacement. Limits are the minimum of relevant chassis and drive-side limits; pivot drive speeds are ±bω/2. Candidate profiles are independently checked and compared against the conservative feasible baseline. Profiles with identical displacement/limits are cached; exact displacement integral is restored after rounded cache indexing. Numerical nonconvergence returns a **labelled safe baseline**, not the last invalid iterate.

This optimizes a small fixed trajectory family, not global path geometry, MPC, force control, CHOMP, TrajOpt, or free-time whole-vehicle direct collocation. Joins occur at rest with zero scalar acceleration and jerk. In-place turns are allowed by model, not accidental loops. Thousands of grid-centre bends and tool waits can make simulated mission time much longer than practical swath mowing. That drawback is reported rather than concealed as commercial productivity.

## Independent validation and residuals

Recompute exact primitive geometry, duration/displacement relation, analytical extrema, body clearance, start/time origin/joins, required-centre service incidence and return location. Raster displays and bounded-rate CSV samples are not collision proofs. The body model, maps, checker and planners still share assumptions; testing is not a safety certification.

Compute unique tool sweep clipped to the original task. The original residual equals predeclared-manual residual plus additional declared-task residual. `repeated_sweep_area` is the sum of primitive swept areas minus their union, including shared endpoint disk overlap; it is not wear or energy. Revisited transit carries no extra coverage credit.

## Primary references

- LaValle, *Planning Algorithms*: https://lavalle.pl/planning/
- LaValle's RRT notes: https://lavalle.pl/rrt/about.html
- Karaman & Frazzoli, *Sampling-based Algorithms for Optimal Motion Planning*: https://arxiv.org/abs/1105.1186
- OMPL planner descriptions: https://ompl.kavrakilab.org/planners.html
- NetworkX Christofides documentation: https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.approximation.traveling_salesman.christofides.html
- SciPy SLSQP documentation: https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html
- Differential-drive model: https://www.roboticsbook.org/S52_diffdrive_actions.html

Accessed 2026-09-14. These sources describe methods; local implementation and empirical claims must be checked against this release's source and test logs.
