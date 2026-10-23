# Architecture and data contracts

## One execution path

`app.py → gui.App → owned subprocess → cli.main → pipeline.run`

`main.py / python -m ccpp_mowing plan → cli.main → pipeline.run`

The GUI does not contain an alternate simplified planner. Worker input is a saved Config; output is a folder whose status is written last. Failed/cancelled folders cannot be loaded as successful snapshots.

## Pipeline

1. **Input contract** (`models`, `geometry`, `maps`). Units are metres/seconds/radians. Reject invalid geometry, unknown parameter names, nonfinite values, degenerate budgets and declared geographic CRS. Original geometry is immutable.
2. **Task/collision separation** (`geometry`). `original_task`, `target`, `manual_predeclared`, `free`, `cspace` differ. The all-heading circular centre space is a safe sufficient pivot test, not a complete SE(2) reachability oracle.
3. **Grid** (`grids`). Mathematical lattice topology is independent of polygon clipping. A clipped cell keeps its lattice centre. Square edge adjacency differs from triangle edge adjacency. All rejected polygons/reasons are exported.
4. **Navigation** (`search`, `navigation`). Persistent sparse graph of safe pivot locations. Exact swept-rectangle straight links can bridge narrow passages; finish/work flags do not remove graph nodes. Query endpoints attach only with safe links. A start not explicitly supplied is chosen in the largest candidate component and all others are logged; a supplied start is never relocated.
5. **Hierarchy** (`order_core`, `ordering`). Connected cluster membership is disjoint and exhaustive over admitted tasks. K-means buckets are split/merged under a hard size cap. Resource-limited exact subproblems are smaller. Cluster representatives and boundary gates are explicit approximations, not vehicle-optimal endpoints.
6. **Coverage connections** (`pipeline`). All chosen task IDs survive into the service ledger. Intermediate navigation points are transit; final new-target connectors are work. Repeat traversal is allowed. Returned paths are checked, and primary/fallback outcomes remain separate.
7. **Motion** (`motion`). Merge only same-kind collinear forward legs; never remove a required geometric incidence. Convert to finite straight, pivot and wait events. Both linear and yaw profiles have bounded scalar acceleration and jerk and zero derivatives at stops. Every action carries a reason.
8. **Validation** (`validation`). Full straight body hull, full all-angle pivot envelope, analytic profile bounds, joins, time origin, all service-centre incidence, return-to-start and area ledger. No reliance on sampled pixels or a numeric solver's success flag.
9. **Export/display** (`pipeline`, `snapshot`, `visualization`, `gui`). Original inputs, configuration, source model, orders, queries, event law, samples, residuals and checksums. JSON is used, not untrusted pickle. `validate_saved` reconstructs the input grid and declared task ledger before rechecking motion.

## Partial completion

A service state is not a collision obstacle. `allow_partial` permits explicit omitted components/queries, not a collision, teleport, missing reason or fabricated safe return. Mandatory centres for the successful subtask plus skipped IDs must reconstruct the admitted original task set. Original-area residual geometry remains visible. `fallback_policy=strict` rejects a primary search failure; `astar_reported` allows bounded recovery with the actual method named. Partial coverage and primary recovery are distinct concepts.

## Complexity boundaries

Let N be admitted centres, K clusters and B the bounded local cluster size. Avoid one global N×N matrix. The grid and navigation graphs are sparse. Local metric closure is at most B×B (B≤180); cluster matrix is capped separately. Beyond that cap a logged streaming nearest-neighbour representative tour uses O(K) storage. Held–Karp is capped at 14 and defaults to 11; exact-local partition changes are recorded. This is not a guarantee of hectare-scale fine-grid real-time performance.

## Reproducibility

Each result carries a seed, dependency versions, metric frame assumption, map copy and exact config. Random seeds reproduce random sequences but wall-clock-limited searches can still differ across machines. Compare primary success and verified recovery separately. Timing tests record wall time, workers and map resolution; no cross-algorithm leaderboard is inferred from integration runs.
