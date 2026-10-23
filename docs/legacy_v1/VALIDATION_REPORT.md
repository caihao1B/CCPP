# Release 1.0.0 — actual qualification record

This report summarizes saved local runs, not a mathematical proof or a product safety certificate. Raw JSON/CSV/logs are committed under `docs/qa/`. Tests apply to the release source and declared model. No Windows or robot tests are implied.

## Results

| Test layer | Recorded result |
|---|---:|
| Unit/regression suite | **110 passed** |
| Algorithm/map matrix | **984/984 independently verified** |
| Candidate-centre completion in matrix | **984/984 cases** |
| Matrix cases with reported A* recovery | **36** |
| Matrix cases with an order/partition resource adjustment | **116** |
| Dense/scaling integration | **18/18 verified** |
| Blocked-direct-route primary-only tests | **18/18**, no recovery |
| Actual GUI checks | **19/19** |

The two recovery/adjustment counts can overlap; they are not disjoint failure categories. “Verified” certifies the resulting declared-model motion and task ledger, not primary random-planner success or100% grass area. Detailed requested/actual algorithms and failure reasons are retained.

## Matrix design

864 =12 intra orders ×6 inter orders ×2 grids ×6 connectors. Shape, obstacle density/distribution, seed and BFS/K-means partition are rotated across these cases, **not** additionally fully Cartesian-multiplied. Another120 =5 shapes ×4 densities ×3 distributions ×2 grids; algorithm choices rotate. Algorithm cases use1m² cells and14×10m maps; map cases use.6m² cells and18×14m maps. Real reference body and cutter sizes remain fixed. This coarse matrix checks interfaces/correctness, not satisfactory area coverage.

Final run wall time: 68.07s with4 workers in this container. These simultaneous, budgeted integration runs are **not** an unbiased planner-performance ranking. Random seeds are recorded; timeout behavior can vary with load.

## Dense and larger maps

| Case | Grid | Candidates | Clusters | Declared area | Planner recovery | Simulation hours | Compute s |
|---|---|---:|---:|---:|---:|---:|---:|
| dense_notched_square | square | 2019 | 41 | 98.65% | 0 | 3.14 | 2.91 |
| dense_notched_triangle | triangle | 2049 | 46 | 99.76% | 0 | 7.03 | 3.70 |
| dense_orchard_square | square | 2442 | 48 | 98.30% | 0 | 3.62 | 3.25 |
| dense_orchard_triangle | triangle | 2490 | 78 | 99.89% | 0 | 8.57 | 4.52 |
| large_024ha_square | square | 16303 | 168 | 99.65% | 0 | 19.29 | 21.38 |
| original_square_christofides | square | 5314 | 98 | 99.78% | 0 | 14.95 | 10.93 |
| original_square_exact | square | 5314 | 515 | 99.77% | 0 | 16.61 | 10.49 |
| original_square_ga | square | 5314 | 98 | 99.78% | 0 | 15.08 | 13.69 |
| original_square_mst_euler | square | 5314 | 98 | 99.77% | 0 | 18.15 | 10.78 |
| original_square_nn2opt | square | 5314 | 98 | 99.77% | 1 | 15.25 | 10.37 |
| original_square_serpentine | square | 5314 | 98 | 99.78% | 0 | 7.82 | 7.07 |
| original_triangle_christofides | triangle | 5399 | 116 | 99.91% | 0 | 19.10 | 12.46 |
| original_triangle_exact | triangle | 5399 | 606 | 99.91% | 0 | 19.38 | 11.01 |
| original_triangle_ga | triangle | 5399 | 116 | 99.91% | 0 | 19.07 | 13.64 |
| original_triangle_mst_euler | triangle | 5399 | 116 | 99.90% | 0 | 24.20 | 11.92 |
| original_triangle_nn2opt | triangle | 5399 | 116 | 99.91% | 0 | 19.02 | 11.32 |
| original_triangle_serpentine | triangle | 5399 | 116 | 99.93% | 0 | 18.64 | 8.88 |
| scale_1ha_triangle_coarse | triangle | 9211 | 155 | 67.87% | 0 | 36.59 | 26.47 |

Original field geometry is unchanged; approx829.91m² free task area. Original and dense map cases use.12m² cells. `large_024ha_square` uses60×40m and.12m². **`scale_1ha_triangle_coarse` uses100×100m and1m²; its low area coverage is exposed and it is only a scaling test.** It is not described as completed hectare mowing.

Exact-local original cases created smaller bounded groups. With more than500 clusters the actual inter solver became `nearest_streaming`, recorded in each metrics file; this is not global exact TSP. The timing includes all conservative stops/tool waits; long mission durations highlight the limitations of centre-based graph walks. They are not manufacturer's productivity measurements.

## GUI

Tk8.6 under Xvfb, actual widgets, actual child planner process. Checks: run dispatch, parameter propagation, validation, button states, analytical robot position, first-turn view, pause callback cancellation, slider, full view, speed plot, audit text, screenshot, image export, bad-input rejection, cancellation and child cleanup on close. This is not a screenshot-only mockup. The final options panel scrolls while Run/Cancel remain visible.

## Export performance correction

A late export audit found an events×samples mask scan. The grouped analytical evaluator preserves arbitrary sample order and avoids rescanning all samples for each event. On2103 events and12000 samples: 2.590s before, 0.244s after; maximum element difference **0.0**. This single-machine diagnostic measures sampling, not path quality or an all-system speedup. A regression test checks shuffled/scalar equivalence, empty and nonfinite inputs.

## Installation / archive reproduction

Local internet package download was unavailable. A new Python virtual-environment entry uses an **explicit .pth link to the container's already installed scientific packages**, then builds this package editable with `--no-deps --no-build-isolation`. This checks package metadata/entry points and relocated source; **it is not a fresh dependency installation**. Initial failed build without inherited setuptools is retained in the installation logs rather than erased. Ruff is configured for CI but was not installed locally; local static checking used compileall.

Relocation and CLI results: **12/12 passed**. See `qa/installation/relocation_checks.json`.

## Deliberate boundary tests and original corrections

Tests include a collision between safe-looking rotation endpoints; a narrow passage accepted by the full body translation but rejected by an all-heading circle; disconnected field with explicit partial ledger; tiny/invalid inputs; goal/start preservation; full MST traversal; GA/exact permutations; asymmetric2-opt; A* direction-dependent state; reusable PRM noncontamination; primary failure vs recovery; negative/NaN bounds; changed time origin; deleted/relocated mandatory task; illegal side motion; forged velocity profile; modified area metric; refused failed/cancelled snapshot. The initial K-means integer-buffer regression was fixed and retested.

Raw source histories are not retroactively green: `pytest_first.txt` retains the initial failure; final results are in `pytest_release.txt` and the final matrix directory. Remote CI is supplied but has not run here.
