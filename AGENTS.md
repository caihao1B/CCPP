# v2.1 updates (take precedence over historical notes)

Read docs/V2_1_测试与展示报告.md and tests/test_forward_loops.py. Preserve task order. `loops_only` must never emit TURN or reverse; fail visibly if no certified forward loop exists. Keep finite steering events for the bounded-curvature model. Numerical optimization success never replaces independent continuous feasibility checks. Fixed G2 fallback must stay explicit in records.

Default GUI preset08 uses a custom compact vehicle JSON. Do not assume every displayed vehicle has default Agria dimensions. New heading-loop grouping must be preserved by geometry, events, exporter and web replay. All 10 website cases are precomputed, not browser-side planning. Preserve exact timing/pose equivalence and separate discrete service from area coverage.

Current test count214 is a recorded version result, not a magic expected constant. Rerun tests after edits; update manifests and evidence only after actually rerunning. Website exporter requiresFFmpeg only when generating MP4, not to view existing files. Never fabricate another project's evidence or publish without owner authorization.

# CCPP Mowing Planner v2 — development contract

Respond to the owner in Chinese. This is a standalone offline Python GUI2, not ROS/hardware control.
Read START_HERE_中文.md, README_中文.md, docs/TRAJECTORY_DESIGN_中文.md and the current tests before editing.

## Structure and commands
- GUI: app.py -> ccpp_mowing/gui.py. GUI planning/export uses owned subprocesses.
- Pipeline: ccpp_mowing/pipeline.py. Existing ordering/navigation stay independent of the new trajectory layer.
- Curves: curves.py, dubins.py, curve_planner.py, curve_motion.py, curve_validation.py.
- Replay: motion.evaluate_events -> curve_motion.evaluate_curve_event; GIF uses the SAME evaluator.
- Run (Windows): .venv\Scripts\python.exe app.py
- Test: .venv\Scripts\python.exe -m pytest -q (POSIX: .venv/bin/python -m pytest -q).
- Rebuild: python tools/build_curve_demos.py --ids 03 04 --out outputs/rebuilt
- Revalidate: python -m ccpp_mowing validate examples/demo_03
- GUI smoke (Linux display/Xvfb): python tools/gui_curve_smoke.py

## Non-negotiable invariants
1. Preserve original coverage ordering/graph/search unless a task explicitly requests changes there.
2. Do not move/drop required centres to make smoothing pass. Distinguish centre_exact, cutter_footprint and area coverage.
3. bounded_curvature: no TURN, no negative forward velocity, no secretly smaller radius or unsafe connectors.
4. Differential pivot fallback must be explicit and counted. Never present it as a curved turn.
5. Geometry, time law, GUI/GIF/CSV, saved JSON and independent validators must agree.
6. Collision checks cover the whole moving body and curve, not just endpoints or sparse time samples.
7. Optimizer success is not a safety certificate. Recompute continuous bounds and saved results independently.
8. Preserve v1 saved-schema loading and the 14-column legacy trajectory.csv contract.
9. New GUIs keep cancel, failed statuses and own-child process cleanup; no global process killing.
10. Do not modify golden examples/QA artifacts without explicit regeneration and clearly labelled new evidence.
11. No secrets, credentials, new remote calls, hardware actuation or arbitrary destructive shell commands.

## Current model boundaries
All curve primitives are rest-to-rest. Bounded model uses finite stationary STEER at curvature jumps.
No Pure Pursuit/MPC closed loop, Reeds–Shepp, Hybrid A*, real Ackermann wheel model or slip simulation.
G2 is for the quintic/line patch, not a blanket claim about mixed Dubins connections.
Fixed-task local conversion can fail; do not hide failure by reducing obligations or radius.

## Work style
First state target files and acceptance criteria, then make a small testable change. Add a regression test reproducing each bug.
Run existing tests and revalidate at least demo03 and demo04. Report exact commands, executed tests, limitations and files changed.
Do not claim Windows/robot/GPU validation based on Linux or analytical replay.


## v2.0.1 audit constraints
- Read docs/PORTFOLIO_AUDIT_中文.md before changes. Never weaken safety/coverage checks to make a plot look smoother.
- Keep task visit order distinct from A* expansion and returned navigation path.
- Do not label verified_visibility or cache hits as fresh A* executions.
- Preserve the 194-test regression suite and compare replay math after modifying curves.
- Web material is reference replay, not feedback control or verified hardware.
- Recompute MANIFEST after intentional release changes; preserve source attribution.
