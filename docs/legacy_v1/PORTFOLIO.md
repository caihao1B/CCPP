# GitHub presentation, reproducibility and interview guide

## A defensible one-paragraph project description

Developed a pure-Python hierarchical agricultural coverage-planning workbench with square and equilateral-triangle tessellations, connected clustering, multiple local/global order solvers and A*/RRT/PRM connection planners. Integrated an Agria9500-70 dimensional reference model, finite-rate differential stop–turn–go execution and constrained numerical time-profile optimization. Added full body-sweep and analytical motion validation, explicit primary/recovery audit records, manual-area accounting, a desktop experiment GUI and reproducible tests.

Do not turn this into claims of real robot deployment, LiDAR perception, ROS navigation, force control, learned policies, globally optimal paths or certified safety. No measured hardware controller has been implemented.

## What to show in a five-minute demo

1. Open GUI, run default fine square grid, point to original vs declared vs centre metrics.
2. Change only grid to triangle; same physical robot and nominal cell area. Explain why graph degree, node positions, path length and stop count change.
3. Select MST–Euler or GA; inspect `ordering.json` rather than infer the algorithm from a line drawing.
4. Select PRM or RRT; inspect `queries.json`. Most adjacent-centre queries are trivially visible. Show the forced-detour test for actual exploration and disclose recovery.
5. First turn: real-size body, finite yaw profile, opposite drive-side speeds, no instantaneous rotation. Explain manual residuals and model limitations.

## Suggested README assets

`docs/assets/gui.png`, `turn_demo.gif`, checked-in square/triangle examples, and actual raw validation report. A small GIF is adequate; do not commit the environment or multi-GB experiment videos. Keep `configs/`, `tests/`, `tools/`, `docs/qa/` to make claims inspectable. Raw result datasets can move to a GitHub Release as they grow.

## Safe repository workflow

Use a new branch or new repository. Back up the existing CCPP repository before replacing folders; do not delete its `.git` history. Compare the new top-level files, preserve origin notes, and run tests. `git status` should not list `.venv`, secrets or huge run folders. This delivery does not push commits or set repository permissions.

The original repository had no detected LICENSE. Decide on the permitted reuse policy as the author; do not describe an unlicensed repository as MIT merely because its code is visible. No manufacturer image license is needed here because no manufacturer images are distributed.

## Interview questions worth preparing

**Why is A* optimal but not the whole coverage task?** It optimizes a particular graph connection. Visit order, gates, geometry and mission time are separate choices.

**Why not use RRT to randomly cover every cell?** A fixed service obligation list is independent of the point-to-point connector. Sampling alone does not establish complete coverage.

**Why may PRM call A*?** PRM builds reusable topology; A* can query that graph. In the audit, an A* recovery on a different graph is a different mechanism and is labelled separately.

**Why can MST revisit nodes?** A coverage walk need not be Hamiltonian. Odd-vertex augmentation produces a continuous traversal; early endpoint truncation is incorrect. Repeated transit does not add unique area.

**Are exact and Christofides guarantees global?** No: exact is bounded local DP. Open endpoint adaptations and cluster proxy costs do not inherit an unqualified Christofides bound for the full robot task.

**Why count both nodes and area?** Cutter sweep between and around nodes differs from node incidence, especially with clipped cells, obstacles and coarse resolution.

**Why not just shrink the map or robot?** It changes the physical problem. Density/spacing and robot sizes have separate SI units. The original map is preserved and scale experiments are named explicitly.

**What is actually optimized numerically?** The peak speed and ramp duration of each smooth straight/pivot profile. Variables, objective, constraints and independently checked fallback are in `motion.py`; it is not global spatial trajectory optimization.

**Why allow zero-turn?** It is part of the published dimensional reference machine's mobility. Finite yaw/track limits and all-angle swept-body geometry still matter.

**What is a realistic weakness?** Centre-heavy stop–turn–go can have poor productivity and soil effects. The model ignores slip and blade dynamics. A production next step would optimize continuous service lanes and calibrated vehicle motion, without renaming this release as that solution.

**How do you demonstrate code correctness?** Deliberate counterexamples (safe endpoints but colliding mid-turn, wrong time origin, removed task, invalid wheel rate), property/parameter tests, algorithm/map matrices, saved-result reconstruction and GUI cancellation tests. None proves the absence of all bugs.

**Why not all-pairs shortest paths across1ha fine cells?** N² memory would explode. Bounded connected clusters, sparse graphs, shared PRM, bounded/cached costs and an explicit streaming order limit the resource demand.

## Learning order

`main.py → cli → models → geometry/maps → grids → order_core/ordering → navigation/search → motion → validation → GUI`.
At each boundary ask: input types/units? output obligations? what can fail? what is persisted? what independently checks the output?
