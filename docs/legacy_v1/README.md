# CCPP Mowing Planner

**Hierarchical square/triangle coverage planning with a dimensioned differential-drive mowing model.**

[中文说明](README_中文.md) · [Algorithms](docs/ALGORITHMS.md) · [Robot model](docs/ROBOT_MODEL.md) · [Test report](docs/VALIDATION_REPORT.md) · [Portfolio guide](docs/PORTFOLIO.md)

![Actual desktop application](docs/assets/gui.png)

This is a **pure-Python, offline research project**, developed from the CCPP agricultural grid-coverage project. It restores clustering, cluster-level ordering, intra-cluster coverage and route stitching, then adds explicit body collision checks, finite-rate stop–turn–go motion, constrained time-profile optimization and independently verified outputs.

The Agria 9500-70 supplies **dimensional reference data**, not a hardware interface or an autonomous-vehicle claim. Every result separates original-area coverage, declared-task coverage, visited centres, primary planner outcomes, recovery calls and manual residuals. A green verification result does **not** mean 100% physical coverage.

## Run in five minutes

Use a new folder; do not merge older CCPP releases into this one. Python 3.13 is the locally tested series.

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe app.py
```

Activation is optional. In VS Code select `.venv/Scripts/python.exe`; use **Run Python File**, not **Run Test**, to start `app.py` or `main.py`. Windows batch launchers are included. A native Tcl/Tk installation is required for the GUI; the official Windows Python installer normally provides it. Headless planning does not require the GUI.

```bash
python main.py                       # seeded default square-grid example
python main.py --grid triangle --intra mst_euler --inter ga --planner prm --out outputs/triangle_mst
python main.py --map maps/original_field.geojson --config configs/original_field.json --out outputs/original
python validate_saved.py outputs/original
```

`main.py` without arguments writes/overwrites `outputs/latest`; explicit CLI outputs refuse accidental overwrite. For a package entry point, run `python -m pip install -e .` and then `ccpp-mow gui` or `ccpp-mow plan ...`. This release is intended to be run from the repository or an editable installation; a standalone wheel with bundled maps is not supplied.

## What is implemented

| Layer | Methods |
|---|---|
| Map | Original CCPP metric GeoJSON; seeded rectangle, L-shape, trapezoid, notched and orchard fields; separate disconnected diagnostic |
| Grid | Square four-neighbour and equilateral-triangle three-neighbour; equal-area, equal-side and tool-cover comparisons |
| Clustering | Connected BFS, K-means followed by connected-component repair, or bounded unpartitioned mode |
| Intra-cluster | Row/column sweep, Greedy, nearest neighbour, NN+2-opt, DFS, MST–Euler, double-tree, GA, simulated annealing, Christofides, bounded Held–Karp |
| Inter-cluster | Nearest, NN+2-opt, GA, SA, Christofides, bounded Held–Karp; explicit proxy/graph cost selection |
| Connections | A*, Dijkstra, RRT, RRT-Connect, RRT*, reusable PRM |
| Motion | Forward straight translation, stop, finite-rate pivot, tool-state accounting, revisits and return to start |
| Timing | Numerical SLSQP optimization of each smooth velocity profile; independently checked conservative recovery |
| Delivery | Tk GUI, analytical animation, PNGs, event JSON, trajectory CSV, GeoJSON residuals, audit logs, independent result validator |

There is no ROS, Gazebo, CUDA, web server, METIS/LKH installation or video-renderer requirement. Experimental strict-forward/BCD branches from earlier prototypes are **not** exposed as supported options in this release.

## What a result means

The planner preserves the original field. `manual_setback_m` creates an **up-front task contract**, not a safety-standard claim. Collision-infeasible cells and unreachable tasks are logged; they are not counted as completed. Finished grass remains traversable. Intermediate detours and repeat visits are labelled `TRANSIT`, while area credit is computed only from `WORK` and `SERVICE` tool footprints.

By default a failed randomized connector may use a **reported A* recovery**. For primary-algorithm experiments use `configs/strict_benchmark.json` or `--strict`, inspect `queries.json`, and report failures. Straight visibility checks are a shared fast path, explicitly labelled, not claimed to be RRT/PRM exploration.

Small exact subproblems are truly solved by dynamic programming. An entire large field is **not** an exact TSP. Resource-limited ordering substitutions are recorded with requested/actual method and reason. See [limits](docs/LIMITATIONS.md).

## Evidence, not badges

The release includes raw local test reports and reproducible scripts. The combined matrix has **984 cases**, including **864 algorithm combinations** and **120 map/density/distribution cases**. Fine-grid integration includes both grids on the unchanged original map, a 0.24 ha fine-grid field, and a **coarse**, not full-coverage, 1 ha scaling check. Primary failures/recoveries are disclosed in the report.

A Windows/Linux CI workflow is provided, but its remote runs are **not claimed to have occurred**. Locally tested platform: Linux / Python 3.13.5 / Tk 8.6. The GUI is exercised by actual button invocation under Xvfb, including cancellation and child-process cleanup. A local internet restriction prevented a fresh online dependency-download test; the extracted-package reproduction uses the installed scientific packages, clearly recorded.

## Inspect and reproduce

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q
python tools/release_matrix.py
python tools/dense_integration.py
```

Dense experiments are more expensive than unit tests. With default `0.12 m²` cells, the original map has about 5,300 tasks. A 1 ha field at that resolution exceeds this release's guarded desktop budget; coarsen explicitly, never silently.

![Finite-rate turn excerpt](docs/assets/turn_demo.gif)

The GIF is evaluated from the same saved event functions as the GUI and CSV, not an artist's rendering or physical recording.

Before modifying the delivered files, `python tools/check_manifest.py` checks their SHA-256 manifest. Source changes are expected to change these hashes; the manifest is for delivery integrity, not a permanent development lock.

## Source layout

```text
ccpp_mowing/
  geometry.py, grids.py, maps.py    # metric maps, body geometry, task tessellation
  order_core.py, ordering.py       # connected clusters, TSP/Euler methods, gates
  search.py, navigation.py         # A*/RRT/PRM and logged recovery
  motion.py                       # finite-rate events and numerical time optimization
  validation.py, pipeline.py       # independent certificates, area ledger and exports
  gui.py, visualization.py         # desktop controls and analytical playback
configs/ maps/ tests/ tools/ docs/ examples/
```

Start reading with [architecture](docs/ARCHITECTURE.md), then compare code and formulas in [algorithms](docs/ALGORITHMS.md). The project demonstrates modeling, search, optimization, testing and experiment discipline. It does **not** claim field deployment, certified safety, calibrated skid-steer dynamics, or commercial productivity.

## Attribution and publication

Origin: [caihao1B/CCPP](https://github.com/caihao1B/CCPP). The original input map is retained byte-for-byte. See [NOTICE](NOTICE.md) and [provenance](docs/PROVENANCE.md). No repository-wide open-source license was detected in the original repository, so this delivery does not silently assign one on the owner's behalf. Select a suitable license before presenting reuse permissions. No manufacturer pictures, logos or font files are bundled.
