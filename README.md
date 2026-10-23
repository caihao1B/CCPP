# v2.1.0 更新入口

请先读 [START_HERE_中文.md](START_HERE_中文.md)。新展示结果在 `comparison_examples/`，推荐08—12预设；原01—07和下面的旧说明保留为兼容基线。新策略、复验、网格/算法对照详见 [本版报告](docs/V2_1_测试与展示报告.md)。

---

> **v2.0.1 审计与作品集补丁**：本轮发现及修复、194项测试与八组展示结果见 [审计报告](docs/PORTFOLIO_AUDIT_中文.md)，演示入口见 [作品集指南](docs/PORTFOLIO_中文.md)。以下原 v2.0.0 功能说明继续适用；历史测试数字不代表本轮统计。

# CCPP Mowing Planner 2.0.1

A standalone upgrade of GUI2 from the user's uploaded `CCPP_VSCode_Interview_Kit.zip`.
Start with **[Chinese quick start](START_HERE_中文.md)** or [complete Chinese README](README_中文.md).

The original coverage graph, partitioning, MST–Euler/TSP/greedy ordering and navigation algorithms are preserved. The new layer constructs and independently validates analytical curved motion, rather than merely rounding the plot.

## Run

Windows: `setup_windows.bat`, then `run_gui.bat`. Python 3.12/3.13 with Tk is recommended; Python >=3.11 is required. A fresh local `.venv` is created. Dependencies are not bundled.

```sh
python -m ccpp_mowing plan --config configs/demo_03.json --map maps/curves_open.geojson --out outputs/bezier
python -m ccpp_mowing validate outputs/bezier
python -m ccpp_mowing render outputs/bezier --out outputs/bezier/first_curve.gif
python app.py
```

In the GUI: **Motion / curves → preset 03 → Apply preset → Generate → First curve / turn → Play**. Preset04 is a forward-only Dubins vehicle with R_min=0.60m. Preset05 demonstrates G2 quintic Bezier motion without pivots, on a deliberately coarse grid; it does NOT provide full area coverage.

## Options

Models: `differential`, `bounded_curvature`. Methods: `stop_turn_go`, `circular_fillet`, `bezier_g2`, `bezier_slsqp`, `dubins`. Independent timing choices: `slsqp`, `conservative`.

The new time model bounds forward/turning speed, longitudinal acceleration/jerk, yaw acceleration/jerk, equivalent wheel envelopes, lateral acceleration and equivalent steering rate. Curve collision certification uses continuous conservative enclosures, not only free endpoints. All original target obligations are rechecked under the declared cutter-footprint or exact-centre rule. Residual areas are explicit.

The animation/GIF and exported reference use the SAME analytical geometry and time law. No feedback controller, localization, ROS 2, Gazebo, real hardware, GPU, API key or network service is needed at runtime.

## Evidence and limits

See `docs/VALIDATION_REPORT.md` and `docs/qa`. Current execution evidence is Linux Python3.13.5 including a real Tk GUI under Xvfb, not a claim of Windows hardware testing. `requirements-tested.txt` records this environment and is not a cross-platform lock.

Every primitive is rest-to-rest. At curvature jumps, bounded-curvature motion uses finite-duration stationary steering, never chassis pivots. Differential fallback pivots are explicit and configurable. The converter is local and may reject dense sharp routes; it is NOT a complete SE(2) planner. G2 applies to Bezier/straight patches, not automatically every mixed Dubins connection. Neither the local shape nor primitive-time optimizer proves global optimality. Workload completion and area coverage have different denominators.

`AGENTS.md` defines follow-on development invariants. Original source provenance and third-party rights remain in `NOTICE.md`; no manufacturer endorsement or physical calibration is claimed.
