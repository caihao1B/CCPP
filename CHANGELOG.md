# 2.1.0

- Added explicit legacy_pivot / loop_then_pivot / loops_only behavior and fail-closed validation.
- Full forward same-point heading loops before optional pivot fallback.
- Certified fixed-G2 fallback after optimized shape rejection; records remain auditable.
- Loop IDs, complete loop playback and no automatic end/start reset on website.
- GUI custom research vehicle JSON and new reproducible presets08—12.
- Six grid/ordering comparisons, three same-order smoothing/timing variants, two radius variants (10 unique cases).
- Original academic-style personal site and integrated ordered-route→curve narrative.
- Detailed Windows preview/PowerShell/GitHub guide and loopback-only one-click preview.
- 214 regression tests; 45 GUI checks; 118 browser checks; 1943 Python/JS pose comparisons.

# 2.0.1 — 2026-10-22

- Strengthen saved-result pivot policy, tool-transition waits, displayed metrics, strict task accounting, and legacy snapshot revalidation.
- Add CLI custom robot JSON and accurate research-vehicle/GUI dimension labeling.
- Add 24 regression checks, reproducible portfolio cases, independent audits, and evidence-backed media export.
- Preserve existing coverage/search/curve implementations and original demonstration inputs.

# Changelog

## 2.0.0 — constrained curve reference replay

- Preserve original coverage task generation, clustering, ordering and navigation algorithms.
- Add analytical LINE/ARC/BEZIER5 curves, tangent fillets, fixed and SLSQP quintic Bezier blends, and six Dubins families.
- Add differential and forward-only bounded-curvature models; forbid forward-mode pivot fallback.
- Add independent radius, body clearance, derivative, actuator, lateral acceleration and target-service validation.
- Add geometry and time optimization selections, explicit conservative fallback records, and finite stationary STEER events.
- Animate actual analytical reference poses; add first-curve/follow zoom, curvature/constraint page and GIF export.
- Export raw_legs, curve_path, curve_diagnostics and 20-column trajectory_reference; preserve legacy 14-column CSV.
- Support saved v1/v2 schemas; independently verify v2 curved snapshots before display.
- Provide seven reproducible presets, real saved outputs, GUI checks, new regression tests and AGENTS.md.
- New Windows scripts use a fresh local .venv and explicit UTF-8; VS Code debugging configurations included.

Important differences: new curve mode defaults to cutter-footprint target service, reports residual area separately, and uses rest-to-rest primitives. G2 is local, not a guarantee for mixed Dubins geometry. No hardware or feedback tracking is added. The CLI's default legacy mode is preserved.

Original changelog and documentation: docs/legacy_v1/.
