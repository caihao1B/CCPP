# Supported release boundary

This release deliberately removes unfinished strict-forward Dubins/Hybrid-A*, continuous BCD, METIS/LKH hooks and unsupported hardware-control branches from selectable production options. They are not silently redirected to another named method. The supported motion is a dimensioned ideal differential stop–turn–go model.

## Guarantees vs evidence

Full primitive sweep and analytical profile checks establish feasibility **in the declared geometric/model assumptions**, not physical safety. Passing 984 integration cases does not prove every map/seed/parameter. Independent verification recomputes results but uses the same shape/model libraries, so common-mode numerical/model errors remain possible.

Finite-budget random searches can fail despite an existing path. Coarse navigation discretization and all-heading safe pivots can reject feasible continuous poses. Replanning can use already finished ground; it does not prove graph completeness. Start auto-selection chooses a component and explicitly lists the others; an explicit start is never moved. No safe return route means failure, not a successful unfinished return.

## Budget policy

- Default max cells30,000 (including grid construction budget; actual admitted count may be lower).
- Local metric nodes≤180; exact Held–Karp default≤11, maximum14.
- Cluster cost matrix default≤500, maximum1000. Normal mode can stream a nearest representative order above the cap and logs the substitution; strict mode fails.
- No global exact TSP or global jointly optimal gate/order/vehicle-time claim.
- `partition=none` can be broken into bounded connected groups under the budget, with a reason in the audit.
- A* recovery is visible. Verification success and primary success must be reported separately.

## Scale and physical coverage

The unchanged original map has about830m² free task area, not hectares. Fine-grid checks use .12m² cells and about5,300 centres. A .24ha synthetic field with16,303 admitted square centres was run. The1ha case uses1m² triangle cells and shows only about68% declared-area coverage: it tests scaling, **not a satisfactory mowing plan**. Fine 1ha at .12m² exceeds the tested desktop budget.

The0.12m² grid is a sensible cutter-based comparison, but retaining lattice centres plus conservative body pivots can still leave declared-area residuals. The user may accept them; the software never counts them as cut. Manual maps are the deliverable, not an altered denominator alone.

Mission durations can be hours or tens of hours for centre-heavy graph walks. Every stop, tool wait and turn is included. They are model predictions for a deliberately restrictive baseline, not Agria field productivity. Timing optimization only improves a fixed family relative to its own feasible baseline, not the spatial route or real machine.

## Platform and distribution

Locally tested Linux/Python3.13.5/Tk8.6. Windows CI/launchers provided, not remotely verified here. Native Tk is required for GUI. Working-folder and extraction-with-spaces tests are included. Native Python dependencies need wheels compatible with the interpreter; `requirements.txt` carries broad supported ranges, `requirements-tested.txt` records this Python3.13 runtime, not a universal OS lock.

Network access for pip was unavailable in the test container. The fresh-folder/venv smoke test uses preinstalled scientific dependencies and does not certify an internet download/install. CI is intended to test fresh installs after upload; no pre-generated green CI badge is claimed.
