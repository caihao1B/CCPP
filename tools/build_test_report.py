"""Summarize recorded tests; do not create test outcomes from expectations."""
from pathlib import Path
import json,platform,sys,hashlib
ROOT=Path(__file__).resolve().parents[1]

def read(p):return json.loads((ROOT/p).read_text(encoding='utf-8'))
def main():
    matrix=read('docs/qa/release_matrix_final/summary.json')
    dense=read('docs/qa/dense_integration/summary.json')
    gui=read('docs/qa/gui_tests.json')['tests']
    force=read('docs/qa/forced_detours.json')
    bench=read('docs/qa/sampling_optimization.json')
    unit=(ROOT/'docs/qa/pytest_release.txt').read_text()
    import re
    match=re.search(r'(\d+) passed',unit)
    if not match:raise RuntimeError('No successful pytest result in log')
    install_path=ROOT/'docs/qa/installation/relocation_checks.json'
    install=json.loads(install_path.read_text()) if install_path.exists() else None
    summary={'release':'1.0.0','unit_tests_passed':int(match.group(1)), 'algorithm_map_matrix':matrix,
        'dense_cases':len(dense),'dense_verified':sum(r['verified'] for r in dense),
        'gui_checks':len(gui),'gui_passed':sum(r['passed'] for r in gui),
        'primary_only_detours':force,'sampling_diagnostic':bench,'relocation':install,
        'environment':{'python':sys.version,'platform':platform.platform()},
        'not_tested':['Windows native runtime','remote GitHub Actions','fresh online dependency installation',
        'hardware tracking, slip, blade safety, dynamic obstacles','all possible map/seed/parameter combinations']}
    (ROOT/'docs/qa/release_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    rows=['| Case | Grid | Candidates | Clusters | Declared area | Planner recovery | Simulation hours | Compute s |',
          '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in sorted(dense,key=lambda x:x['id']):
        if r['verified']:
            m=r['metrics'];rows.append(f"| {r['id']} | {r['grid']} | {m['candidate_centres']} | {m['cluster_count']} | {m['declared_area_coverage_ratio']:.2%} | {m['connector_fallbacks']} | {m['trajectory_duration_s']/3600:.2f} | {m['compute_seconds']:.2f} |")
        else:rows.append(f"| {r['id']} | FAILED | — | — | — | — | — | — |")
    text=f'''# Release 1.0.0 — actual qualification record

This report summarizes saved local runs, not a mathematical proof or a product safety certificate. Raw JSON/CSV/logs are committed under `docs/qa/`. Tests apply to the release source and declared model. No Windows or robot tests are implied.

## Results

| Test layer | Recorded result |
|---|---:|
| Unit/regression suite | **{summary['unit_tests_passed']} passed** |
| Algorithm/map matrix | **{matrix['verified']}/{matrix['cases']} independently verified** |
| Candidate-centre completion in matrix | **{matrix['full_centres']}/{matrix['cases']} cases** |
| Matrix cases with reported A* recovery | **{matrix['with_connector_fallback']}** |
| Matrix cases with an order/partition resource adjustment | **{matrix['with_ordering_substitution']}** |
| Dense/scaling integration | **{summary['dense_verified']}/{len(dense)} verified** |
| Blocked-direct-route primary-only tests | **{force['passed']}/{force['cases']}**, no recovery |
| Actual GUI checks | **{summary['gui_passed']}/{len(gui)}** |

The two recovery/adjustment counts can overlap; they are not disjoint failure categories. “Verified” certifies the resulting declared-model motion and task ledger, not primary random-planner success or100% grass area. Detailed requested/actual algorithms and failure reasons are retained.

## Matrix design

864 =12 intra orders ×6 inter orders ×2 grids ×6 connectors. Shape, obstacle density/distribution, seed and BFS/K-means partition are rotated across these cases, **not** additionally fully Cartesian-multiplied. Another120 =5 shapes ×4 densities ×3 distributions ×2 grids; algorithm choices rotate. Algorithm cases use1m² cells and14×10m maps; map cases use.6m² cells and18×14m maps. Real reference body and cutter sizes remain fixed. This coarse matrix checks interfaces/correctness, not satisfactory area coverage.

Final run wall time: {matrix['elapsed_wall_seconds']:.2f}s with{matrix['workers']} workers in this container. These simultaneous, budgeted integration runs are **not** an unbiased planner-performance ranking. Random seeds are recorded; timeout behavior can vary with load.

## Dense and larger maps

{chr(10).join(rows)}

Original field geometry is unchanged; approx829.91m² free task area. Original and dense map cases use.12m² cells. `large_024ha_square` uses60×40m and.12m². **`scale_1ha_triangle_coarse` uses100×100m and1m²; its low area coverage is exposed and it is only a scaling test.** It is not described as completed hectare mowing.

Exact-local original cases created smaller bounded groups. With more than500 clusters the actual inter solver became `nearest_streaming`, recorded in each metrics file; this is not global exact TSP. The timing includes all conservative stops/tool waits; long mission durations highlight the limitations of centre-based graph walks. They are not manufacturer's productivity measurements.

## GUI

Tk8.6 under Xvfb, actual widgets, actual child planner process. Checks: run dispatch, parameter propagation, validation, button states, analytical robot position, first-turn view, pause callback cancellation, slider, full view, speed plot, audit text, screenshot, image export, bad-input rejection, cancellation and child cleanup on close. This is not a screenshot-only mockup. The final options panel scrolls while Run/Cancel remain visible.

## Export performance correction

A late export audit found an events×samples mask scan. The grouped analytical evaluator preserves arbitrary sample order and avoids rescanning all samples for each event. On{bench['events']} events and{bench['samples']} samples: {bench['before_seconds']:.3f}s before, {bench['after_seconds']:.3f}s after; maximum element difference **{bench['max_absolute_difference']}**. This single-machine diagnostic measures sampling, not path quality or an all-system speedup. A regression test checks shuffled/scalar equivalence, empty and nonfinite inputs.

## Installation / archive reproduction

Local internet package download was unavailable. A new Python virtual-environment entry uses an **explicit .pth link to the container's already installed scientific packages**, then builds this package editable with `--no-deps --no-build-isolation`. This checks package metadata/entry points and relocated source; **it is not a fresh dependency installation**. Initial failed build without inherited setuptools is retained in the installation logs rather than erased. Ruff is configured for CI but was not installed locally; local static checking used compileall.

{('Relocation and CLI results: **'+str(install.get('passed'))+'/'+str(install.get('checks'))+' passed**. See `qa/installation/relocation_checks.json`.') if install else 'Archive relocation verification is recorded separately when the release archive is constructed.'}

## Deliberate boundary tests and original corrections

Tests include a collision between safe-looking rotation endpoints; a narrow passage accepted by the full body translation but rejected by an all-heading circle; disconnected field with explicit partial ledger; tiny/invalid inputs; goal/start preservation; full MST traversal; GA/exact permutations; asymmetric2-opt; A* direction-dependent state; reusable PRM noncontamination; primary failure vs recovery; negative/NaN bounds; changed time origin; deleted/relocated mandatory task; illegal side motion; forged velocity profile; modified area metric; refused failed/cancelled snapshot. The initial K-means integer-buffer regression was fixed and retested.

Raw source histories are not retroactively green: `pytest_first.txt` retains the initial failure; final results are in `pytest_release.txt` and the final matrix directory. Remote CI is supplied but has not run here.
'''
    (ROOT/'docs/VALIDATION_REPORT.md').write_text(text,encoding='utf-8')
    print('Report generated from logs.')
if __name__=='__main__':main()
