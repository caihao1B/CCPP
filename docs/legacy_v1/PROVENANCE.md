# Provenance

Original repository: https://github.com/caihao1B/CCPP

The original map is stored unchanged as `maps/original_field.geojson`. Hashes are in `provenance.json`. The original grid/clustering/TSP/MST ideas are restructured into `grids`, `order_core`, `ordering`, `navigation` and `pipeline`; they are not claimed as newly invented algorithms.

This release intentionally replaces old unconstrained XY smoothing and unverified forward-only branches with a separately validated differential motion model. `main.py`/GUI command interfaces are new; old configurations are rejected instead of guessed. User Git history and remote repository were not changed.

Synthetic examples record shape, physical extent, density, requested/placed obstacles, distribution and seed. They are not real field surveys. No external map tiles or manufacturer photographs are included.
