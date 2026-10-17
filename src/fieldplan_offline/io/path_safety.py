"""Point-path geometry checks; no vehicle curvature or footprint guarantee."""
from __future__ import annotations

import math
from shapely.geometry import LineString, Point
from shapely.prepared import prep

# Only for floating-point rotation/intersection error; not a safety margin.
GEOMETRY_TOL = 1e-8


class NoSafePathError(RuntimeError):
    """No verified connection was found at the chosen grid resolution."""


def covers_segment(free, a, b, *, tol=GEOMETRY_TOL):
    geometry = Point(a) if a == b else LineString([a, b])
    return free.buffer(min(max(tol, 0.0), GEOMETRY_TOL)).covers(geometry)


def validate_path(free, points, *, context='path'):
    """Check every full edge, including zero-length edges and singleton paths."""
    if not points or any(not all(math.isfinite(v) for v in p) for p in points):
        raise NoSafePathError(f'{context}: empty or non-finite path')
    allowed = prep(free.buffer(GEOMETRY_TOL))
    if any(not allowed.covers(Point(p)) for p in points):
        raise NoSafePathError(f'{context}: endpoint outside free space')
    for i, (a, b) in enumerate(zip(points, points[1:])):
        if not allowed.covers(Point(a) if a == b else LineString([a, b])):
            raise NoSafePathError(f'{context}: unsafe edge {i}: {a} -> {b}')
