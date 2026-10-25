#!/usr/bin/env python3
"""Measure the physical interior of a container from its planning collision mesh.

The destination bin a cell plans against is a triangle mesh in the generated MoveIt manifest
(config/moveit_collision_objects.yaml). Its floor height, free interior and rim are measured here by
casting rays from a point inside the authored drop region, so the physics world and the place seating
use the same geometry MoveIt certifies against, for any container mesh, pose or scale.

Conservative by construction: the interior half-extent on each side is the nearest wall hit over the
height that side's wall rises (draft-angled walls give the narrowest section), each side keeps its own
measured wall top (a stacking bin's open front is lower), and the floor must be flat under the drop
footprint. A missing floor, a side with no wall or a covered cavity fails closed.
Coordinates are the container's own frame (manifest object frame), metres.
"""
import math

import numpy as np

EPS = 1e-9


class ContainerGeometryError(ValueError):
    """The mesh does not bound a measurable container interior at the requested point (fail closed)."""


def _triangles(vertices, triangles):
    v = np.asarray(vertices, dtype=float)
    t = np.asarray(triangles, dtype=int)
    if v.ndim != 2 or v.shape[1] != 3 or t.ndim != 2 or t.shape[1] != 3 or not len(t):
        raise ContainerGeometryError('container mesh needs vertices [n,3] and triangles [m,3]')
    if not np.isfinite(v).all() or t.min() < 0 or t.max() >= len(v):
        raise ContainerGeometryError('container mesh has non-finite vertices or invalid triangle indices')
    return v[t]


def ray_hits(tris, origin, direction):
    """Distances along ``direction`` (unit) at which the ray from ``origin`` hits the triangles."""
    o, d = np.asarray(origin, float), np.asarray(direction, float)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = b - a, c - a
    p = np.cross(d, e2)
    det = (e1 * p).sum(1)
    ok = np.abs(det) > EPS
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    s = o - a
    u = (s * p).sum(1) * inv
    q = np.cross(s, e1)
    v = (q * d).sum(1) * inv
    dist = (q * e2).sum(1) * inv
    hit = ok & (u >= -EPS) & (v >= -EPS) & (u + v <= 1 + EPS) & (dist > EPS)
    return np.sort(dist[hit])


def measure_container(vertices, triangles, center_xy, start_z, footprint_xy, levels=12, flat_tolerance_m=0.001):
    """Floor top, interior box and rim of the container around ``center_xy``.

    ``start_z`` is a height inside the cavity (e.g. the drop region centre) and ``footprint_xy`` the
    authored drop footprint whose corners must all stand on the same flat floor.
    """
    tris = _triangles(vertices, triangles)
    cx, cy = (float(v) for v in center_xy)
    def floor_below(x, y):
        hits = ray_hits(tris, [x, y, start_z], [0.0, 0.0, -1.0])
        if not len(hits):
            raise ContainerGeometryError(f'no container floor below ({x:.4f}, {y:.4f})')
        return start_z - hits[0]
    floor = floor_below(cx, cy)
    fx, fy = (float(v) / 2 for v in footprint_xy)
    corners = [floor_below(cx + sx * fx, cy + sy * fy) for sx in (-1, 1) for sy in (-1, 1)]
    if max(abs(z - floor) for z in corners) > flat_tolerance_m:
        raise ContainerGeometryError(f'container floor is not flat under the drop footprint: {floor:.4f} vs {corners}')
    up = ray_hits(tris, [cx, cy, floor + 1e-4], [0.0, 0.0, 1.0])
    rim = float(tris[:, :, 2].max())
    if len(up) and floor + 1e-4 + up[0] < rim - 1e-6:
        raise ContainerGeometryError('container cavity is covered above the drop point')
    if rim - floor < 0.01:
        raise ContainerGeometryError('container wall height above the floor is below 10 mm')
    extents, wall_tops = {}, {}
    for name, direction in (('x_minus', (-1, 0, 0)), ('x_plus', (1, 0, 0)), ('y_minus', (0, -1, 0)), ('y_plus', (0, 1, 0))):
        def wall_at(z):
            hits = ray_hits(tris, [cx, cy, z], direction)
            return float(hits[0]) if len(hits) else None
        nearest, top = math.inf, None
        for k in range(levels):
            z = floor + 0.002 + (rim - floor - 0.004) * k / (levels - 1)
            distance = wall_at(z)
            if distance is None:
                break                       # an open side (e.g. a stacking bin's front opening) ends here
            nearest, top = min(nearest, distance), z
        if top is None:
            raise ContainerGeometryError(f'container has no wall towards {name} above its floor')
        if top < rim - 0.003:
            low, high = top, top + (rim - floor - 0.004) / (levels - 1)
            for _ in range(20):             # the side's wall top, to 0.1 mm or better
                middle = (low + high) / 2
                low, high = (middle, high) if wall_at(middle) is not None else (low, middle)
            top = low
        if top - floor < 0.01:
            raise ContainerGeometryError(f'container wall towards {name} rises less than 10 mm above the floor')
        extents[name], wall_tops[name] = nearest, float(top)
    interior = dict(x=[cx - extents['x_minus'], cx + extents['x_plus']], y=[cy - extents['y_minus'], cy + extents['y_plus']])
    if (interior['x'][0] > cx - fx or interior['x'][1] < cx + fx or
            interior['y'][0] > cy - fy or interior['y'][1] < cy + fy):
        raise ContainerGeometryError(f'drop footprint {footprint_xy} exceeds the measured interior {interior}')
    return dict(floor_top_z=float(floor), rim_z=rim, interior_x=interior['x'], interior_y=interior['y'],
                wall_distances=extents, wall_top_z=wall_tops)
