"""Obstacle geometry: signed distance rho and outward normal n-hat = grad(rho).

Spec: docs/specs/T04-geometry.md. Sign convention: rho > 0 outside, n-hat points away
from the obstacle, so a repulsive force acts along +n-hat.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]

_E_Z = np.array([0.0, 0.0, 1.0])  # normal at a sphere centre (degenerate gradient)


# --------------------------------------------------------------------------- #
# Shapes
# --------------------------------------------------------------------------- #
def _finite_tuple(values: ArrayLike, size: int, name: str) -> tuple[float, ...]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.shape != (size,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be {size} finite numbers, got {values!r}")
    return tuple(float(x) for x in arr)


def _positive(value: float, name: str) -> float:
    value = float(value)
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{name} must be a positive finite number, got {value}")
    return value


@dataclass(frozen=True, slots=True)
class Sphere:
    """Sphere obstacle.

    Attributes
    ----------
    center : tuple of 3 float
        Centre in m.
    radius : float
        Radius in m, > 0.
    """

    center: tuple[float, float, float]
    radius: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", _finite_tuple(self.center, 3, "center"))
        object.__setattr__(self, "radius", _positive(self.radius, "radius"))


@dataclass(frozen=True, slots=True)
class Cylinder:
    """Vertical (z-aligned) cylinder obstacle, optionally capped.

    Attributes
    ----------
    center_xy : tuple of 2 float
        Axis position (x, y) in m.
    radius : float
        Radius in m, > 0.
    z_min, z_max : float
        Vertical extent in m; ``-inf`` / ``+inf`` for an open end (e.g. a building is
        ``z_min=-inf, z_max=height``).
    """

    center_xy: tuple[float, float]
    radius: float
    z_min: float = -math.inf
    z_max: float = math.inf

    def __post_init__(self) -> None:
        object.__setattr__(self, "center_xy", _finite_tuple(self.center_xy, 2, "center_xy"))
        object.__setattr__(self, "radius", _positive(self.radius, "radius"))
        z_min, z_max = float(self.z_min), float(self.z_max)
        if math.isnan(z_min) or math.isnan(z_max) or not z_max > z_min:
            raise ValueError(f"need z_max > z_min, got [{z_min}, {z_max}]")
        object.__setattr__(self, "z_min", z_min)
        object.__setattr__(self, "z_max", z_max)


Obstacle = Sphere | Cylinder


# --------------------------------------------------------------------------- #
# Vectorised kernels: N points x M obstacles of one type
# --------------------------------------------------------------------------- #
def _as_points(points: ArrayLike) -> FloatArray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points must have shape (N, 3), got {pts.shape}")
    return pts


def sphere_distance(
    points: ArrayLike, centers: ArrayLike, radii: ArrayLike
) -> tuple[FloatArray, FloatArray]:
    """Signed distance and outward normal for M spheres, eq. (G1).

    Parameters
    ----------
    points : array_like, shape (N, 3)
        Query points in m.
    centers : array_like, shape (M, 3)
        Sphere centres in m.
    radii : array_like, shape (M,)
        Radii in m.

    Returns
    -------
    rho : ndarray, shape (N, M)
        Signed distance in m (negative inside).
    n : ndarray, shape (N, M, 3)
        Unit outward normal; ``+e_z`` at a centre.
    """
    pts = _as_points(points)
    centers = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
    radii = np.asarray(radii, dtype=np.float64).reshape(-1)

    d = pts[:, None, :] - centers[None, :, :]  # (N, M, 3)
    dist = np.sqrt(np.einsum("nmk,nmk->nm", d, d))
    rho = dist - radii  # eq. (G1)
    ok = dist > 0.0
    n = np.divide(d, dist[..., None], out=np.zeros_like(d), where=ok[..., None])
    n[~ok] = _E_Z
    return rho, n


def cylinder_distance(
    points: ArrayLike,
    centers_xy: ArrayLike,
    radii: ArrayLike,
    z_min: ArrayLike,
    z_max: ArrayLike,
) -> tuple[FloatArray, FloatArray]:
    """Signed distance and outward normal for M vertical cylinders, eq. (G2).

    Parameters
    ----------
    points : array_like, shape (N, 3)
        Query points in m.
    centers_xy : array_like, shape (M, 2)
        Axis positions in m.
    radii, z_min, z_max : array_like, shape (M,)
        Radius and vertical extent in m (bounds may be infinite).

    Returns
    -------
    rho : ndarray, shape (N, M)
        Exact signed distance in m (negative inside).
    n : ndarray, shape (N, M, 3)
        Unit outward normal; on the axis the radial direction is taken as ``+e_x``.
    """
    pts = _as_points(points)
    centers_xy = np.asarray(centers_xy, dtype=np.float64).reshape(-1, 2)
    radii = np.asarray(radii, dtype=np.float64).reshape(-1)
    z_lo = np.asarray(z_min, dtype=np.float64).reshape(-1)
    z_hi = np.asarray(z_max, dtype=np.float64).reshape(-1)

    q = pts[:, None, :2] - centers_xy[None, :, :]  # (N, M, 2)
    rq = np.hypot(q[..., 0], q[..., 1])
    on_axis = rq == 0.0
    er = np.divide(q, rq[..., None], out=np.zeros_like(q), where=~on_axis[..., None])
    er[on_axis] = (1.0, 0.0)

    d_r = rq - radii
    z = pts[:, 2:3]
    above = z - z_hi  # -inf for an open top: never NaN
    below = z_lo - z
    d_z = np.maximum(above, below)
    s = np.where(above >= below, 1.0, -1.0)  # sign of the nearer cap

    a = np.maximum(d_r, 0.0)
    b = np.maximum(d_z, 0.0)
    outside = (d_r > 0.0) | (d_z > 0.0)

    # outside: distance to the closest surface point (side, cap or rim), eq. (G2)
    rho_out = np.hypot(a, b)
    inv = np.divide(1.0, rho_out, out=np.zeros_like(rho_out), where=outside)
    # inside: nearest face is the side if d_r >= d_z, otherwise a cap
    side = d_r >= d_z

    rho = np.where(outside, rho_out, np.maximum(d_r, d_z))
    w_r = np.where(outside, a * inv, side.astype(np.float64))
    w_z = np.where(outside, b * inv, (~side).astype(np.float64)) * s

    n = np.empty((*rho.shape, 3))
    n[..., :2] = er * w_r[..., None]
    n[..., 2] = w_z
    return rho, n


# --------------------------------------------------------------------------- #
# Mixed collection
# --------------------------------------------------------------------------- #
class ObstacleSet:
    """Immutable, vectorised collection of spheres and cylinders.

    Column ``j`` of every output corresponds to ``obstacles[j]`` as given.
    """

    __slots__ = ("_m", "_sph_idx", "_sph_c", "_sph_r", "_cyl_idx", "_cyl_c", "_cyl_r", "_cyl_z")

    def __init__(self, obstacles: Sequence[Obstacle]) -> None:
        obstacles = list(obstacles)
        for ob in obstacles:
            if not isinstance(ob, Sphere | Cylinder):
                raise TypeError(f"unsupported obstacle type: {type(ob).__name__}")
        sph = [(j, ob) for j, ob in enumerate(obstacles) if isinstance(ob, Sphere)]
        cyl = [(j, ob) for j, ob in enumerate(obstacles) if isinstance(ob, Cylinder)]
        self._m = len(obstacles)
        self._sph_idx = np.array([j for j, _ in sph], dtype=np.intp)
        self._sph_c = np.array([ob.center for _, ob in sph], dtype=np.float64).reshape(-1, 3)
        self._sph_r = np.array([ob.radius for _, ob in sph], dtype=np.float64)
        self._cyl_idx = np.array([j for j, _ in cyl], dtype=np.intp)
        self._cyl_c = np.array([ob.center_xy for _, ob in cyl], dtype=np.float64).reshape(-1, 2)
        self._cyl_r = np.array([ob.radius for _, ob in cyl], dtype=np.float64)
        self._cyl_z = np.array([(ob.z_min, ob.z_max) for _, ob in cyl], dtype=np.float64)
        self._cyl_z = self._cyl_z.reshape(-1, 2)

    def __len__(self) -> int:
        return self._m

    def clearance(self, points: ArrayLike) -> FloatArray:
        """Signed distance only, shape (N, M) in m; cheaper than :meth:`distance`.

        Equals ``distance(points)[0]`` but skips the normals, so it is the right call
        for metrics over long trajectories (memory O(N·M) instead of O(3·N·M)).
        """
        pts = _as_points(points)
        rho = np.empty((pts.shape[0], self._m))
        if self._sph_idx.size:
            d = pts[:, None, :] - self._sph_c[None, :, :]
            rho[:, self._sph_idx] = np.sqrt(np.einsum("nmk,nmk->nm", d, d)) - self._sph_r
        if self._cyl_idx.size:
            q = pts[:, None, :2] - self._cyl_c[None, :, :]
            d_r = np.hypot(q[..., 0], q[..., 1]) - self._cyl_r
            z = pts[:, 2:3]
            d_z = np.maximum(self._cyl_z[:, 0] - z, z - self._cyl_z[:, 1])
            outside = np.hypot(np.maximum(d_r, 0.0), np.maximum(d_z, 0.0))
            inside = np.minimum(np.maximum(d_r, d_z), 0.0)
            rho[:, self._cyl_idx] = outside + inside  # eq. (G2): one of the two is zero
        return rho

    def distance(self, points: ArrayLike) -> tuple[FloatArray, FloatArray]:
        """Signed distance and outward normal from every point to every obstacle.

        Parameters
        ----------
        points : array_like, shape (N, 3)
            Query points in m.

        Returns
        -------
        rho : ndarray, shape (N, M)
            Signed distance in m.
        n : ndarray, shape (N, M, 3)
            Unit outward normal.
        """
        pts = _as_points(points)
        n_pts = pts.shape[0]
        rho = np.empty((n_pts, self._m))
        nrm = np.empty((n_pts, self._m, 3))
        if self._sph_idx.size:
            r, v = sphere_distance(pts, self._sph_c, self._sph_r)
            rho[:, self._sph_idx], nrm[:, self._sph_idx] = r, v
        if self._cyl_idx.size:
            r, v = cylinder_distance(
                pts, self._cyl_c, self._cyl_r, self._cyl_z[:, 0], self._cyl_z[:, 1]
            )
            rho[:, self._cyl_idx], nrm[:, self._cyl_idx] = r, v
        return rho, nrm
