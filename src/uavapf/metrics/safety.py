"""Safety metrics and collision stop (spec: docs/specs/T11b-contact.md, decision D005).

The free set of main.tex §3.3 is F = {rho_ik > 0 for all i, k; r_ij > d_min for all i != j}.
A state outside F is a collision. Every function accepts one state (N, 3) or a
trajectory (K, N, 3) and is vectorised over agents and time.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from uavapf.geometry import ObstacleSet
from uavapf.sim import StopFn

FloatArray = NDArray[np.float64]

_CHUNK = 1 << 16  # points per clearance call, bounds memory for long trajectories


def _as_trajectory(p: ArrayLike) -> FloatArray:
    arr = np.asarray(p, dtype=np.float64)
    if arr.ndim == 2:
        arr = arr[None]
    if arr.ndim != 3 or arr.shape[2] != 3 or arr.shape[1] < 1:
        raise ValueError(f"p must have shape (N, 3) or (K, N, 3), got {np.shape(p)}")
    return arr


def _per_step_clearance(traj: FloatArray, obstacles: ObstacleSet) -> FloatArray:
    """Minimum clearance per time step, shape (K,), in m (+inf without obstacles)."""
    k, n, _ = traj.shape
    if len(obstacles) == 0:
        return np.full(k, np.inf)
    flat = traj.reshape(-1, 3)
    out = np.empty(flat.shape[0])
    for start in range(0, flat.shape[0], _CHUNK):
        block = flat[start : start + _CHUNK]
        out[start : start + _CHUNK] = obstacles.clearance(block).min(axis=1)
    return out.reshape(k, n).min(axis=1)


def _per_step_separation(traj: FloatArray) -> FloatArray:
    """Minimum pairwise distance per time step, shape (K,), in m (+inf for N = 1)."""
    k, n, _ = traj.shape
    if n < 2:
        return np.full(k, np.inf)
    i, j = np.triu_indices(n, 1)
    diff = traj[:, i] - traj[:, j]
    return np.sqrt(np.einsum("kpd,kpd->kp", diff, diff).min(axis=1))


def min_clearance(p: ArrayLike, obstacles: ObstacleSet) -> float:
    """Smallest signed clearance to any obstacle over all agents (and steps), in m.

    Returns ``+inf`` when there are no obstacles.
    """
    return float(_per_step_clearance(_as_trajectory(p), obstacles).min())


def min_separation(p: ArrayLike) -> float:
    """Smallest distance between two agents over all pairs (and steps), in m.

    Returns ``+inf`` for a single agent.
    """
    return float(_per_step_separation(_as_trajectory(p)).min())


def collision_mask(p: ArrayLike, obstacles: ObstacleSet, d_min: float) -> NDArray[np.bool_]:
    """True where the state is outside the free set (rho <= 0 or r <= d_min).

    Parameters
    ----------
    p : array_like, shape (N, 3) or (K, N, 3)
        Positions in m.
    obstacles : ObstacleSet
        Static obstacles.
    d_min : float
        Minimum admissible separation in m.

    Returns
    -------
    ndarray of bool
        Shape (K,) for a trajectory, 0-d for a single state.
    """
    single = np.ndim(p) == 2
    traj = _as_trajectory(p)
    hit = (_per_step_clearance(traj, obstacles) <= 0.0) | (_per_step_separation(traj) <= d_min)
    return np.asarray(hit[0]) if single else hit


def collision_stop(obstacles: ObstacleSet, d_min: float) -> StopFn:
    """Stop function for :func:`uavapf.sim.simulate` that ends a run at the first collision."""

    def stop(t: float, p: FloatArray, v: FloatArray) -> bool:
        return bool(collision_mask(p, obstacles, d_min))

    return stop
