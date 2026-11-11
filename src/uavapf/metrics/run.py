"""Per-run evaluation metrics of main.tex (spec: docs/specs/T12-metrics.md).

Time integrals use the trapezoidal rule on the logged samples; the jerk integral is exact
for a command that is piecewise linear between samples.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from uavapf.geometry import ObstacleSet
from uavapf.metrics.safety import (
    _per_step_clearance,
    _per_step_separation,
)
from uavapf.sim import Trajectory

FloatArray = NDArray[np.float64]

_SAT_TOL = 1e-9  # relative: |u| >= (1 - tol) a_max counts as saturated


@dataclass(frozen=True, slots=True)
class RunMetrics:
    """Metrics of one run; units in docs/specs/T12-metrics.md."""

    success: bool
    collided: bool
    completion_time: float  # s, inf if the formation is never reached
    min_separation: float  # m
    min_clearance: float  # m
    formation_error: float  # m
    control_effort: float  # m^2/s^3
    rms_jerk: float  # m/s^3
    saturation_fraction: float  # 1

    def as_dict(self) -> dict[str, float | bool]:
        """Plain dict, e.g. for one row of a results table."""
        return asdict(self)


def _trapezoid(y: FloatArray, t: FloatArray) -> float:
    return float(np.sum(0.5 * (y[1:] + y[:-1]) * np.diff(t)))


def evaluate_run(
    traj: Trajectory,
    slots: ArrayLike,
    obstacles: ObstacleSet,
    *,
    d_min: float,
    eps_p: float,
    a_max: float | None = None,
    t_max: float | None = None,
) -> RunMetrics:
    """Compute every per-run metric of the paper for one trajectory.

    Parameters
    ----------
    traj : Trajectory
        Logged run (use ``log_every = 1`` for paper numbers).
    slots : array_like, shape (N, 3) or (K, N, 3)
        Formation slots p_i* in m: stationary (Assumption 3) or one set per logged
        sample (moving centre, T13b). Completion time is measured to the last sample's
        slots, the final formation.
    obstacles : ObstacleSet
        Static obstacles of the scenario.
    d_min : float
        Minimum admissible separation in m (collision threshold).
    eps_p : float
        Arrival tolerance in m.
    a_max : float, optional
        Acceleration limit used in the run, m/s²; ``None`` → saturation fraction 0.
    t_max : float, optional
        Deadline for success in s; defaults to the last logged time.
    """
    slots = np.asarray(slots, dtype=np.float64)
    t, p, u = traj.t, traj.p, traj.u
    if p.ndim != 3 or slots.shape not in (p.shape[1:], p.shape):
        raise ValueError(f"slots {slots.shape} do not match trajectory agents {p.shape[1:]}")
    if t.shape[0] < 2:
        raise ValueError("need at least two logged samples")
    if not d_min > 0 or not eps_p > 0:
        raise ValueError("d_min and eps_p must be > 0")
    if a_max is not None and not a_max > 0:
        raise ValueError("a_max must be > 0 or None")
    n = p.shape[1]
    duration = float(t[-1] - t[0])

    clearance = _per_step_clearance(p, obstacles)
    separation = _per_step_separation(p)
    collided = bool(np.any(clearance <= 0.0) or np.any(separation <= d_min))

    err = np.linalg.norm(p - slots, axis=2)  # (K, N)
    final = slots if slots.ndim == 2 else slots[-1]
    err_final = err if slots.ndim == 2 else np.linalg.norm(p - final, axis=2)
    arrived = np.flatnonzero(np.all(err_final <= eps_p, axis=1))
    completion = float(t[arrived[0]] - t[0]) if arrived.size else math.inf
    deadline = duration if t_max is None else float(t_max)
    success = (not collided) and completion <= deadline

    formation_error = _trapezoid(err.mean(axis=1), t) / duration
    u_sq = np.einsum("knd,knd->kn", u, u)
    control_effort = _trapezoid(u_sq.sum(axis=1), t)
    du = np.diff(u, axis=0)
    jerk_integral = float(np.sum(np.einsum("knd,knd->k", du, du) / np.diff(t)))
    rms_jerk = math.sqrt(jerk_integral / (n * duration))
    if a_max is None:
        saturation = 0.0
    else:
        active = (np.sqrt(u_sq) >= (1.0 - _SAT_TOL) * a_max).astype(np.float64)
        saturation = _trapezoid(active.mean(axis=1), t) / duration

    return RunMetrics(
        success=bool(success),
        collided=collided,
        completion_time=completion,
        min_separation=float(separation.min()),
        min_clearance=float(clearance.min()),
        formation_error=formation_error,
        control_effort=control_effort,
        rms_jerk=rms_jerk,
        saturation_fraction=saturation,
    )


def controller_cost(
    controller: Callable[[float, FloatArray, FloatArray], ArrayLike],
    p: ArrayLike,
    v: ArrayLike,
    *,
    reps: int = 200,
    warmup: int = 10,
) -> tuple[float, float]:
    """Wall time of one control step per agent: (mean, 99th percentile) in µs."""
    p = np.asarray(p, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    n = p.shape[0]
    for _ in range(warmup):
        controller(0.0, p, v)
    samples = np.empty(reps)
    for k in range(reps):
        t0 = time.perf_counter()
        controller(0.0, p, v)
        samples[k] = time.perf_counter() - t0
    per_agent_us = samples * 1e6 / n
    return float(per_agent_us.mean()), float(np.percentile(per_agent_us, 99))
