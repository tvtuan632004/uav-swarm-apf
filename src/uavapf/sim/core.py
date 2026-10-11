"""Double-integrator swarm plant with fixed-step RK4 (spec: docs/specs/T03-sim.md).

Model (1), main.tex: p_i' = v_i, v_i' = sat(u_i(t, p, v), a_max).
Norm saturation (1), main.tex: sat(x, L) = x * min(1, L / ||x||), row-wise.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]
Controller = Callable[[float, FloatArray, FloatArray], ArrayLike]
"""``u(t, p, v) -> (N, 3)`` acceleration command in m/s^2 for the whole swarm."""
StopFn = Callable[[float, FloatArray, FloatArray], bool]
ResetFn = Callable[[float, FloatArray, FloatArray], ArrayLike]
AccelFn = Callable[[float, FloatArray, FloatArray], FloatArray]

_STEP_TOL = 1e-9  # relative tolerance when rounding t_final / dt up to an integer


# --------------------------------------------------------------------------- #
# Saturation
# --------------------------------------------------------------------------- #
def saturate_norm(x: ArrayLike, limit: float | None) -> FloatArray:
    """Scale every row of ``x`` so that its Euclidean norm is at most ``limit``.

    Implements norm saturation in eq. (1), main.tex over the last axis. Rows already inside
    the ball are returned bit-for-bit unchanged; the zero row stays zero.

    Parameters
    ----------
    x : array_like, shape (..., 3)
        Vectors to saturate (any unit, e.g. m/s^2 or m/s).
    limit : float or None
        Maximum norm, same unit as ``x``. ``None`` disables saturation.

    Returns
    -------
    numpy.ndarray
        New float64 array with the shape of ``x``.
    """
    out = np.array(x, dtype=np.float64, copy=True)
    if limit is None:
        return out
    norm = np.sqrt(np.einsum("...i,...i->...", out, out))[..., None]
    scale = np.divide(limit, norm, out=np.ones_like(norm), where=norm > limit)
    out *= scale
    return out


# --------------------------------------------------------------------------- #
# Configuration and result container
# --------------------------------------------------------------------------- #
@dataclass(frozen=True, slots=True)
class SimConfig:
    """Integration settings.

    Attributes
    ----------
    dt : float
        Fixed step size in s, > 0.
    t_final : float
        Simulated horizon in s, > 0. Rounded up to a whole number of steps.
    a_max : float or None
        Per-agent acceleration norm limit in m/s^2 (applied inside every RK4 stage).
    v_max : float or None
        Per-agent speed limit in m/s (projection after each full step).
    log_every : int
        Store every ``log_every``-th step; the final state is always stored.
    """

    dt: float
    t_final: float
    a_max: float | None = None
    v_max: float | None = None
    log_every: int = 1

    def __post_init__(self) -> None:
        if not (math.isfinite(self.dt) and self.dt > 0):
            raise ValueError(f"dt must be a positive finite number, got {self.dt}")
        if not (math.isfinite(self.t_final) and self.t_final > 0):
            raise ValueError(f"t_final must be a positive finite number, got {self.t_final}")
        if self.a_max is not None and not self.a_max > 0:
            raise ValueError(f"a_max must be > 0 or None, got {self.a_max}")
        if self.v_max is not None and not self.v_max > 0:
            raise ValueError(f"v_max must be > 0 or None, got {self.v_max}")
        if int(self.log_every) != self.log_every or self.log_every < 1:
            raise ValueError(f"log_every must be an integer >= 1, got {self.log_every}")

    @property
    def n_steps(self) -> int:
        """Number of RK4 steps: ceil(t_final / dt) with a small relative tolerance."""
        ratio = self.t_final / self.dt
        return max(1, math.ceil(ratio * (1.0 - _STEP_TOL)))


@dataclass(frozen=True, slots=True)
class Trajectory:
    """Logged swarm trajectory.

    Attributes
    ----------
    t : ndarray, shape (K,)
        Time stamps in s.
    p : ndarray, shape (K, N, 3)
        Positions in m.
    v : ndarray, shape (K, N, 3)
        Velocities in m/s.
    u : ndarray, shape (K, N, 3)
        Saturated acceleration command in m/s^2 evaluated at ``(t[k], p[k], v[k])``.
    """

    t: FloatArray
    p: FloatArray
    v: FloatArray
    u: FloatArray

    @property
    def n_agents(self) -> int:
        """Number of agents N."""
        return self.p.shape[1]

    def __len__(self) -> int:
        return self.t.shape[0]

    def save(self, path: Path | str, **meta: Any) -> Path:
        """Write a compressed ``.npz`` with keys t, p, v, u and ``meta_<key>``.

        Parent directories are created. The file name is used exactly as given.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        extra = {f"meta_{key}": np.asarray(value) for key, value in meta.items()}
        with path.open("wb") as fh:
            np.savez_compressed(fh, t=self.t, p=self.p, v=self.v, u=self.u, **extra)
        return path

    @classmethod
    def load(cls, path: Path | str) -> Trajectory:
        """Read a file written by :meth:`save` (metadata is ignored; use ``np.load``)."""
        with np.load(Path(path)) as z:
            return cls(
                t=np.ascontiguousarray(z["t"], dtype=np.float64),
                p=np.ascontiguousarray(z["p"], dtype=np.float64),
                v=np.ascontiguousarray(z["v"], dtype=np.float64),
                u=np.ascontiguousarray(z["u"], dtype=np.float64),
            )


# --------------------------------------------------------------------------- #
# Integrator
# --------------------------------------------------------------------------- #
def _rk4_from_k1(
    accel: AccelFn, t: float, p: FloatArray, v: FloatArray, dt: float, a1: FloatArray
) -> tuple[FloatArray, FloatArray]:
    """Classical RK4 step for (p' = v, v' = accel) given the first stage ``a1``."""
    h = 0.5 * dt
    v2 = v + h * a1
    a2 = accel(t + h, p + h * v, v2)
    v3 = v + h * a2
    a3 = accel(t + h, p + h * v2, v3)
    v4 = v + dt * a3
    a4 = accel(t + dt, p + dt * v3, v4)
    w = dt / 6.0
    p_next = p + w * (v + 2.0 * (v2 + v3) + v4)
    v_next = v + w * (a1 + 2.0 * (a2 + a3) + a4)
    return p_next, v_next


def rk4_step(
    accel: AccelFn, t: float, p: ArrayLike, v: ArrayLike, dt: float
) -> tuple[FloatArray, FloatArray]:
    """Advance the double integrator by one classical RK4 step.

    Parameters
    ----------
    accel : callable
        ``accel(t, p, v) -> (N, 3)`` acceleration in m/s^2 (saturation already applied).
    t : float
        Current time in s.
    p, v : array_like, shape (N, 3)
        Positions in m and velocities in m/s. Not modified.
    dt : float
        Step size in s.

    Returns
    -------
    (p_next, v_next) : tuple of ndarray, shape (N, 3)
    """
    p = np.asarray(p, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    a1 = np.asarray(accel(t, p, v), dtype=np.float64)
    return _rk4_from_k1(accel, t, p, v, dt, a1)


def _as_state(x: ArrayLike, name: str) -> FloatArray:
    arr = np.array(x, dtype=np.float64, copy=True, order="C")
    if arr.ndim != 2 or arr.shape[1] != 3 or arr.shape[0] < 1:
        raise ValueError(f"{name} must have shape (N, 3) with N >= 1, got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains non-finite values")
    return arr


def simulate(
    controller: Controller,
    p0: ArrayLike,
    v0: ArrayLike,
    config: SimConfig,
    stop: StopFn | None = None,
    reset: ResetFn | None = None,
) -> Trajectory:
    """Integrate the swarm from ``(p0, v0)`` under ``controller``.

    The controller is called 4 times per step (once per RK4 stage) plus once for the final
    logged state; the first-stage value of each step doubles as the logged command.

    Parameters
    ----------
    controller : callable
        ``u(t, p, v) -> (N, 3)`` acceleration command in m/s^2 (before saturation).
    p0 : array_like, shape (N, 3)
        Initial positions in m. Not modified.
    v0 : array_like, shape (N, 3)
        Initial velocities in m/s. Not modified.
    config : SimConfig
        Step size, horizon, limits and logging stride.
    stop : callable, optional
        ``stop(t, p, v) -> bool`` checked after every step; ``True`` ends the run after
        logging the current state.
    reset : callable, optional
        ``reset(t, p, v) -> v_plus`` applies a discrete velocity reset in m/s after
        each RK4 step. The returned (N, 3) velocity is projected by ``v_max``.

    Returns
    -------
    Trajectory
    """
    p = _as_state(p0, "p0")
    v = _as_state(v0, "v0")
    if p.shape != v.shape:
        raise ValueError(f"p0 and v0 shapes differ: {p.shape} vs {v.shape}")
    shape = p.shape
    a_max, v_max, dt = config.a_max, config.v_max, config.dt

    def accel(t: float, p_: FloatArray, v_: FloatArray) -> FloatArray:
        u = np.asarray(controller(t, p_, v_), dtype=np.float64)
        if u.shape != shape:
            raise ValueError(f"controller returned shape {u.shape}, expected {shape}")
        return saturate_norm(u, a_max) if a_max is not None else u

    n_steps, stride = config.n_steps, config.log_every
    cap = n_steps // stride + 2
    t_log = np.empty(cap)
    p_log = np.empty((cap, *shape))
    v_log = np.empty((cap, *shape))
    u_log = np.empty((cap, *shape))

    a = accel(0.0, p, v)
    t_log[0], p_log[0], v_log[0], u_log[0] = 0.0, p, v, a
    k = 1
    t = 0.0
    for step in range(1, n_steps + 1):
        p, v = _rk4_from_k1(accel, t, p, v, dt, a)
        t = step * dt  # no accumulated round-off in the time stamps
        if reset is not None:
            # eq. (16), main.tex: velocity jump at a sampled hybrid reset.
            v = np.asarray(reset(t, p, v), dtype=np.float64)
            if v.shape != shape or not np.all(np.isfinite(v)):
                raise ValueError("reset must return finite velocities with shape (N, 3)")
        if v_max is not None:
            v = saturate_norm(v, v_max)
        a = accel(t, p, v)  # first stage of the next step == logged command
        last = step == n_steps or (stop is not None and bool(stop(t, p, v)))
        if last or step % stride == 0:
            t_log[k], p_log[k], v_log[k], u_log[k] = t, p, v, a
            k += 1
        if last:
            break

    return Trajectory(
        t=np.ascontiguousarray(t_log[:k]),
        p=np.ascontiguousarray(p_log[:k]),
        v=np.ascontiguousarray(v_log[:k]),
        u=np.ascontiguousarray(u_log[:k]),
    )
