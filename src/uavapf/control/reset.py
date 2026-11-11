"""Sampled stall-triggered velocity reset from eq. (16), paper/main.tex."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from uavapf.control.apf import APFController
from uavapf.control.tracking import TrackingController

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class ResetConfig:
    """Hybrid reset thresholds from Table 4, paper/main.tex.

    ``speed_tol`` and ``delta`` are in m/s; ``position_tol`` in m;
    ``stall_time`` and ``dwell_time`` in s. ``max_resets`` is a count per UAV.
    """

    speed_tol: float = 0.05
    position_tol: float = 0.3
    delta: float = 0.5
    stall_time: float = 1.0
    dwell_time: float = 2.0
    max_resets: int = 5

    def __post_init__(self) -> None:
        for name in ("speed_tol", "position_tol", "delta", "stall_time", "dwell_time"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f"{name} must be positive and finite")
        if not (isinstance(self.max_resets, int) and self.max_resets >= 0):
            raise ValueError("max_resets must be a nonnegative integer")


class StallReset:
    """Explicit reset state for one simulation, sampled at each RK4 step.

    Construct a new instance for each run. The continuous-time trigger in eq. (16)
    is approximated at the simulator's step times. With a moving centre (T13b) the
    position test and the reset direction use the current slot p*(t); the speed test
    stays on the absolute speed.
    """

    def __init__(
        self,
        controller: APFController | TrackingController,
        config: ResetConfig | None = None,
    ) -> None:
        self.controller = controller
        self.config = ResetConfig() if config is None else config
        n = len(controller.slots)
        self.stall_since = np.full(n, np.nan)
        self.last_reset = np.full(n, -np.inf)
        self.count = np.zeros(n, dtype=np.int64)

    def __call__(self, t: float, p: ArrayLike, v: ArrayLike) -> FloatArray:
        """Return velocities in m/s after possible resets at time ``t`` in s.

        Parameters
        ----------
        t : float
            Current sample time in s.
        p, v : array_like, shape (N, 3)
            Current positions in m and velocities in m/s.

        Returns
        -------
        ndarray, shape (N, 3)
            New float64 velocities in m/s; input arrays are not modified.
        """
        if not math.isfinite(t) or t < 0:
            raise ValueError("t must be finite and nonnegative")
        p, v = self.controller.validate_state(p, v)
        c = self.config
        target = self.controller.slots_at(t)
        stalled = (np.linalg.norm(v, axis=1) <= c.speed_tol) & (
            np.linalg.norm(p - target, axis=1) >= c.position_tol
        )
        self.stall_since[~stalled] = np.nan
        newly_stalled = stalled & np.isnan(self.stall_since)
        self.stall_since[newly_stalled] = t
        eligible = (
            stalled
            & (t - self.stall_since >= c.stall_time)
            & (t - self.last_reset >= c.dwell_time)
            & (self.count < c.max_resets)
        )
        out = np.array(v, dtype=np.float64, copy=True)
        if not np.any(eligible):
            return out

        # eq. (16), main.tex: closest sensed obstacle normal and gyro axis.
        rho, normal = self.controller.geometry.distance(p)
        omega = self.controller.components(p, v, slots=target).omega
        axis = np.array([0.0, 0.0, 1.0])
        goal = target - p
        direction = np.cross(axis, goal)
        if rho.shape[1]:
            nearest = np.argmin(rho, axis=1)
            selected = normal[np.arange(len(p)), nearest]
            primary = np.cross(omega, -selected)
            use_primary = (rho[np.arange(len(p)), nearest] <= self.controller.config.rho0) & (
                np.linalg.norm(primary, axis=1) > 0
            )
            direction[use_primary] = primary[use_primary]
        length = np.linalg.norm(direction, axis=1)
        apply = eligible & (length > 0)
        out[apply] += c.delta * direction[apply] / length[apply, None]
        self.last_reset[apply] = t
        self.count[apply] += 1
        self.stall_since[apply] = np.nan
        return out
