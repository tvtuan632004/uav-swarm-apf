"""Reference path of the formation centre (D009; spec: docs/specs/T13b-tracking.md §1).

Straight line with a minimum-jerk time scaling, main.tex Remark "Moving formation":
s(τ) = 10τ³ − 15τ⁴ + 6τ⁵, τ = clip(t / T, 0, 1), c(t) = start + s(τ) (goal − start).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]

_PEAK_SPEED = 1.875  # max of s'(τ), at τ = 1/2
_PEAK_ACCEL = 10.0 / math.sqrt(3.0)  # max of |s''(τ)|, at τ = 1/2 ± √3/6


@dataclass(frozen=True, slots=True)
class StraightPath:
    """Minimum-jerk straight path from ``start`` to ``goal``.

    Attributes
    ----------
    start, goal : tuple of 3 floats
        End points in m.
    v_ref : float
        Speed limit in m/s, > 0.
    a_ref : float
        Acceleration limit in m/s², > 0.
    """

    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    v_ref: float
    a_ref: float

    def __post_init__(self) -> None:
        for name in ("start", "goal"):
            value = tuple(float(x) for x in getattr(self, name))
            if len(value) != 3 or not all(math.isfinite(x) for x in value):
                raise ValueError(f"{name} must be 3 finite numbers")
            object.__setattr__(self, name, value)
        for name in ("v_ref", "a_ref"):
            value = float(getattr(self, name))
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f"{name} must be positive and finite")
            object.__setattr__(self, name, value)

    @property
    def length(self) -> float:
        """Path length L in m."""
        return math.dist(self.start, self.goal)

    @property
    def duration(self) -> float:
        """Smallest T in s meeting both limits; 0 when start == goal."""
        length = self.length
        if length == 0.0:
            return 0.0
        return max(_PEAK_SPEED * length / self.v_ref, math.sqrt(_PEAK_ACCEL * length / self.a_ref))

    def _tau(self, t: float | ArrayLike) -> tuple[FloatArray, float]:
        duration = self.duration
        t = np.asarray(t, dtype=np.float64)
        if duration == 0.0:
            return np.ones_like(t), duration
        return np.clip(t / duration, 0.0, 1.0), duration

    def position(self, t: float | ArrayLike) -> FloatArray:
        """Centre c(t) in m: shape (3,) for scalar ``t`` (s), (K, 3) for shape (K,)."""
        tau, _ = self._tau(t)
        s = tau**3 * (10.0 - 15.0 * tau + 6.0 * tau**2)
        start = np.asarray(self.start)
        return start + s[..., None] * (np.asarray(self.goal) - start)

    def velocity(self, t: float | ArrayLike) -> FloatArray:
        """Centre velocity ċ(t) = s'(τ)/T · (goal − start) in m/s."""
        tau, duration = self._tau(t)
        if duration == 0.0:
            return np.zeros(tau.shape + (3,))
        ds = 30.0 * tau**2 * (1.0 - tau) ** 2
        return (ds / duration)[..., None] * (np.asarray(self.goal) - np.asarray(self.start))

    def acceleration(self, t: float | ArrayLike) -> FloatArray:
        """Centre acceleration c̈(t) = s''(τ)/T² · (goal − start) in m/s²."""
        tau, duration = self._tau(t)
        if duration == 0.0:
            return np.zeros(tau.shape + (3,))
        dds = 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau)
        return (dds / duration**2)[..., None] * (np.asarray(self.goal) - np.asarray(self.start))
