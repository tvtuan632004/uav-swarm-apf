"""Baseline B1: Khatib APF with formation attraction and pairwise repulsion.

Spec: docs/specs/T06-b1-khatib.md; main.tex §6.3 (B1). The obstacle potential is the
FIRAS function of Khatib (1986, Sec. 3): 0.5 * eta * (1/rho - 1/rho0)^2 inside rho0.
It has no goal factor (so it shows the GNRON problem, deficiency D1) and the controller
has no gyroscopic force or reset (so it stalls in local minima, D2).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from uavapf.control.reference import StraightPath
from uavapf.control.tracking import moving_slot_velocity, moving_slots
from uavapf.geometry import Cylinder, ObstacleSet, Sphere

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class KhatibConfig:
    """Gains of B1; defaults reuse Table 4 of main.tex for a like-for-like start.

    ``ka`` 1/s², ``eta`` m⁴/s², ``rho0`` m, ``kb`` m⁴/s², ``d_s`` m, ``kd`` 1/s,
    ``contact_eps`` m (clamp outside the free set, D005). Tuned in T17 with the same
    budget as the proposed method.
    """

    ka: float = 1.0
    eta: float = 1.0
    rho0: float = 2.0
    kb: float = 0.5
    d_s: float = 1.2
    kd: float = 1.5
    contact_eps: float = 1e-3

    def __post_init__(self) -> None:
        for name in ("ka", "eta", "rho0", "kb", "d_s", "kd", "contact_eps"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f"{name} must be positive and finite")
        if not self.contact_eps < min(self.rho0, self.d_s):
            raise ValueError("need contact_eps < min(rho0, d_s)")


class KhatibController:
    """Callable B1 controller ``u = -grad(U_a + U_o + U_p) - kd v`` (m/s²).

    Parameters
    ----------
    slots : array_like, shape (N, 3)
        Formation slots in m.
    obstacles : sequence of Sphere or Cylinder
        Static obstacles (capped cylinders allowed).
    config : KhatibConfig, optional
    path : StraightPath, optional
        Moving formation centre (T13b, D009); then
        ``u = c̈ - grad U(p; p*(t)) - kd (v - ċ)`` with p*(t) = c(t) + slots - goal.
    """

    def __init__(
        self,
        slots: ArrayLike,
        obstacles: Sequence[Sphere | Cylinder],
        config: KhatibConfig | None = None,
        path: StraightPath | None = None,
    ) -> None:
        self.config = KhatibConfig() if config is None else config
        self.path = path
        q = np.array(slots, dtype=np.float64, copy=True)
        if q.ndim != 2 or q.shape[1] != 3 or q.shape[0] < 1 or not np.isfinite(q).all():
            raise ValueError("slots must be finite with shape (N, 3), N >= 1")
        q.flags.writeable = False
        self.slots = q
        self.obstacles = tuple(obstacles)
        self.geometry = ObstacleSet(self.obstacles)
        if len(self.obstacles) and np.any(self.geometry.clearance(q) <= 0):
            raise ValueError("each slot must lie outside every obstacle")
        self.ii, self.jj = np.triu_indices(len(q), k=1)

    def slots_at(self, t: float | ArrayLike) -> FloatArray:
        """Slots p*(t) in m at time ``t`` in s (static without a path)."""
        if self.path is None:
            return self.slots
        return moving_slots(self.slots, self.path, t)

    def slot_velocity(self, t: float | ArrayLike) -> FloatArray:
        """Slot velocities ṗ*(t) in m/s at time ``t`` in s."""
        if self.path is None:
            return np.zeros_like(self.slots)
        return moving_slot_velocity(len(self.slots), self.path, t)

    def _terms(
        self, p: ArrayLike, v: ArrayLike | None, slots: FloatArray | None = None
    ) -> tuple[float, FloatArray]:
        p = np.asarray(p, dtype=np.float64)
        if p.shape != self.slots.shape or not np.isfinite(p).all():
            raise ValueError("p must be finite with the same (N, 3) shape as slots")
        c = self.config
        n = p.shape[0]
        delta = p - (self.slots if slots is None else slots)
        potential = 0.5 * c.ka * float(np.sum(delta * delta))  # eq. (4), main.tex
        force = -c.ka * delta

        if len(self.obstacles):
            rho, normal = self.geometry.distance(p)
            rho = np.maximum(rho, c.contact_eps)  # D005: finite outside the free set
            ni, mk = np.nonzero(rho <= c.rho0)
            if ni.size:
                r_k = rho[ni, mk]
                w = 1.0 / r_k - 1.0 / c.rho0
                potential += 0.5 * c.eta * float(np.sum(w * w))  # Khatib 1986, FIRAS
                mag = c.eta * w / r_k**2  # -dU/drho, pushes along +n
                for axis in range(3):
                    force[:, axis] += np.bincount(
                        ni, weights=mag * normal[ni, mk, axis], minlength=n
                    )

        if self.ii.size:
            x = p[self.ii] - p[self.jj]
            r_geo = np.sqrt(np.einsum("ij,ij->i", x, x))
            sel = np.flatnonzero(r_geo <= c.d_s)
            if sel.size:
                ii, jj = self.ii[sel], self.jj[sel]
                unit = np.empty((sel.size, 3))
                unit[:] = (1.0, 0.0, 0.0)  # coincident agents
                np.divide(x[sel], r_geo[sel, None], out=unit, where=r_geo[sel, None] > 0)
                r = np.maximum(r_geo[sel], c.contact_eps)
                w = 1.0 / r - 1.0 / c.d_s
                potential += 0.5 * c.kb * float(np.sum(w * w))  # Khatib form on r_ij
                f = (c.kb * w / r**2)[:, None] * unit  # on agent i, away from j
                for axis in range(3):
                    force[:, axis] += np.bincount(ii, weights=f[:, axis], minlength=n)
                    force[:, axis] -= np.bincount(jj, weights=f[:, axis], minlength=n)

        if v is not None:
            v = np.asarray(v, dtype=np.float64)
            if v.shape != p.shape or not np.isfinite(v).all():
                raise ValueError("v must be finite with the same shape as p")
            force = force - c.kd * v
        return potential, force

    def potential(self, p: ArrayLike) -> float:
        """Total potential U_a + U_o + U_p in m²/s² at positions ``p`` in m."""
        return self._terms(p, None)[0]

    def __call__(self, t: float, p: ArrayLike, v: ArrayLike) -> FloatArray:
        """Acceleration command in m/s² at time ``t`` in s."""
        if self.path is None:
            return self._terms(p, v)[1]
        # T13b: same tracking law as Remark 2 without the T2-T4 terms.
        e = np.asarray(v, dtype=np.float64) - self.path.velocity(t)
        return self.path.acceleration(t) + self._terms(p, e, self.slots_at(t))[1]
