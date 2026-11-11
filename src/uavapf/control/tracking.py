"""APF with a moving formation centre (main.tex Remark "Moving formation"; D009, D010).

Spec: docs/specs/T13b-tracking.md §2. Slots ride the reference path,
p_i*(t) = c(t) + δ_i with δ_i = slots_i − goal; the T1 goal factor stays on the final
slots (D010), handled inside ``APFController.components``.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from uavapf.control.apf import APFConfig, APFController, ControlComponents
from uavapf.control.reference import StraightPath
from uavapf.geometry import Cylinder, Sphere

FloatArray = NDArray[np.float64]


def moving_slots(slots: FloatArray, path: StraightPath, t: float | ArrayLike) -> FloatArray:
    """p*(t) = c(t) + (slots − goal) in m: (N, 3) for scalar ``t``, (K, N, 3) for (K,)."""
    # Written as slots + (c(t) − goal) so a still path returns ``slots`` bit-for-bit.
    return slots + (path.position(t) - np.asarray(path.goal))[..., None, :]


def moving_slot_velocity(n: int, path: StraightPath, t: float | ArrayLike) -> FloatArray:
    """ṗ*(t) = ċ(t) for every agent in m/s: (N, 3) or (K, N, 3)."""
    vel = path.velocity(t)[..., None, :]
    return np.broadcast_to(vel, vel.shape[:-2] + (n, 3)).copy()


class TrackingController:
    """Pure callable controller of Remark 2 for slots on a moving reference.

    Parameters
    ----------
    slots : array_like, shape (N, 3)
        Final formation slots in m.
    edges : sequence of pairs of int
        Formation graph (Morse coupling).
    obstacles : sequence of Sphere or Cylinder
        Static obstacles in m.
    path : StraightPath
        Reference of the formation centre; ``path.goal`` is the final centre.
    config : APFConfig, optional
        Gains (Table 4 defaults).
    """

    def __init__(
        self,
        slots: ArrayLike,
        edges: Sequence[tuple[int, int]],
        obstacles: Sequence[Sphere | Cylinder],
        path: StraightPath,
        config: APFConfig | None = None,
    ) -> None:
        self.base = APFController(slots, edges, obstacles, config)
        self.path = path
        self.config = self.base.config
        self.geometry = self.base.geometry
        self.obstacles = self.base.obstacles
        self.slots = self.base.slots

    def validate_state(self, p: ArrayLike, v: ArrayLike) -> tuple[FloatArray, FloatArray]:
        """See :meth:`APFController.validate_state`."""
        return self.base.validate_state(p, v)

    def components(
        self, p: ArrayLike, v: ArrayLike, *, slots: ArrayLike | None = None
    ) -> ControlComponents:
        """See :meth:`APFController.components`."""
        return self.base.components(p, v, slots=slots)

    def slots_at(self, t: float | ArrayLike) -> FloatArray:
        """Moving slots p*(t) in m at time ``t`` in s."""
        return moving_slots(self.slots, self.path, t)

    def slot_velocity(self, t: float | ArrayLike) -> FloatArray:
        """Slot velocities ṗ*(t) in m/s at time ``t`` in s."""
        return moving_slot_velocity(len(self.slots), self.path, t)

    def __call__(self, t: float, p: ArrayLike, v: ArrayLike) -> FloatArray:
        """Acceleration command in m/s² at time ``t`` in s."""
        p, v = self.validate_state(p, v)
        e = v - self.path.velocity(t)
        terms = self.base.components(p, e, slots=self.slots_at(t))
        c = self.config
        # Remark 2, main.tex: u = c̈ + F − K_d e − K_r D e + ω × e
        return (
            self.path.acceleration(t)
            + terms.force
            - c.kd * e
            - c.kr * terms.damping
            + terms.gyroscopic
        )
