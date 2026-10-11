"""Continuous APF controller for static formation slots (paper/main.tex, eqs. 4-15)."""

from __future__ import annotations

import math
import operator
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from uavapf.geometry import Cylinder, ObstacleSet, Sphere

FloatArray = NDArray[np.float64]
_EYE = np.eye(3)


def _scatter_add(out: FloatArray, idx: NDArray[np.intp], vals: FloatArray) -> None:
    """out[idx[k]] += vals[k] for (N, 3) ``out``; faster than ``np.add.at``."""
    n = out.shape[0]
    for axis in range(3):
        out[:, axis] += np.bincount(idx, weights=vals[:, axis], minlength=n)


def _cross(a: FloatArray, b: FloatArray) -> FloatArray:
    """Row-wise cross product of (K, 3) arrays (avoids np.cross overhead)."""
    out = np.empty_like(a)
    out[:, 0] = a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1]
    out[:, 1] = a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2]
    out[:, 2] = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    return out


@dataclass(frozen=True, slots=True)
class APFConfig:
    """Gains and distances for the stationary-slot APF.

    ``ka`` is in 1/s², ``kd`` in 1/s, ``kr`` in s, ``de`` in m²/s²,
    ``alpha`` in 1/m, ``kg`` in 1/(m·s), and ``mu`` in m. ``rho0``,
    ``rho1``, ``d_min``, and ``d_s`` are in m. For exponents ``m`` and ``n``,
    ``eta`` is in m^(m+2-n)/s² and ``kb`` in m^(m+2)/s², so every potential
    term has units m²/s² per unit mass. Initial values follow Table 4.
    ``contact_eps`` (m) is the clamp used outside the free set (D005, T11b): clearances
    and pair gaps below it are evaluated as if they were equal to it.
    """

    ka: float = 1.0
    eta: float = 1.0
    rho0: float = 2.0
    m: int = 3
    n: int = 2
    de: float = 1.0
    alpha: float = 1.0
    kb: float = 0.5
    d_min: float = 0.5
    d_s: float = 1.2
    kd: float = 1.5
    kr: float = 0.1
    kg: float = 1.0
    mu: float = 0.2
    rho1: float = 0.5
    contact_eps: float = 1e-3

    def __post_init__(self) -> None:
        for name in ("ka", "eta", "rho0", "de", "alpha", "kb", "d_min", "d_s", "kd", "mu"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f"{name} must be positive and finite")
        for name in ("kr", "kg"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0):
                raise ValueError(f"{name} must be nonnegative and finite")
        if not (isinstance(self.m, int) and self.m >= 3):
            raise ValueError("m must be an integer >= 3")
        if not (isinstance(self.n, int) and self.n >= 2):
            raise ValueError("n must be an integer >= 2")
        if not 0 < self.rho1 < self.rho0:
            raise ValueError("need 0 < rho1 < rho0")
        if not self.d_min < self.d_s:
            raise ValueError("need d_min < d_s")
        if not (
            math.isfinite(self.contact_eps)
            and 0 < self.contact_eps < min(self.rho1, self.d_s - self.d_min)
        ):
            raise ValueError("need 0 < contact_eps < min(rho1, d_s - d_min)")


@dataclass(frozen=True, slots=True)
class ControlComponents:
    """Acceleration components in m/s², rotation rate in 1/s, and potential in m²/s²."""

    potential: float
    force: FloatArray
    damping: FloatArray
    gyroscopic: FloatArray
    omega: FloatArray


class APFController:
    """Pure callable controller for static slots and smooth obstacle geometry.

    Parameters
    ----------
    slots : array_like, shape (N, 3)
        Desired positions in m.
    edges : sequence of pairs of int
        Undirected formation graph; Morse coupling applies to these pairs.
    obstacles : sequence of Sphere or Cylinder
        Obstacles in m; cylinders must have infinite vertical extent.
    config : APFConfig
        Gains, influence radii and separation radii in SI units.
    """

    def __init__(
        self,
        slots: ArrayLike,
        edges: Sequence[tuple[int, int]],
        obstacles: Sequence[Sphere | Cylinder],
        config: APFConfig | None = None,
    ) -> None:
        self.config = APFConfig() if config is None else config
        q = np.array(slots, dtype=np.float64, copy=True)
        if q.ndim != 2 or q.shape[1] != 3 or q.shape[0] < 1 or not np.isfinite(q).all():
            raise ValueError("slots must be finite with shape (N, 3), N >= 1")
        self.slots = q
        self.slots.flags.writeable = False
        self.obstacles = tuple(obstacles)
        if any(
            isinstance(ob, Cylinder) and (math.isfinite(ob.z_min) or math.isfinite(ob.z_max))
            for ob in self.obstacles
        ):
            raise ValueError("capped cylinders need a Hessian not defined by paper eq. (2)")
        self.geometry = ObstacleSet(self.obstacles)
        rho_slot, _ = self.geometry.distance(self.slots)
        if np.any(rho_slot <= 0):
            raise ValueError("each slot must lie outside every obstacle")

        ii, jj = np.triu_indices(len(q), k=1)
        self.ii, self.jj = ii, jj
        self.desired_r = np.linalg.norm(q[ii] - q[jj], axis=1)
        if np.any(self.desired_r <= self.config.d_s):
            raise ValueError("every slot pair must be farther apart than d_s")
        edge_set: set[tuple[int, int]] = set()
        for edge in edges:
            try:
                # operator.index accepts int and NumPy integers, rejects floats and bools
                i, j = (operator.index(k) for k in edge)
                if isinstance(i, bool) or isinstance(j, bool):
                    raise TypeError
            except (TypeError, ValueError) as exc:
                raise ValueError(f"edge {edge!r} must be a pair of integer indices") from exc
            if not (0 <= i < len(q) and 0 <= j < len(q) and i != j):
                raise ValueError("edges must be distinct valid agent indices")
            pair = (min(i, j), max(i, j))
            if pair in edge_set:
                raise ValueError("duplicate edge")
            edge_set.add(pair)
        self.edges: tuple[tuple[int, int], ...] = tuple(
            sorted(edge_set, key=lambda e: (e[0], e[1]))
        )
        self.edge_mask = np.array(
            [(int(i), int(j)) in edge_set for i, j in zip(ii, jj, strict=True)], dtype=np.float64
        )
        self._edge_bool = self.edge_mask > 0
        self.radii = np.array([ob.radius for ob in self.obstacles], dtype=np.float64)
        self.projectors = np.array(
            [
                np.eye(3) if isinstance(ob, Sphere) else np.diag([1.0, 1.0, 0.0])
                for ob in self.obstacles
            ],
            dtype=np.float64,
        ).reshape(-1, 3, 3)
        # T11c: per-obstacle data for computing normals of active pairs only.
        self._centers = np.array(
            [
                ob.center if isinstance(ob, Sphere) else (*ob.center_xy, 0.0)
                for ob in self.obstacles
            ],
            dtype=np.float64,
        ).reshape(-1, 3)
        self._planar = np.array([isinstance(ob, Cylinder) for ob in self.obstacles], dtype=bool)

    def validate_state(self, p: ArrayLike, v: ArrayLike) -> tuple[FloatArray, FloatArray]:
        """Return ``(p, v)`` as float64 arrays after checking shape (N, 3) and finiteness.

        Raises
        ------
        ValueError
            If the shapes differ from ``slots`` or a value is not finite.
        """
        p = np.asarray(p, dtype=np.float64)
        v = np.asarray(v, dtype=np.float64)
        if p.shape != self.slots.shape or v.shape != p.shape:
            raise ValueError("p and v must have the same (N, 3) shape as slots")
        if not (np.isfinite(p).all() and np.isfinite(v).all()):
            raise ValueError("p and v must be finite")
        return p, v

    def components(self, p: ArrayLike, v: ArrayLike) -> ControlComponents:
        """Return potential, conservative force, PSD damping and gyroscopic acceleration.

        Parameters
        ----------
        p, v : array_like, shape (N, 3)
            Positions in m and velocities in m/s.

        Returns
        -------
        ControlComponents
            Potential in m²/s²; each vector array has shape (N, 3), unit m/s².
        """
        p, v = self.validate_state(p, v)
        c = self.config
        delta = p - self.slots
        g2 = np.einsum("ij,ij->i", delta, delta)
        g = np.sqrt(g2)
        # eq. (4), main.tex
        potential = 0.5 * c.ka * float(g2.sum())
        force = -c.ka * delta
        damping = c.ka * v  # eq. (12), main.tex (new array)
        omega = np.zeros_like(p)

        if len(self.obstacles):
            # T11c: only agent-obstacle pairs inside the influence region contribute;
            # phi, beta and their derivatives vanish identically for rho > rho0.
            rho_all = self.geometry.clearance(p)
            # D005 / T11b: outside the free set evaluate at the clamped clearance instead
            # of raising, so one contact cannot abort a Monte-Carlo batch.
            np.maximum(rho_all, c.contact_eps, out=rho_all)
            ni, mk = np.nonzero(rho_all <= c.rho0)
            if ni.size:
                rho = rho_all[ni, mk]
                d = delta[ni]
                gi = g[ni]
                normal = self._normals(p[ni], mk)
                inv_rho = 1.0 / rho
                z = inv_rho - 1.0 / c.rho0
                # eqs. (5)-(7), main.tex: phi and its first two derivatives.
                z_m2 = z ** (c.m - 2)
                z_m1 = z_m2 * z
                phi = c.eta / c.m * z_m1 * z
                phi1 = -c.eta * z_m1 * inv_rho**2
                phi2 = c.eta * ((c.m - 1) * z_m2 * inv_rho**4 + 2 * z_m1 * inv_rho**3)
                h = gi**c.n
                dh = c.n * gi ** (c.n - 2)
                grad_h = dh[:, None] * d
                potential += float(np.dot(phi, h))  # eq. (6), main.tex
                grad_phi = phi1[:, None] * normal
                _scatter_add(force, ni, -(h[:, None] * grad_phi + phi[:, None] * grad_h))

                # Hessian of rho, derived from eq. (2), main.tex: (P - n n^T) / ||p - o||,
                # with P = I (sphere) or diag(1, 1, 0) (infinite vertical cylinder).
                nn = normal[:, :, None] * normal[:, None, :]
                hess_rho = (self.projectors[mk] - nn) / (rho + self.radii[mk])[:, None, None]
                # eq. (12), main.tex: product-rule Hessian of phi(rho) * g^n.
                q_hess = (h * phi2)[:, None, None] * nn + (h * phi1)[:, None, None] * hess_rho
                q_hess += (phi * dh)[:, None, None] * _EYE
                if c.n > 2:
                    coef = np.zeros_like(gi)
                    np.power(gi, c.n - 4, out=coef, where=gi > 0)
                    coef *= phi * c.n * (c.n - 2)
                    q_hess += coef[:, None, None] * (d[:, :, None] * d[:, None, :])
                cross = grad_phi[:, :, None] * grad_h[:, None, :]
                q_hess += cross + cross.transpose(0, 2, 1)
                # [Q]_+ v = U max(w, 0) U^T v without forming [Q]_+ (Lemma psd).
                w, u = np.linalg.eigh(q_hess)
                y = np.einsum("kji,kj->ki", u, v[ni]) * np.maximum(w, 0.0)
                _scatter_add(damping, ni, np.einsum("kij,kj->ki", u, y))

                # eq. (13), main.tex: smoothstep activation and work-free gyro force.
                s = np.clip((c.rho0 - rho) / (c.rho0 - c.rho1), 0.0, 1.0)
                beta = s * s * (3.0 - 2.0 * s)
                term = _cross(-d, normal)
                term[:, 2] += c.mu
                _scatter_add(omega, ni, (c.kg * beta)[:, None] * term)

        if self.ii.size:
            x_all = p[self.ii] - p[self.jj]
            r_all = np.sqrt(np.einsum("ij,ij->i", x_all, x_all))
            # T11c: Morse acts on graph edges, the barrier only below d_s; other pairs
            # contribute exactly zero to every term.
            sel = np.flatnonzero(self._edge_bool | (r_all <= c.d_s))
            if sel.size:
                ii, jj = self.ii[sel], self.jj[sel]
                x, r_geo = x_all[sel], r_all[sel]
                unit = np.empty_like(x)
                unit[:] = (1.0, 0.0, 0.0)  # direction for coincident agents (r = 0)
                np.divide(x, r_geo[:, None], out=unit, where=r_geo[:, None] > 0)
                # D005 / T11b: evaluate pair terms at r >= d_min + contact_eps.
                r = np.maximum(r_geo, c.d_min + c.contact_eps)
                a_ij = self.edge_mask[sel]
                e = np.exp(-c.alpha * (r - self.desired_r[sel]))
                # eq. (8), main.tex: Morse energy and radial derivatives on graph edges.
                morse = a_ij * c.de * (1 - e) ** 2
                psi1 = a_ij * 2 * c.de * c.alpha * e * (1 - e)
                psi2 = a_ij * 2 * c.de * c.alpha**2 * (2 * e**2 - e)
                near = r <= c.d_s
                inv_gap = np.divide(1.0, r - c.d_min, out=np.zeros_like(r), where=near)
                z = np.where(near, inv_gap - 1.0 / (c.d_s - c.d_min), 0.0)
                # eq. (9), main.tex: barrier applies to every unordered pair.
                z_m2 = z ** (c.m - 2)
                z_m1 = z_m2 * z
                barrier = c.kb / c.m * z_m1 * z
                psi1 -= c.kb * z_m1 * inv_gap**2
                psi2 += c.kb * ((c.m - 1) * z_m2 * inv_gap**4 + 2 * z_m1 * inv_gap**3)
                potential += float(np.sum(morse + barrier))  # eq. (10), main.tex
                pair_force = -psi1[:, None] * unit
                _scatter_add(force, ii, pair_force)
                _scatter_add(force, jj, -pair_force)
                # eqs. (11)-(12), main.tex: PSD-projected radial/tangential curvature.
                dv = v[ii] - v[jj]
                radial = np.einsum("ij,ij->i", dv, unit)[:, None] * unit
                pair_damping = np.maximum(psi2, 0)[:, None] * radial
                pair_damping += np.maximum(psi1 / r, 0)[:, None] * (dv - radial)
                _scatter_add(damping, ii, pair_damping)
                _scatter_add(damping, jj, -pair_damping)

        gyroscopic = _cross(omega, v)  # eq. (13), main.tex
        return ControlComponents(potential, force, damping, gyroscopic, omega)

    def _normals(self, pts: FloatArray, mk: NDArray[np.intp]) -> FloatArray:
        """Outward unit normals n_ik = grad rho for the given (point, obstacle) pairs.

        Same convention as ``uavapf.geometry``: +e_z at a sphere centre, +e_x on a
        cylinder axis (never reached in the free set).
        """
        d = pts - self._centers[mk]
        d[self._planar[mk], 2] = 0.0
        dist = np.sqrt(np.einsum("ij,ij->i", d, d))
        out = np.empty_like(d)
        out[:] = (0.0, 0.0, 1.0)
        out[self._planar[mk] & (dist == 0)] = (1.0, 0.0, 0.0)
        np.divide(d, dist[:, None], out=out, where=dist[:, None] > 0)
        return out

    def potential(self, p: ArrayLike) -> float:
        """Total potential in m²/s² per unit mass at positions ``p`` in m."""
        return self.components(p, np.zeros_like(self.slots)).potential

    def __call__(self, t: float, p: ArrayLike, v: ArrayLike) -> FloatArray:
        """Compute acceleration commands in m/s² at time ``t`` in s."""
        terms = self.components(p, v)
        # eq. (15), main.tex
        return (
            terms.force
            - self.config.kd * np.asarray(v)
            - self.config.kr * terms.damping
            + terms.gyroscopic
        )
