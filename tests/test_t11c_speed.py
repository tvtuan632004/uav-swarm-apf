"""T11c: the sparse (active-pairs-only) controller equals the dense T06/T11b one.

The dense reference below is a verbatim copy of `APFController.components` as reviewed
in T06 and T11b (before the speed-up), rebuilt only from the controller's public
inputs (slots, edges, obstacles, config). Spec: docs/specs/T11c-speed.md.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from uavapf.control import APFController
from uavapf.geometry import Cylinder, Sphere


def dense_components(ctrl: APFController, p, v):
    """Reference: every agent-obstacle and agent-agent pair, no sparsity."""
    q = ctrl.slots
    ii, jj = np.triu_indices(len(q), k=1)
    desired_r = np.linalg.norm(q[ii] - q[jj], axis=1)
    edge_set = set(ctrl.edges)
    edge_mask = np.array([(int(i), int(j)) in edge_set for i, j in zip(ii, jj, strict=True)], float)
    radii = np.array([ob.radius for ob in ctrl.obstacles], dtype=np.float64)
    projectors = np.array(
        [
            np.eye(3) if isinstance(ob, Sphere) else np.diag([1.0, 1.0, 0.0])
            for ob in ctrl.obstacles
        ],
        dtype=np.float64,
    ).reshape(-1, 3, 3)
    p, v = ctrl.validate_state(p, v)
    c = ctrl.config
    delta = p - ctrl.slots
    g = np.linalg.norm(delta, axis=1)
    # eq. (4), main.tex
    potential = float(0.5 * c.ka * np.sum(delta * delta))
    force = -c.ka * delta
    damping = c.ka * v.copy()  # eq. (12), main.tex
    omega = np.zeros_like(p)

    if len(ctrl.obstacles):
        rho, normal = ctrl.geometry.distance(p)
        # D005 / T11b: outside the free set evaluate at the clamped clearance instead
        # of raising, so one contact cannot abort a Monte-Carlo batch.
        rho = np.maximum(rho, c.contact_eps)
        active = rho <= c.rho0
        inv_rho = np.divide(1.0, rho, out=np.zeros_like(rho), where=active)
        z = np.where(active, inv_rho - 1.0 / c.rho0, 0.0)
        # eqs. (5)-(7), main.tex: phi and its first two derivatives.
        phi = c.eta / c.m * z**c.m
        phi1 = -c.eta * z ** (c.m - 1) * inv_rho**2
        phi2 = c.eta * ((c.m - 1) * z ** (c.m - 2) * inv_rho**4 + 2 * z ** (c.m - 1) * inv_rho**3)
        h = g**c.n
        grad_h = c.n * g[:, None] ** (c.n - 2) * delta
        eye = np.eye(3)
        hess_h = c.n * g[:, None, None] ** (c.n - 2) * eye
        if c.n > 2:
            power = np.zeros_like(g)
            np.power(g, c.n - 4, out=power, where=g > 0)
            hess_h += (c.n * (c.n - 2) * power)[:, None, None] * (
                delta[:, :, None] * delta[:, None, :]
            )
        potential += float(np.sum(phi * h[:, None]))  # eq. (6), main.tex
        grad_phi = phi1[:, :, None] * normal
        force -= np.sum(h[:, None, None] * grad_phi + phi[:, :, None] * grad_h[:, None], axis=1)

        # Hessian of rho, derived from eq. (2), main.tex: (P - n n^T) / ||p - o||, with
        # P = I (sphere) or diag(1, 1, 0) (infinite vertical cylinder).
        nn = normal[:, :, :, None] * normal[:, :, None, :]
        hess_rho = (projectors[None] - nn) / (rho + radii)[..., None, None]
        # eq. (12), main.tex: full product-rule Hessian of phi(rho)*g^n.
        hess_phi = phi2[:, :, None, None] * nn + phi1[:, :, None, None] * hess_rho
        cross = grad_phi[:, :, :, None] * grad_h[:, None, None, :]
        q_hess = (
            h[:, None, None, None] * hess_phi
            + phi[:, :, None, None] * hess_h[:, None]
            + cross
            + cross.swapaxes(-1, -2)
        )
        eigval, eigvec = np.linalg.eigh(q_hess)
        projected = np.einsum("nmik,nmk,nmjk->nmij", eigvec, np.maximum(eigval, 0.0), eigvec)
        damping += np.einsum("nmij,nj->ni", projected, v)

        # eq. (13), main.tex: smoothstep activation and work-free gyro force.
        s = np.clip((c.rho0 - rho) / (c.rho0 - c.rho1), 0.0, 1.0)
        beta = s * s * (3.0 - 2.0 * s)
        axis = np.array([0.0, 0.0, 1.0])
        omega = c.kg * np.sum(
            beta[:, :, None] * (np.cross(-delta[:, None, :], normal) + c.mu * axis),
            axis=1,
        )

    if ii.size:
        x = p[ii] - p[jj]
        r_geo = np.linalg.norm(x, axis=1)
        unit = np.empty_like(x)
        unit[:] = (1.0, 0.0, 0.0)  # direction for coincident agents (r = 0)
        np.divide(x, r_geo[:, None], out=unit, where=r_geo[:, None] > 0)
        # D005 / T11b: evaluate pair terms at r >= d_min + contact_eps.
        r = np.maximum(r_geo, c.d_min + c.contact_eps)
        e = np.exp(-c.alpha * (r - desired_r))
        # eq. (8), main.tex: Morse energy and radial derivatives on graph edges.
        morse = edge_mask * c.de * (1 - e) ** 2
        psi1 = edge_mask * 2 * c.de * c.alpha * e * (1 - e)
        psi2 = edge_mask * 2 * c.de * c.alpha**2 * (2 * e**2 - e)
        near = r <= c.d_s
        inv_gap = np.divide(1.0, r - c.d_min, out=np.zeros_like(r), where=near)
        z = np.where(near, inv_gap - 1.0 / (c.d_s - c.d_min), 0.0)
        # eq. (9), main.tex: barrier applies to every unordered pair.
        barrier = c.kb / c.m * z**c.m
        psi1 -= c.kb * z ** (c.m - 1) * inv_gap**2
        psi2 += c.kb * ((c.m - 1) * z ** (c.m - 2) * inv_gap**4 + 2 * z ** (c.m - 1) * inv_gap**3)
        potential += float(np.sum(morse + barrier))  # eq. (10), main.tex
        pair_force = -psi1[:, None] * unit
        np.add.at(force, ii, pair_force)
        np.add.at(force, jj, -pair_force)
        # eqs. (11)-(12), main.tex: PSD-projected radial/tangential curvature.
        dv = v[ii] - v[jj]
        radial = np.sum(dv * unit, axis=1)[:, None] * unit
        pair_damping = np.maximum(psi2, 0)[:, None] * radial
        pair_damping += np.maximum(psi1 / r, 0)[:, None] * (dv - radial)
        np.add.at(damping, ii, pair_damping)
        np.add.at(damping, jj, -pair_damping)

    gyroscopic = np.cross(omega, v)  # eq. (13), main.tex
    return potential, force, damping, gyroscopic, omega


def swarm(n_agents: int, n_obs: int, seed: int, spread: float = 0.25):
    """Grid formation with obstacles scattered between the slots (many active pairs)."""
    rng = np.random.default_rng(seed)
    side = int(np.ceil(np.sqrt(n_agents)))
    slots = np.array([[2.0 * (i % side), 2.0 * (i // side), 5.0] for i in range(n_agents)])
    obstacles = []
    while len(obstacles) < n_obs:
        c = rng.uniform(-2.0, 2.0 * side, size=2)
        if np.min(np.linalg.norm(slots[:, :2] - c, axis=1)) < 1.4:
            continue
        if len(obstacles) % 2:
            obstacles.append(Sphere((c[0], c[1], 5.0 + rng.uniform(-1, 1)), 0.5))
        else:
            obstacles.append(Cylinder((c[0], c[1]), 0.5))
    edges = [(i, i + 1) for i in range(n_agents - 1)]
    if n_agents >= 3:
        edges.append((0, n_agents - 1))
    ctrl = APFController(slots, edges, obstacles)
    p = slots + rng.normal(scale=spread, size=slots.shape)
    v = rng.normal(size=slots.shape)
    return ctrl, p, v


@pytest.mark.parametrize(
    ("n", "m", "seed"), [(1, 3, 0), (4, 0, 1), (12, 6, 2), (30, 15, 3), (50, 20, 4)]
)
def test_sparse_equals_dense(n, m, seed):
    ctrl, p, v = swarm(n, m, seed)
    for k in range(4):  # several states, from tight (contact-like) to loose
        pk = ctrl.slots + (p - ctrl.slots) * (0.5 + k)
        ref = dense_components(ctrl, pk, v)
        got = ctrl.components(pk, v)
        assert got.potential == pytest.approx(ref[0], rel=1e-12, abs=1e-12)
        for name, want in zip(("force", "damping", "gyroscopic", "omega"), ref[1:], strict=True):
            have = getattr(got, name)
            scale = max(1.0, float(np.abs(want).max()))
            np.testing.assert_allclose(have, want, rtol=1e-11, atol=1e-12 * scale, err_msg=name)


def test_sparse_equals_dense_outside_free_set():
    """Clamped (contact) states are handled identically too (T11b)."""
    ctrl, p, v = swarm(6, 4, 9)
    p = p.copy()
    p[1] = p[0] + [0.2, 0.0, 0.0]  # pair below d_min
    ob = ctrl.obstacles[0]
    centre = np.array(ob.center) if isinstance(ob, Sphere) else np.array([*ob.center_xy, 5.0])
    p[2] = centre + [0.1, 0.0, 0.0]  # inside an obstacle
    ref = dense_components(ctrl, p, v)
    got = ctrl.components(p, v)
    for name, want in zip(("force", "damping", "gyroscopic", "omega"), ref[1:], strict=True):
        scale = max(1.0, float(np.abs(want).max()))
        np.testing.assert_allclose(getattr(got, name), want, rtol=1e-11, atol=1e-12 * scale)


def test_edges_are_public_and_normalised():
    ctrl = APFController(np.array([[0.0, 0, 0], [2.0, 0, 0], [4.0, 0, 0]]), [(1, 0), (2, 1)], [])
    assert ctrl.edges == ((0, 1), (1, 2))


def test_faster_than_dense():
    """Loose guard against regressions; the absolute target is in scripts/bench_controller.py."""
    ctrl, p, v = swarm(50, 20, 4)
    for _ in range(3):
        ctrl(0.0, p, v)
        dense_components(ctrl, p, v)

    def best(fn, reps=30):
        times = []
        for _ in range(reps):
            t0 = time.perf_counter()
            fn()
            times.append(time.perf_counter() - t0)
        return min(times)

    fast = best(lambda: ctrl(0.0, p, v))
    slow = best(lambda: dense_components(ctrl, p, v))
    assert fast < 0.6 * slow
