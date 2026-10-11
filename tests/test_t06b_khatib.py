"""T06 acceptance tests: baseline B1 (Khatib APF). Spec: docs/specs/T06-b1-khatib.md."""

from __future__ import annotations

import math

import numpy as np
import pytest

from uavapf.baselines import KhatibConfig, KhatibController
from uavapf.control import APFController, StallReset
from uavapf.geometry import Cylinder, ObstacleSet, Sphere
from uavapf.sim import SimConfig, simulate

CFG = KhatibConfig()


def ref_U(p, slots, obstacles, c: KhatibConfig = CFG) -> float:
    """Loop implementation of section 1 of the spec."""
    geo = ObstacleSet(obstacles)
    total = 0.0
    for i in range(len(p)):
        total += 0.5 * c.ka * float(np.sum((p[i] - slots[i]) ** 2))
        if obstacles:
            for rho in geo.distance(p[i : i + 1])[0][0]:
                if rho <= c.rho0:
                    total += 0.5 * c.eta * (1.0 / rho - 1.0 / c.rho0) ** 2
        for j in range(i + 1, len(p)):
            r = float(np.linalg.norm(p[i] - p[j]))
            if r <= c.d_s:
                total += 0.5 * c.kb * (1.0 / r - 1.0 / c.d_s) ** 2
    return total


def fd_grad(f, x, h=1e-6):
    g = np.zeros_like(x)
    for idx in np.ndindex(x.shape):
        e = np.zeros_like(x)
        e[idx] = h
        g[idx] = (f(x + e) - f(x - e)) / (2 * h)
    return g


SLOTS = np.array([[0.0, 0.0, 2.0], [2.0, 0.0, 2.0], [0.0, 2.0, 2.0]])
OBS = [Sphere((-1.4, 0.0, 2.0), 0.8), Cylinder((2.5, 2.5), 0.4, 0.0, 6.0)]


def test_force_is_minus_gradient_and_potential_matches():
    rng = np.random.default_rng(66)
    ctrl = KhatibController(SLOTS, OBS)
    checked = 0
    while checked < 15:
        p = SLOTS + rng.uniform(-0.7, 0.7, size=SLOTS.shape)
        rho = ObstacleSet(OBS).distance(p)[0]
        r = [np.linalg.norm(p[i] - p[j]) for i in range(3) for j in range(i + 1, 3)]
        if rho.min() < 0.15 or min(r) < 0.3 or np.any(np.abs(rho - CFG.rho0) < 1e-3):
            continue
        assert ctrl.potential(p) == pytest.approx(ref_U(p, SLOTS, OBS), rel=1e-12)
        u = ctrl(0.0, p, np.zeros_like(p))  # v = 0 -> u = -grad U
        np.testing.assert_allclose(
            u, -fd_grad(lambda q: ref_U(q, SLOTS, OBS), p), rtol=1e-6, atol=1e-6
        )
        checked += 1


def test_damping_term():
    ctrl = KhatibController(SLOTS, OBS)
    p = SLOTS + 0.1
    v = np.array([[0.3, 0.0, 0.0], [0.0, -0.2, 0.0], [0.0, 0.0, 0.1]])
    np.testing.assert_allclose(ctrl(0.0, p, v) - ctrl(0.0, p, np.zeros_like(v)), -CFG.kd * v)


def test_terms_vanish_outside_influence_and_pairs_cancel():
    slots = np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    far = KhatibController(slots, [Sphere((0.0, 10.0, 0.0), 1.0)])
    p = slots + [[0.2, 0.0, 0.0], [0.0, 0.1, 0.0]]
    np.testing.assert_allclose(far(0.0, p, np.zeros_like(p)), -CFG.ka * (p - slots))
    pair = KhatibController(slots, [])
    q = np.array([[0.0, 0.0, 0.0], [0.8, 0.0, 0.0]])
    u = pair(0.0, q, np.zeros_like(q))
    np.testing.assert_allclose(u.sum(axis=0), -CFG.ka * (q - slots).sum(axis=0), atol=1e-12)
    assert u[1, 0] + CFG.ka * (q[1, 0] - slots[1, 0]) > 0  # pushed apart


def settle(ctrl, p0, t_final, reset=None):
    return simulate(ctrl, p0, np.zeros_like(p0), SimConfig(dt=0.01, t_final=t_final), reset=reset)


def test_reproduces_gnron_and_apf_fixes_it():
    """Criterion 3: slot 0.6 m from the sphere surface, inside rho0 = 2 m."""
    sphere = Sphere((0.0, 0.0, 2.0), 0.8)
    slots = np.array([[1.4, 0.0, 2.0]])
    p0 = np.array([[3.0, 0.5, 2.0]])
    b1 = settle(KhatibController(slots, [sphere]), p0, 40.0)
    apf = settle(APFController(slots, [], [sphere]), p0, 40.0)
    err_b1 = float(np.linalg.norm(b1.p[-1, 0] - slots[0]))
    err_apf = float(np.linalg.norm(apf.p[-1, 0] - slots[0]))
    assert err_b1 >= 0.1, f"B1 should miss the slot (GNRON), error {err_b1:.3f} m"
    assert err_apf < 0.01, f"APF should reach the slot, error {err_apf:.3f} m"
    assert np.linalg.norm(b1.v[-1]) < 1e-3  # B1 really settled (an equilibrium, not transient)


def test_reproduces_local_minimum_and_apf_escapes():
    """Criterion 4: symmetric trap, slot directly behind the sphere."""
    sphere = Sphere((0.0, 0.0, 2.0), 0.8)
    slots = np.array([[3.0, 0.0, 2.0]])
    p0 = np.array([[-5.0, 0.0, 2.0]])
    b1 = settle(KhatibController(slots, [sphere]), p0, 30.0)
    assert b1.p[:, 0, 0].max() < 0.0  # never passes the obstacle centre plane
    assert np.linalg.norm(b1.v[-1]) < 1e-3  # stalled
    assert np.linalg.norm(b1.p[-1, 0] - slots[0]) > 3.0
    ctrl = APFController(slots, [], [sphere])
    apf = settle(ctrl, p0, 30.0, reset=StallReset(ctrl))
    assert np.linalg.norm(apf.p[-1, 0] - slots[0]) < 0.1


@pytest.mark.parametrize("pos", [[0.85, 0.0, 2.0], [0.5, 0.0, 2.0], [0.0, 0.0, 2.0]])
def test_finite_at_contact(pos):
    ctrl = KhatibController([[5.0, 0.0, 2.0]], [Sphere((0.0, 0.0, 2.0), 0.8)])
    u = ctrl(0.0, [pos], [[0.0, 0.0, 0.0]])
    assert np.all(np.isfinite(u))
    pair = KhatibController([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]], [])
    assert np.all(np.isfinite(pair(0.0, np.zeros((2, 3)), np.zeros((2, 3)))))
    assert math.isfinite(pair.potential(np.zeros((2, 3))))


@pytest.mark.parametrize(
    "make",
    [
        lambda: KhatibConfig(ka=0.0),
        lambda: KhatibConfig(rho0=-1.0),
        lambda: KhatibConfig(kd=math.nan),
        lambda: KhatibConfig(contact_eps=2.0),
        lambda: KhatibController([[0.0, 0.0, 0.0]], [Sphere((0.0, 0.0, 0.0), 1.0)]),
        lambda: KhatibController([[0.0, 0.0]], []),
    ],
)
def test_invalid(make):
    with pytest.raises(ValueError):
        make()


def test_capped_cylinders_allowed_and_shape_checked():
    ctrl = KhatibController([[0.0, 0.0, 10.0]], [Cylinder((0.0, 0.0), 1.0, 0.0, 5.0)])
    assert np.all(np.isfinite(ctrl(0.0, [[0.0, 0.0, 6.0]], [[0.0, 0.0, 0.0]])))
    with pytest.raises(ValueError):
        ctrl(0.0, np.zeros((2, 3)), np.zeros((2, 3)))
