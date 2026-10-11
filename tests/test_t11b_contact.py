"""T11b acceptance tests: contact is an outcome (D005). Spec: docs/specs/T11b-contact.md."""

from __future__ import annotations

import numpy as np
import pytest

# Skipped until Codex creates the module (T11b); "done" requires 0 skipped tests.
pytest.importorskip("uavapf.metrics.safety", reason="T11b not implemented yet")

from uavapf.control import APFConfig, APFController  # noqa: E402
from uavapf.geometry import Cylinder, ObstacleSet, Sphere  # noqa: E402
from uavapf.metrics import collision_mask, collision_stop, min_clearance, min_separation  # noqa: E402
from uavapf.sim import SimConfig, simulate  # noqa: E402

CFG = APFConfig()
EPS = CFG.contact_eps


@pytest.fixture
def rng():
    return np.random.default_rng(1111)


# --------------------------------------------------------------------------- #
# Controller stays finite outside the free set (criterion 1)
# --------------------------------------------------------------------------- #
def test_contact_eps_default_and_validation():
    assert EPS == pytest.approx(1e-3)
    for bad in (0.0, -1e-3, CFG.rho1, CFG.d_s - CFG.d_min):
        with pytest.raises(ValueError):
            APFConfig(contact_eps=bad)


@pytest.mark.parametrize("rho", [0.5 * EPS, 0.0, -0.3])
def test_finite_when_touching_or_inside_obstacle(rho):
    sphere = Sphere((0.0, 0.0, 2.0), 0.8)
    slots = np.array([[5.0, 0.0, 2.0]])
    c = APFController(slots, [], [sphere])
    p = np.array([[0.8 + rho, 0.0, 2.0]])  # on the +x side, normal +e_x
    v = np.array([[-0.5, 0.2, 0.0]])
    comp = c.components(p, v)
    for arr in (comp.force, comp.damping, comp.gyroscopic, comp.omega):
        assert np.all(np.isfinite(arr))
    assert np.isfinite(comp.potential)
    assert np.all(np.isfinite(c(0.0, p, v)))
    assert comp.force[0, 0] > 0  # still pushed out of the obstacle


@pytest.mark.parametrize("gap", [0.0, -0.1, -CFG.d_min])  # r = d_min + gap, last one r = 0
def test_finite_when_pair_at_or_below_d_min(gap):
    slots = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    c = APFController(slots, [(0, 1)], [])
    r = CFG.d_min + gap
    p = np.array([[0.0, 0.0, 0.0], [r, 0.0, 0.0]])
    v = np.array([[0.1, 0.0, 0.0], [-0.1, 0.0, 0.0]])
    comp = c.components(p, v)
    assert np.all(np.isfinite(comp.force)) and np.all(np.isfinite(comp.damping))
    assert np.isfinite(comp.potential)
    scale = np.abs(comp.force).max()  # pair forces are ~1e11 here: compare relative to them
    np.testing.assert_allclose(
        comp.force.sum(axis=0), -CFG.ka * (p - slots).sum(axis=0), atol=1e-12 * scale
    )
    if r > 0:
        assert comp.force[1, 0] > 0 > comp.force[0, 0]  # pushed apart


def test_shape_and_nan_checks_still_raise():
    c = APFController([[0.0, 0.0, 0.0]], [], [])
    with pytest.raises(ValueError):
        c(0.0, np.zeros((2, 3)), np.zeros((2, 3)))
    with pytest.raises(ValueError):
        c(0.0, [[np.nan, 0.0, 0.0]], [[0.0, 0.0, 0.0]])


def test_construction_checks_stay_strict():
    with pytest.raises(ValueError):
        APFController([[0.0, 0.0, 2.0]], [], [Sphere((0.0, 0.0, 2.0), 0.5)])
    with pytest.raises(ValueError):
        APFController([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [], [])


# --------------------------------------------------------------------------- #
# Unchanged inside the free set (criterion 2)
# --------------------------------------------------------------------------- #
def test_output_unchanged_away_from_contact(rng):
    """Same controller with a much smaller eps gives identical output in F."""
    slots = np.array([[0.0, 0.0, 2.0], [2.0, 0.0, 2.0], [0.0, 2.0, 2.0]])
    obs = [Sphere((-1.4, 0.0, 2.0), 0.8), Cylinder((3.0, 3.0), 0.5)]
    a = APFController(slots, [(0, 1), (1, 2)], obs)
    b = APFController(slots, [(0, 1), (1, 2)], obs, APFConfig(contact_eps=1e-9))
    geo = ObstacleSet(obs)
    checked = 0
    while checked < 30:
        p = slots + rng.uniform(-0.6, 0.6, size=slots.shape)
        if geo.distance(p)[0].min() < 2 * EPS or min_separation(p) < CFG.d_min + 2 * EPS:
            continue
        v = rng.normal(size=p.shape)
        np.testing.assert_array_equal(a(0.0, p, v), b(0.0, p, v))
        checked += 1


# --------------------------------------------------------------------------- #
# Closed loop with saturation (criterion 3, the S2 reproduction)
# --------------------------------------------------------------------------- #
def head_on():
    slots = np.array([[5.0, 0.0, 2.0], [-5.0, 0.0, 2.0]])
    p0 = np.array([[-5.0, 0.001, 2.0], [5.0, 0.0, 2.0]])
    return APFController(slots, [], []), p0


def test_saturated_head_on_swap_runs_to_the_end():
    c, p0 = head_on()
    traj = simulate(c, p0, np.zeros_like(p0), SimConfig(dt=0.01, t_final=20.0, a_max=5.0))
    assert traj.t[-1] == pytest.approx(20.0)
    assert np.all(np.isfinite(traj.p))
    assert min_separation(traj.p) <= CFG.d_min  # the collision is reported, not raised


def test_collision_stop_ends_run_at_first_contact():
    c, p0 = head_on()
    stop = collision_stop(ObstacleSet([]), CFG.d_min)
    traj = simulate(
        c, p0, np.zeros_like(p0), SimConfig(dt=0.01, t_final=20.0, a_max=5.0), stop=stop
    )
    assert traj.t[-1] < 20.0
    hits = collision_mask(traj.p, ObstacleSet([]), CFG.d_min)
    assert hits[-1] and not hits[:-1].any()


# --------------------------------------------------------------------------- #
# Safety metrics (criterion 4)
# --------------------------------------------------------------------------- #
def brute(p, obs_list, d_min):
    """Loop reference: (min clearance, min separation, collision per step)."""
    geo = ObstacleSet(obs_list)
    clear, sep, coll = np.inf, np.inf, []
    for state in p:
        hit = False
        for i in range(len(state)):
            for ob in range(len(obs_list)):
                rho = geo.distance(state[i : i + 1])[0][0, ob]
                clear = min(clear, rho)
                hit |= rho <= 0
            for j in range(i + 1, len(state)):
                r = float(np.linalg.norm(state[i] - state[j]))
                sep = min(sep, r)
                hit |= r <= d_min
        coll.append(hit)
    return clear, sep, np.array(coll)


def test_metrics_match_brute_force(rng):
    obs_list = [Sphere((0.0, 0.0, 0.0), 1.0), Cylinder((3.0, 0.0), 0.5, -1.0, 4.0)]
    geo = ObstacleSet(obs_list)
    p = rng.uniform(-3, 4, size=(25, 6, 3))
    clear, sep, coll = brute(p, obs_list, 0.5)
    assert min_clearance(p, geo) == pytest.approx(clear, rel=1e-12)
    assert min_separation(p) == pytest.approx(sep, rel=1e-12)
    np.testing.assert_array_equal(collision_mask(p, geo, 0.5), coll)
    # single state (N, 3) gives the same as a length-1 trajectory
    assert min_clearance(p[3], geo) == pytest.approx(min_clearance(p[3:4], geo))
    assert min_separation(p[3]) == pytest.approx(min_separation(p[3:4]))
    assert bool(collision_mask(p[3], geo, 0.5)) == coll[3]


def test_metric_edge_cases():
    p = np.array([[0.0, 0.0, 0.0]])
    assert min_clearance(p, ObstacleSet([])) == np.inf
    assert min_separation(p) == np.inf
    assert not bool(collision_mask(p, ObstacleSet([]), 0.5))
    stop = collision_stop(ObstacleSet([Sphere((0.0, 0.0, 0.0), 1.0)]), 0.5)
    assert bool(stop(0.0, p, np.zeros_like(p)))  # p is inside the sphere
    assert not stop(0.0, np.array([[5.0, 0.0, 0.0]]), np.zeros((1, 3)))


def test_boundary_counts_as_collision():
    geo = ObstacleSet([Sphere((0.0, 0.0, 0.0), 1.0)])
    on_surface = np.array([[1.0, 0.0, 0.0]])
    assert bool(collision_mask(on_surface, geo, 0.5))  # rho = 0 is not in F
    at_d_min = np.array([[5.0, 0.0, 0.0], [5.5, 0.0, 0.0]])
    assert bool(collision_mask(at_d_min, ObstacleSet([]), 0.5))  # r = d_min is not in F
