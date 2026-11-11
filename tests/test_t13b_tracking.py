"""T13b acceptance tests: moving formation centre (main.tex Remark "Moving formation").

Spec: docs/specs/T13b-tracking.md. Skipped until uavapf.control.reference exists.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

_ref = pytest.importorskip("uavapf.control.reference", reason="T13b not implemented yet")
StraightPath = _ref.StraightPath

from uavapf.baselines import KhatibController  # noqa: E402
from uavapf.control import (  # noqa: E402
    APFConfig,
    APFController,
    ResetConfig,
    StallReset,
    TrackingController,
)
from uavapf.geometry import Cylinder, ObstacleSet, Sphere  # noqa: E402
from uavapf.metrics import evaluate_run  # noqa: E402
from uavapf.scenarios import (  # noqa: E402
    GeneratorParams,
    make_scenario,
    scenario_from_dict,
    scenario_to_dict,
)
from uavapf.sim import SimConfig, Trajectory, simulate  # noqa: E402

CFG = APFConfig()
SLOTS = np.array([[20.0, -2.0, 5.0], [20.0, 0.0, 5.0], [20.0, 2.0, 5.0]])
EDGES = [(0, 1), (1, 2)]
GOAL = tuple(SLOTS.mean(axis=0))
PATH = StraightPath(start=(0.0, 0.0, 5.0), goal=GOAL, v_ref=1.5, a_ref=0.5)


# --------------------------------------------------------------------------- #
# 1. Reference path
# --------------------------------------------------------------------------- #
def test_duration_rule():
    L = 20.0
    expected = max(1.875 * L / 1.5, math.sqrt(10 / math.sqrt(3) * L / 0.5))
    assert PATH.duration == pytest.approx(expected)
    still = StraightPath((1.0, 2.0, 3.0), (1.0, 2.0, 3.0), 1.0, 1.0)
    assert still.duration == 0.0
    np.testing.assert_array_equal(still.position(5.0), [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(still.velocity(5.0), [0.0, 0.0, 0.0])


def test_endpoints_limits_and_derivatives():
    T = PATH.duration
    t = np.linspace(-1.0, T + 1.0, 4001)
    c, cd, cdd = PATH.position(t), PATH.velocity(t), PATH.acceleration(t)
    assert c.shape == cd.shape == cdd.shape == (t.size, 3)
    np.testing.assert_allclose(PATH.position(0.0), [0.0, 0.0, 5.0], atol=1e-12)
    np.testing.assert_allclose(PATH.position(T), GOAL, atol=1e-12)
    np.testing.assert_allclose(PATH.velocity(0.0), 0.0, atol=1e-12)
    np.testing.assert_allclose(PATH.velocity(T), 0.0, atol=1e-12)
    assert np.linalg.norm(cd, axis=1).max() <= 1.5 * (1 + 1e-9)
    assert np.linalg.norm(cdd, axis=1).max() <= 0.5 * (1 + 1e-6)
    before, after = t < 0, t > T
    np.testing.assert_allclose(c[before], np.broadcast_to([0.0, 0.0, 5.0], c[before].shape))
    np.testing.assert_allclose(c[after], np.broadcast_to(GOAL, c[after].shape))
    np.testing.assert_allclose(cd[after], 0.0, atol=1e-12)
    np.testing.assert_allclose(cdd[after], 0.0, atol=1e-12)
    h = 1e-5
    tt = np.linspace(0.1, T - 0.1, 50)
    np.testing.assert_allclose(
        PATH.velocity(tt), (PATH.position(tt + h) - PATH.position(tt - h)) / (2 * h), atol=1e-7
    )
    np.testing.assert_allclose(
        PATH.acceleration(tt),
        (PATH.velocity(tt + h) - PATH.velocity(tt - h)) / (2 * h),
        atol=1e-7,
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"start": (0, 0, 0), "goal": (1, 0, 0), "v_ref": 0.0, "a_ref": 1.0},
        {"start": (0, 0, 0), "goal": (1, 0, 0), "v_ref": 1.0, "a_ref": -1.0},
        {"start": (0, 0, math.nan), "goal": (1, 0, 0), "v_ref": 1.0, "a_ref": 1.0},
    ],
)
def test_path_validation(kwargs):
    with pytest.raises(ValueError):
        StraightPath(**kwargs)


# --------------------------------------------------------------------------- #
# 2-3. Controllers
# --------------------------------------------------------------------------- #
OBS = [Sphere((10.0, 0.5, 5.0), 0.8), Cylinder((14.0, -3.0), 0.5)]


def random_states(rng, n=10):
    out = []
    geo = ObstacleSet(OBS)
    while len(out) < n:
        c = PATH.position(rng.uniform(0, PATH.duration))
        p = c + (SLOTS - np.asarray(GOAL)) + rng.normal(scale=0.5, size=SLOTS.shape)
        if geo.clearance(p).min() < 0.2:
            continue
        r = np.linalg.norm(p[:, None] - p[None], axis=-1)[np.triu_indices(3, 1)]
        if r.min() < CFG.d_min + 0.2:
            continue
        out.append((p, rng.normal(size=p.shape), float(rng.uniform(0, PATH.duration))))
    return out


def test_static_path_equals_apf():
    still = StraightPath(GOAL, GOAL, 1.0, 1.0)
    trk = TrackingController(SLOTS, EDGES, OBS, still)
    apf = APFController(SLOTS, EDGES, OBS)
    for p, v, t in random_states(np.random.default_rng(1)):
        np.testing.assert_array_equal(trk(t, p, v), apf(t, p, v))


def test_remark2_assembly():
    trk = TrackingController(SLOTS, EDGES, OBS, PATH)
    delta = SLOTS - np.asarray(GOAL)
    for p, v, t in random_states(np.random.default_rng(2)):
        slots_t = PATH.position(t) + delta
        np.testing.assert_allclose(trk.slots_at(t), slots_t, atol=1e-12)
        np.testing.assert_allclose(
            trk.slot_velocity(t), np.broadcast_to(PATH.velocity(t), (3, 3)), atol=1e-12
        )
        e = v - PATH.velocity(t)
        comp = trk.base.components(p, e, slots=slots_t)
        want = (
            PATH.acceleration(t)
            + comp.force
            - CFG.kd * e
            - CFG.kr * comp.damping
            + np.cross(comp.omega, e)
        )
        np.testing.assert_allclose(trk(t, p, v), want, rtol=1e-12, atol=1e-12)


def test_components_slots_override():
    apf = APFController(SLOTS, EDGES, OBS)
    rng = np.random.default_rng(3)
    for p, v, _ in random_states(rng, 5):
        a = apf.components(p, v)
        b = apf.components(p, v, slots=SLOTS)
        for name in ("force", "damping", "gyroscopic", "omega"):
            np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
        assert a.potential == b.potential
    np.testing.assert_array_equal(apf.slots_at(3.0), SLOTS)
    np.testing.assert_array_equal(apf.slot_velocity(3.0), np.zeros((3, 3)))
    # D010: shifting every slot by d changes the force by exactly ka * d, even with
    # obstacles present, because the T1 goal factor stays on the final slots
    shift = np.array([1.0, -2.0, 0.5])
    for p, _, _ in random_states(rng, 5):
        z = np.zeros_like(p)
        df = apf.components(p, z, slots=SLOTS + shift).force - apf.components(p, z).force
        np.testing.assert_allclose(df, np.broadcast_to(CFG.ka * shift, df.shape), atol=1e-9)


def test_reference_through_obstacle_is_avoided():
    """The failure behind D010: the slot moves through a sphere, the agent must not."""
    slot = np.array([[24.0, 0.0, 5.0]])
    path = StraightPath((0.0, 0.0, 5.0), (24.0, 0.0, 5.0), 1.5, 0.5)
    sphere = Sphere((12.0, 0.3, 5.0), 1.0)
    trk = TrackingController(slot, [], [sphere], path)
    reset = StallReset(trk)
    traj = simulate(
        trk,
        trk.slots_at(0.0),
        trk.slot_velocity(0.0),
        SimConfig(dt=0.01, t_final=path.duration + 20.0, a_max=5.0),
        reset=reset,
    )
    rho = ObstacleSet([sphere]).clearance(traj.p.reshape(-1, 3))
    assert rho.min() > 0.0
    assert np.linalg.norm(traj.p[-1, 0] - slot[0]) < 0.3


@pytest.mark.parametrize("edges", [[], EDGES])
def test_exact_tracking_in_free_space(edges):
    trk = TrackingController(SLOTS, edges, [], PATH)
    p0 = trk.slots_at(0.0)
    v0 = trk.slot_velocity(0.0)
    traj = simulate(trk, p0, v0, SimConfig(dt=0.01, t_final=PATH.duration + 2.0))
    ref = np.stack([trk.slots_at(t) for t in traj.t])
    assert np.abs(traj.p - ref).max() < 1e-8
    np.testing.assert_allclose(traj.p[-1], SLOTS, atol=1e-8)


def test_khatib_tracking_in_free_space():
    b1 = KhatibController(SLOTS, [], path=PATH)
    traj = simulate(b1, b1.slots_at(0.0), b1.slot_velocity(0.0), SimConfig(0.01, PATH.duration))
    ref = np.stack([b1.slots_at(t) for t in traj.t])
    assert np.abs(traj.p - ref).max() < 1e-8


# --------------------------------------------------------------------------- #
# 6. Reset with a moving slot
# --------------------------------------------------------------------------- #
def test_reset_quiet_while_riding_the_reference():
    trk = TrackingController(SLOTS, EDGES, [], PATH)
    reset = StallReset(trk)
    for k in range(0, int(PATH.duration * 100) + 300, 5):
        t = k * 0.01
        v = trk.slot_velocity(t)
        p = trk.slots_at(t) + [0.2, 0.0, 0.0]  # within eps_p of the moving slot
        np.testing.assert_array_equal(reset(t, p, v), v)
    assert reset.count.sum() == 0


def test_reset_uses_moving_slot_and_absolute_speed():
    trk = TrackingController(SLOTS, EDGES, [], PATH)
    cfg = ResetConfig(stall_time=0.5)
    t0 = 0.4 * PATH.duration  # reference at cruise speed
    # (a) moving WITH the reference, 3 m behind: not stalled (it is moving)
    a = StallReset(trk, cfg)
    for k in range(200):
        t = t0 + k * 0.01
        v = trk.slot_velocity(t)
        np.testing.assert_array_equal(a(t, trk.slots_at(t) - [3.0, 0, 0], v), v)
    # (b) standing still while the slot moves away: stalled -> one kick after stall_time
    b = StallReset(trk, cfg)
    p = trk.slots_at(t0) - [3.0, 0.0, 0.0]
    fired = [
        t0 + k * 0.01 for k in range(200) if np.any(b(t0 + k * 0.01, p, np.zeros((3, 3))) != 0)
    ]
    assert fired and fired[0] == pytest.approx(t0 + 0.5)
    assert b.count.min() >= 1


# --------------------------------------------------------------------------- #
# 7. Scenario YAML and generator
# --------------------------------------------------------------------------- #
def test_generator_sets_reference_except_s5():
    for sid in ("S1", "S2", "S3", "S4"):
        s = make_scenario(sid, 5, 0)
        assert isinstance(s.reference, StraightPath)
        np.testing.assert_allclose(s.reference.goal, s.slots.mean(axis=0), atol=1e-12)
        assert s.sim.t_final == pytest.approx(s.reference.duration + 20.0)
        assert isinstance(s.build_controller(), TrackingController)
        trk = s.build_controller()
        assert np.abs(trk.slots_at(0.0) - s.p0).max() < 5 * GeneratorParams().start_noise
    for var in ("groups", "swap"):
        s5 = make_scenario("S5", 6, 0, variant=var)
        assert s5.reference is None
        assert isinstance(s5.build_controller(), APFController)
    static = make_scenario(
        "S3", 5, 0, params=dataclasses.replace(GeneratorParams(), ref_speed=None)
    )
    assert static.reference is None


def test_yaml_round_trip_with_reference():
    s = make_scenario("S3", 5, 4)
    d = scenario_to_dict(s)
    assert set(d["reference"]) == {"start", "speed", "accel"}
    back = scenario_from_dict(d)
    assert back.reference == s.reference
    d_bad = dict(d)
    d_bad["reference"] = {**d["reference"], "speeed": 1.0}
    with pytest.raises(ValueError, match="speeed"):
        scenario_from_dict(d_bad)


# --------------------------------------------------------------------------- #
# 8. Metrics with moving slots
# --------------------------------------------------------------------------- #
def test_evaluate_run_with_time_varying_slots():
    rng = np.random.default_rng(8)
    k = 40
    t = np.linspace(0, 4, k)
    slots_t = np.stack([PATH.position(tt) + (SLOTS - np.asarray(GOAL)) for tt in t])
    p = slots_t + rng.normal(scale=0.3, size=slots_t.shape)
    traj = Trajectory(t=t, p=p, v=np.zeros_like(p), u=np.zeros_like(p))
    m = evaluate_run(traj, slots_t, ObstacleSet([]), d_min=0.5, eps_p=0.3)
    err = np.linalg.norm(p - slots_t, axis=2).mean(axis=1)
    ef = np.sum(0.5 * (err[1:] + err[:-1]) * np.diff(t)) / (t[-1] - t[0])
    assert m.formation_error == pytest.approx(ef, rel=1e-12)
    final = np.linalg.norm(p - slots_t[-1], axis=2)  # completion vs the final formation
    hit = np.flatnonzero(np.all(final <= 0.3, axis=1))
    assert m.completion_time == (t[hit[0]] if hit.size else math.inf)
