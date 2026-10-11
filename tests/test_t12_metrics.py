"""T12 acceptance tests: per-run evaluation metrics. Spec: docs/specs/T12-metrics.md."""

from __future__ import annotations

import math

import numpy as np
import pytest

from uavapf.control import APFController
from uavapf.geometry import ObstacleSet, Sphere
from uavapf.metrics import RunMetrics, controller_cost, evaluate_run
from uavapf.sim import Trajectory

NO_OBS = ObstacleSet([])


def make_traj(t, p, u=None, v=None) -> Trajectory:
    t = np.asarray(t, dtype=float)
    p = np.asarray(p, dtype=float)
    u = np.zeros_like(p) if u is None else np.asarray(u, dtype=float)
    v = np.zeros_like(p) if v is None else np.asarray(v, dtype=float)
    return Trajectory(t=t, p=p, v=v, u=u)


def test_linear_approach_formation_error_and_constant_effort():
    t = np.linspace(0.0, 4.0, 41)
    slots = np.array([[10.0, 0.0, 0.0], [10.0, 5.0, 0.0]])
    d0 = np.array([[-6.0, 0.0, 0.0], [0.0, 0.0, 2.0]])  # |d0| = 6 and 2
    p = slots + (1 - t / 4.0)[:, None, None] * d0
    u = np.broadcast_to(np.array([[1.0, 2.0, 2.0], [0.0, 3.0, 4.0]]), p.shape)  # |u| = 3, 5
    m = evaluate_run(make_traj(t, p, u), slots, NO_OBS, d_min=0.5, eps_p=0.3)
    assert m.formation_error == pytest.approx((6.0 / 2 + 2.0 / 2) / 2, rel=1e-12)
    assert m.control_effort == pytest.approx((9.0 + 25.0) * 4.0, rel=1e-12)
    assert m.rms_jerk == pytest.approx(0.0, abs=1e-12)
    assert m.saturation_fraction == 0.0
    assert m.min_clearance == math.inf
    assert m.min_separation == pytest.approx(np.linalg.norm(p[:, 0] - p[:, 1], axis=1).min())


def test_ramp_command_gives_constant_jerk():
    a, T, n = 0.7, 3.0, 3
    t = np.linspace(0.0, T, 3001)
    slots = np.array([[0.0, 2.0 * i, 0.0] for i in range(n)])
    p = np.broadcast_to(slots, (len(t), n, 3))
    u = np.zeros((len(t), n, 3))
    u[:, :, 0] = a * t[:, None]
    m = evaluate_run(make_traj(t, p, u), slots, NO_OBS, d_min=0.5, eps_p=0.3)
    assert m.rms_jerk == pytest.approx(a, rel=1e-9)
    assert m.control_effort == pytest.approx(n * a**2 * T**3 / 3, rel=1e-5)
    assert m.formation_error == 0.0
    assert m.completion_time == 0.0 and m.success


def test_completion_time_and_success_with_t_max():
    t = np.arange(0.0, 5.01, 0.5)
    slots = np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    err0 = np.where(t >= 1.0, 0.1, 1.0)  # agent 0 arrives at t = 1
    err1 = np.where(t >= 2.0, 0.2, 1.0)  # agent 1 arrives at t = 2
    err1[t >= 4.0] = 1.0  # ... and leaves again later (does not matter)
    p = np.stack([slots[0] + err0[:, None] * [0, 1, 0], slots[1] + err1[:, None] * [0, 1, 0]], 1)
    traj = make_traj(t, p)
    m = evaluate_run(traj, slots, NO_OBS, d_min=0.5, eps_p=0.3)
    assert m.completion_time == pytest.approx(2.0)
    assert m.success
    assert not evaluate_run(traj, slots, NO_OBS, d_min=0.5, eps_p=0.3, t_max=1.5).success
    assert evaluate_run(traj, slots, NO_OBS, d_min=0.5, eps_p=0.3, t_max=2.0).success


def test_never_completes():
    t = np.linspace(0, 1, 11)
    slots = np.zeros((1, 3))
    p = np.ones((11, 1, 3))
    m = evaluate_run(make_traj(t, p), slots, NO_OBS, d_min=0.5, eps_p=0.3)
    assert m.completion_time == math.inf and not m.success


def test_collision_means_no_success():
    t = np.linspace(0, 2, 21)
    slots = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    p = np.broadcast_to(slots, (21, 2, 3)).copy()
    p[10, 1] = [0.4, 0.0, 0.0]  # one sample closer than d_min
    m = evaluate_run(make_traj(t, p), slots, NO_OBS, d_min=0.5, eps_p=0.3)
    assert m.collided and not m.success
    assert m.completion_time == 0.0
    obs = ObstacleSet([Sphere((10.0, 0.0, 0.0), 1.0)])
    p2 = np.broadcast_to(slots, (21, 2, 3)).copy()
    p2[5, 1] = [10.5, 0.0, 0.0]  # inside the sphere once
    m2 = evaluate_run(make_traj(t, p2), slots, obs, d_min=0.5, eps_p=0.3)
    assert m2.collided and m2.min_clearance == pytest.approx(-0.5)


def test_saturation_fraction_trapezoid():
    t = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
    slots = np.zeros((1, 3))
    p = np.zeros((5, 1, 3))
    u = np.zeros((5, 1, 3))
    u[:, 0, 0] = [2.0, 2.0, 2.0, 1.0, 1.0]  # a_max = 2 active on [0, 1]
    m = evaluate_run(make_traj(t, p, u), slots, NO_OBS, d_min=0.5, eps_p=0.3, a_max=2.0)
    # trapezoid of the indicator [1,1,1,0,0]: 0.5 + 0.5 + 0.25 + 0 = 1.25 over T = 2
    assert m.saturation_fraction == pytest.approx(0.625)


def test_matches_brute_force_on_random_trajectory():
    rng = np.random.default_rng(12)
    k, n = 60, 4
    t = np.cumsum(rng.uniform(0.01, 0.1, size=k))
    t -= t[0]
    slots = rng.uniform(-5, 5, size=(n, 3))
    p = slots + rng.normal(scale=1.0, size=(k, n, 3))
    u = rng.normal(size=(k, n, 3))
    obs_list = [Sphere((0.0, 0.0, 0.0), 0.5)]
    m = evaluate_run(make_traj(t, p, u), slots, ObstacleSet(obs_list), d_min=0.3, eps_p=0.5)
    T = t[-1] - t[0]
    ef = effort = jerk = 0.0
    for s in range(k - 1):
        dt = t[s + 1] - t[s]
        e0 = np.mean([np.linalg.norm(p[s, i] - slots[i]) for i in range(n)])
        e1 = np.mean([np.linalg.norm(p[s + 1, i] - slots[i]) for i in range(n)])
        ef += 0.5 * (e0 + e1) * dt
        f0 = sum(float(u[s, i] @ u[s, i]) for i in range(n))
        f1 = sum(float(u[s + 1, i] @ u[s + 1, i]) for i in range(n))
        effort += 0.5 * (f0 + f1) * dt
        jerk += sum(float(np.sum((u[s + 1, i] - u[s, i]) ** 2)) / dt for i in range(n))
    assert m.formation_error == pytest.approx(ef / T, rel=1e-12)
    assert m.control_effort == pytest.approx(effort, rel=1e-12)
    assert m.rms_jerk == pytest.approx(math.sqrt(jerk / (n * T)), rel=1e-12)
    clear = min(np.linalg.norm(p[s, i]) - 0.5 for s in range(k) for i in range(n))
    assert m.min_clearance == pytest.approx(clear, rel=1e-12)


def test_as_dict_has_every_field():
    t = np.linspace(0, 1, 5)
    m = evaluate_run(
        make_traj(t, np.zeros((5, 1, 3))), np.zeros((1, 3)), NO_OBS, d_min=0.5, eps_p=0.3
    )
    assert isinstance(m, RunMetrics)
    assert set(m.as_dict()) == {
        "success",
        "collided",
        "completion_time",
        "min_separation",
        "min_clearance",
        "formation_error",
        "control_effort",
        "rms_jerk",
        "saturation_fraction",
    }


@pytest.mark.parametrize(
    "kwargs",
    [
        {"d_min": 0.0, "eps_p": 0.3},
        {"d_min": 0.5, "eps_p": 0.0},
        {"d_min": 0.5, "eps_p": 0.3, "a_max": 0.0},
    ],
)
def test_bad_arguments(kwargs):
    t = np.linspace(0, 1, 5)
    with pytest.raises(ValueError):
        evaluate_run(make_traj(t, np.zeros((5, 1, 3))), np.zeros((1, 3)), NO_OBS, **kwargs)


def test_shape_checks():
    t = np.linspace(0, 1, 5)
    traj = make_traj(t, np.zeros((5, 2, 3)))
    with pytest.raises(ValueError):
        evaluate_run(traj, np.zeros((3, 3)), NO_OBS, d_min=0.5, eps_p=0.3)
    with pytest.raises(ValueError):
        evaluate_run(
            make_traj([0.0], np.zeros((1, 2, 3))), np.zeros((2, 3)), NO_OBS, d_min=0.5, eps_p=0.3
        )


def test_controller_cost_per_agent():
    slots = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
    ctrl = APFController(slots, [(0, 1), (1, 2)], [Sphere((2.0, 2.0, 0.0), 0.5)])
    mean, p99 = controller_cost(ctrl, slots + 0.1, np.zeros((3, 3)), reps=50)
    assert 0 < mean <= p99 < 1e5  # microseconds per agent


def test_separation_exactly_d_min_is_a_collision():
    """F requires r > d_min strictly (main.tex §3.3)."""
    t = np.linspace(0, 1, 3)
    slots = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    p = np.broadcast_to(slots, (3, 2, 3)).copy()
    p[1, 1] = [0.5, 0.0, 0.0]
    assert evaluate_run(make_traj(t, p), slots, NO_OBS, d_min=0.5, eps_p=0.3).collided
