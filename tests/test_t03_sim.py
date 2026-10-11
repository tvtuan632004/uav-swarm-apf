"""T03 acceptance tests: double-integrator simulator. Spec: docs/specs/T03-sim.md."""

from __future__ import annotations

import numpy as np
import pytest

from uavapf.sim import SimConfig, Trajectory, rk4_step, saturate_norm, simulate

K_SPRING = 1.5  # 1/s^2


def harmonic(t, p, v):
    return -K_SPRING * p


def zero(t, p, v):
    return np.zeros_like(p)


def energy(traj: Trajectory, k: float = K_SPRING) -> np.ndarray:
    kin = 0.5 * np.sum(traj.v**2, axis=(1, 2))
    pot = 0.5 * k * np.sum(traj.p**2, axis=(1, 2))
    return kin + pot


@pytest.fixture
def state(rng: np.random.Generator):
    n = 7
    return rng.normal(size=(n, 3)), rng.normal(size=(n, 3))


@pytest.fixture
def rng():
    return np.random.default_rng(1234)


# --------------------------------------------------------------------------- #
# saturate_norm (M2)
# --------------------------------------------------------------------------- #
def test_saturate_norm_caps_and_preserves_direction(rng):
    x = rng.normal(size=(50, 3)) * 10
    y = saturate_norm(x, 2.0)
    norms = np.linalg.norm(y, axis=-1)
    assert np.all(norms <= 2.0 + 1e-12)
    big = np.linalg.norm(x, axis=-1) > 2.0
    np.testing.assert_allclose(norms[big], 2.0, rtol=1e-12)
    # same direction
    cos = np.sum(x * y, axis=-1) / (np.linalg.norm(x, axis=-1) * norms)
    np.testing.assert_allclose(cos, 1.0, rtol=1e-12)


def test_saturate_norm_leaves_small_rows_and_zero(rng):
    x = rng.normal(size=(10, 3)) * 0.01
    x[3] = 0.0
    y = saturate_norm(x, 1.0)
    np.testing.assert_array_equal(y, x)
    assert np.all(np.isfinite(y))


def test_saturate_norm_none_returns_copy_float64(rng):
    x = rng.normal(size=(4, 3)).astype(np.float32)
    y = saturate_norm(x, None)
    assert y.dtype == np.float64
    assert y is not x
    np.testing.assert_allclose(y, x)


def test_saturate_norm_batched_shape(rng):
    x = rng.normal(size=(5, 4, 3)) * 5
    y = saturate_norm(x, 1.0)
    assert y.shape == x.shape
    assert np.all(np.linalg.norm(y, axis=-1) <= 1.0 + 1e-12)


# --------------------------------------------------------------------------- #
# rk4_step
# --------------------------------------------------------------------------- #
def test_rk4_step_is_pure(state):
    p, v = state
    p0, v0 = p.copy(), v.copy()
    rk4_step(harmonic, 0.0, p, v, 0.01)
    np.testing.assert_array_equal(p, p0)
    np.testing.assert_array_equal(v, v0)


def test_rk4_step_matches_one_step_taylor(state):
    # for linear ODE, one RK4 step equals the 4th-order Taylor polynomial of exp(A dt)
    p, v = state
    dt, w2 = 0.05, K_SPRING
    p1, v1 = rk4_step(harmonic, 0.0, p, v, dt)
    h2 = w2 * dt**2
    c = 1 - h2 / 2 + h2**2 / 24  # cos-like series
    s = dt * (1 - h2 / 6)  # sin/omega series
    np.testing.assert_allclose(p1, c * p + s * v, rtol=1e-13, atol=1e-14)
    np.testing.assert_allclose(v1, -w2 * s * p + c * v, rtol=1e-13, atol=1e-14)


# --------------------------------------------------------------------------- #
# simulate: physics
# --------------------------------------------------------------------------- #
def test_energy_conservation_undamped(state):
    """Acceptance criterion 1 (task table: error < 1e-6)."""
    p, v = state
    traj = simulate(harmonic, p, v, SimConfig(dt=0.01, t_final=20.0))
    e = energy(traj)
    assert np.max(np.abs(e - e[0])) / e[0] < 1e-6


def test_matches_analytic_solution(state):
    p, v = state
    w = np.sqrt(K_SPRING)
    traj = simulate(harmonic, p, v, SimConfig(dt=0.01, t_final=5.0))
    t = traj.t[:, None, None]
    p_exact = p * np.cos(w * t) + v / w * np.sin(w * t)
    np.testing.assert_allclose(traj.p, p_exact, atol=1e-8)


def test_fourth_order_convergence(state):
    """Acceptance criterion 2."""
    p, v = state
    w, t_end = np.sqrt(K_SPRING), 3.0
    p_exact = p * np.cos(w * t_end) + v / w * np.sin(w * t_end)
    errs = []
    for dt in (0.1, 0.05):
        traj = simulate(harmonic, p, v, SimConfig(dt=dt, t_final=t_end))
        assert traj.t[-1] == pytest.approx(t_end)
        errs.append(np.max(np.abs(traj.p[-1] - p_exact)))
    assert 13.0 < errs[0] / errs[1] < 19.0


def test_free_flight_is_exact(state):
    """Acceptance criterion 4."""
    p, v = state
    traj = simulate(zero, p, v, SimConfig(dt=0.1, t_final=2.0))
    t = traj.t[:, None, None]
    np.testing.assert_allclose(traj.p, p + v * t, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(traj.v, np.broadcast_to(v, traj.v.shape), rtol=0, atol=1e-15)


def test_acceleration_saturation_inside_dynamics(state):
    """Acceptance criterion 3: saturation is part of the ODE, not only of the log."""
    p, v = state
    a_max = 0.3

    def strong(t, p, v):
        return -100.0 * p

    traj = simulate(strong, p, v, SimConfig(dt=0.01, t_final=1.0, a_max=a_max))
    assert np.all(np.linalg.norm(traj.u, axis=-1) <= a_max + 1e-12)
    dv = np.diff(traj.v, axis=0)
    assert np.all(np.linalg.norm(dv, axis=-1) <= a_max * 0.01 * (1 + 1e-9))


def test_speed_limit(state):
    p, v = state
    traj = simulate(
        lambda t, p, v: np.ones_like(p) * 5.0,
        p,
        v,
        SimConfig(dt=0.01, t_final=2.0, v_max=1.2),
    )
    assert np.all(np.linalg.norm(traj.v[1:], axis=-1) <= 1.2 + 1e-12)


def test_controller_receives_whole_swarm_state(state):
    p, v = state
    seen = []

    def ctrl(t, p_, v_):
        seen.append((p_.shape, v_.shape))
        return np.zeros_like(p_)

    simulate(ctrl, p, v, SimConfig(dt=0.1, t_final=0.3))
    assert all(s == (p.shape, v.shape) for s in seen)


# --------------------------------------------------------------------------- #
# simulate: bookkeeping
# --------------------------------------------------------------------------- #
def test_output_shapes_dtypes_and_times(state):
    p, v = state
    traj = simulate(harmonic, p, v, SimConfig(dt=0.1, t_final=1.0))
    assert traj.t.shape == (11,)
    assert traj.p.shape == traj.v.shape == traj.u.shape == (11, *p.shape)
    for a in (traj.t, traj.p, traj.v, traj.u):
        assert a.dtype == np.float64
        assert a.flags["C_CONTIGUOUS"]
    np.testing.assert_allclose(traj.t, np.arange(11) * 0.1, atol=1e-12)
    np.testing.assert_array_equal(traj.p[0], p)
    np.testing.assert_array_equal(traj.v[0], v)


def test_time_stamps_are_step_times_without_drift():
    dt = 0.1
    traj = simulate(zero, np.zeros((1, 3)), np.zeros((1, 3)), SimConfig(dt=dt, t_final=1000.0))
    np.testing.assert_array_equal(traj.t, np.arange(len(traj)) * dt)


def test_t_final_not_multiple_of_dt_rounds_up(state):
    p, v = state
    traj = simulate(zero, p, v, SimConfig(dt=0.3, t_final=1.0))
    assert traj.t[-1] == pytest.approx(1.2)


def test_log_every_keeps_last_state(state):
    p, v = state
    full = simulate(harmonic, p, v, SimConfig(dt=0.1, t_final=1.0))
    thin = simulate(harmonic, p, v, SimConfig(dt=0.1, t_final=1.0, log_every=3))
    np.testing.assert_allclose(thin.t, [0.0, 0.3, 0.6, 0.9, 1.0], atol=1e-12)
    idx = [0, 3, 6, 9, 10]
    np.testing.assert_allclose(thin.p, full.p[idx], rtol=1e-14)
    np.testing.assert_allclose(thin.u, full.u[idx], rtol=1e-14)


def test_logged_u_is_command_at_logged_state(state):
    p, v = state
    traj = simulate(harmonic, p, v, SimConfig(dt=0.05, t_final=1.0, a_max=0.8))
    expected = saturate_norm(-K_SPRING * traj.p, 0.8)
    np.testing.assert_allclose(traj.u, expected, rtol=1e-14, atol=1e-15)


def test_controller_call_count(state):
    """4 calls per step + 1 for the final logged state."""
    p, v = state
    calls = 0

    def ctrl(t, p_, v_):
        nonlocal calls
        calls += 1
        return -p_

    simulate(ctrl, p, v, SimConfig(dt=0.1, t_final=1.0))
    assert calls == 4 * 10 + 1


def test_stop_condition(state):
    p, v = state
    traj = simulate(zero, p, v, SimConfig(dt=0.1, t_final=10.0), stop=lambda t, p, v: t >= 0.5)
    assert traj.t[-1] == pytest.approx(0.5)
    assert len(traj.t) == 6


def test_inputs_not_modified(state):
    p, v = state
    p0, v0 = p.copy(), v.copy()
    simulate(harmonic, p, v, SimConfig(dt=0.1, t_final=1.0, v_max=0.1))
    np.testing.assert_array_equal(p, p0)
    np.testing.assert_array_equal(v, v0)


def test_single_agent():
    traj = simulate(harmonic, [[1.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]], SimConfig(0.01, 1.0))
    assert traj.p.shape == (101, 1, 3)


# --------------------------------------------------------------------------- #
# validation
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    "kwargs",
    [
        {"dt": 0.0, "t_final": 1.0},
        {"dt": -0.1, "t_final": 1.0},
        {"dt": 0.1, "t_final": 0.0},
        {"dt": 0.1, "t_final": 1.0, "log_every": 0},
        {"dt": 0.1, "t_final": 1.0, "a_max": 0.0},
        {"dt": 0.1, "t_final": 1.0, "v_max": -1.0},
    ],
)
def test_bad_config_raises(kwargs):
    with pytest.raises(ValueError):
        SimConfig(**kwargs)


@pytest.mark.parametrize(
    ("p0", "v0"),
    [
        (np.zeros((3, 2)), np.zeros((3, 2))),
        (np.zeros((3, 3)), np.zeros((4, 3))),
        (np.zeros((0, 3)), np.zeros((0, 3))),
        (np.zeros(3), np.zeros(3)),
        (np.array([[np.nan, 0, 0]]), np.zeros((1, 3))),
    ],
)
def test_bad_state_raises(p0, v0):
    with pytest.raises(ValueError):
        simulate(zero, p0, v0, SimConfig(0.1, 1.0))


def test_bad_controller_output_raises(state):
    p, v = state
    with pytest.raises(ValueError):
        simulate(lambda t, p, v: np.zeros(3), p, v, SimConfig(0.1, 1.0))


# --------------------------------------------------------------------------- #
# I/O
# --------------------------------------------------------------------------- #
def test_save_load_roundtrip(state, tmp_path):
    p, v = state
    traj = simulate(harmonic, p, v, SimConfig(dt=0.1, t_final=1.0))
    out = traj.save(tmp_path / "nested" / "run.npz", seed=7, scenario="S1")
    assert out.is_file()
    back = Trajectory.load(out)
    for name in ("t", "p", "v", "u"):
        np.testing.assert_array_equal(getattr(back, name), getattr(traj, name))
    with np.load(out) as z:
        assert int(z["meta_seed"]) == 7
        assert str(z["meta_scenario"]) == "S1"
