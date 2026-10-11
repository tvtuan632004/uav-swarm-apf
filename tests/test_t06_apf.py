"""T06 acceptance tests: APF controller eqs. (4)-(15) and reset eq. (16), paper/main.tex.

Written by Claude independently of the implementation (Codex). Everything is checked
against a plain-Python reference transcribed from main.tex, not against uavapf internals.
Spec: docs/specs/T06-base-controller.md.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uavapf.control import APFConfig, APFController, ResetConfig, StallReset
from uavapf.geometry import Cylinder, Sphere
from uavapf.sim import SimConfig, simulate

CFG = APFConfig()  # Table 4 initial values
E_Z = np.array([0.0, 0.0, 1.0])

# Square formation, 2 m spacing (> d_s = 1.2 m); slot 0 sits 0.6 m from the sphere
# surface, i.e. inside its influence region rho0 = 2 m (the GNRON situation).
SLOTS = np.array([[0.0, 0.0, 2.0], [2.0, 0.0, 2.0], [0.0, 2.0, 2.0], [2.0, 2.0, 2.0]])
EDGES = [(0, 1), (0, 2), (1, 3), (2, 3)]
SPHERE = Sphere((-1.4, 0.0, 2.0), 0.8)
CYL = Cylinder((3.0, 3.4), 0.5)
OBSTACLES = [SPHERE, CYL]


# --------------------------------------------------------------------------- #
# Reference model, transcribed from main.tex (loops, no vectorisation)
# --------------------------------------------------------------------------- #
def ref_rho(p: np.ndarray, ob) -> tuple[float, np.ndarray]:
    """Clearance and outward normal, eq. (2) (vertical cylinder: horizontal plane)."""
    if isinstance(ob, Sphere):
        d = p - np.array(ob.center)
    else:
        d = p - np.array([*ob.center_xy, p[2]])
    dist = float(np.linalg.norm(d))
    return dist - ob.radius, d / dist


def ref_phi(rho: float, c: APFConfig = CFG) -> float:
    """Eq. (5)."""
    if rho > c.rho0:
        return 0.0
    return c.eta / c.m * (1.0 / rho - 1.0 / c.rho0) ** c.m


def ref_psi(r: float, a_ij: float, d_star: float, c: APFConfig = CFG) -> float:
    """Eqs. (8)-(9)."""
    morse = a_ij * c.de * (1.0 - math.exp(-c.alpha * (r - d_star))) ** 2
    barrier = 0.0
    if r <= c.d_s:
        barrier = c.kb / c.m * (1.0 / (r - c.d_min) - 1.0 / (c.d_s - c.d_min)) ** c.m
    return morse + barrier


def ref_U(p, slots=SLOTS, edges=EDGES, obstacles=OBSTACLES, c: APFConfig = CFG) -> float:
    """Eq. (10): U = U_a + U_r + U_p."""
    n_agents = len(slots)
    edge_set = {tuple(sorted(e)) for e in edges}
    total = 0.0
    for i in range(n_agents):
        delta = p[i] - slots[i]
        g = float(np.linalg.norm(delta))
        total += 0.5 * c.ka * g * g  # eq. (4)
        for ob in obstacles:
            total += ref_phi(ref_rho(p[i], ob)[0], c) * g**c.n  # eq. (6)
        for j in range(i + 1, n_agents):
            r = float(np.linalg.norm(p[i] - p[j]))
            d_star = float(np.linalg.norm(slots[i] - slots[j]))
            total += ref_psi(r, float((i, j) in edge_set), d_star, c)
    return total


def fd_grad(f, x: np.ndarray, h: float = 1e-6) -> np.ndarray:
    g = np.zeros_like(x)
    for idx in np.ndindex(x.shape):
        e = np.zeros_like(x)
        e[idx] = h
        g[idx] = (f(x + e) - f(x - e)) / (2 * h)
    return g


def fd_hess3(f, x: np.ndarray, h: float = 1e-4) -> np.ndarray:
    """3x3 Hessian of a scalar function of one 3-vector by central differences."""
    hess = np.zeros((3, 3))
    eye = np.eye(3) * h
    for a in range(3):
        for b in range(3):
            hess[a, b] = (
                f(x + eye[a] + eye[b])
                - f(x + eye[a] - eye[b])
                - f(x - eye[a] + eye[b])
                + f(x - eye[a] - eye[b])
            ) / (4 * h * h)
    return 0.5 * (hess + hess.T)


def psd(m: np.ndarray) -> np.ndarray:
    w, q = np.linalg.eigh(m)
    return (q * np.maximum(w, 0.0)) @ q.T


def ref_Dv(p, v, slots=SLOTS, edges=EDGES, obstacles=OBSTACLES, c: APFConfig = CFG):
    """Eq. (12) with every local Hessian taken numerically, then PSD-projected."""
    n_agents = len(slots)
    edge_set = {tuple(sorted(e)) for e in edges}
    out = c.ka * v.copy()
    for i in range(n_agents):
        for ob in obstacles:

            def u_ik(q, i=i, ob=ob):
                return ref_phi(ref_rho(q, ob)[0], c) * np.linalg.norm(q - slots[i]) ** c.n

            out[i] += psd(fd_hess3(u_ik, p[i])) @ v[i]
        for j in range(n_agents):
            if j == i:
                continue
            a_ij = float(tuple(sorted((i, j))) in edge_set)
            d_star = float(np.linalg.norm(slots[i] - slots[j]))

            def u_ij(x, a_ij=a_ij, d_star=d_star):
                return ref_psi(float(np.linalg.norm(x)), a_ij, d_star, c)

            out[i] += psd(fd_hess3(u_ij, p[i] - p[j])) @ (v[i] - v[j])
    return out


def ref_omega(p, slots=SLOTS, obstacles=OBSTACLES, c: APFConfig = CFG) -> np.ndarray:
    """Eq. (13)."""
    out = np.zeros_like(p)
    for i in range(len(p)):
        for ob in obstacles:
            rho, n = ref_rho(p[i], ob)
            s = min(max((c.rho0 - rho) / (c.rho0 - c.rho1), 0.0), 1.0)
            beta = 3 * s * s - 2 * s**3
            out[i] += c.kg * beta * (np.cross(slots[i] - p[i], n) + c.mu * E_Z)
    return out


def random_states(rng: np.random.Generator, count: int, spread: float = 0.7):
    """Valid perturbed states: clearance > 0.2 m, separations > d_min + 0.15 m."""
    states = []
    while len(states) < count:
        p = SLOTS + rng.uniform(-spread, spread, size=SLOTS.shape)
        rho = [ref_rho(p[i], ob)[0] for i in range(len(p)) for ob in OBSTACLES]
        r = [np.linalg.norm(p[i] - p[j]) for i in range(4) for j in range(i + 1, 4)]
        if min(rho) > 0.2 and min(r) > CFG.d_min + 0.15:
            states.append((p, rng.normal(size=p.shape)))
    return states


@pytest.fixture
def ctrl():
    return APFController(SLOTS, EDGES, OBSTACLES)


@pytest.fixture
def rng():
    return np.random.default_rng(606)


# --------------------------------------------------------------------------- #
# Potential and gradient (criteria 1-3)
# --------------------------------------------------------------------------- #
def test_potential_matches_reference(ctrl, rng):
    for p, _ in random_states(rng, 20):
        assert ctrl.potential(p) == pytest.approx(ref_U(p), rel=1e-12, abs=1e-12)


def test_force_is_minus_gradient_of_reference(ctrl, rng):
    """Criterion 3: analytic gradient vs central finite differences of eq. (10)."""
    for p, v in random_states(rng, 20):
        force = ctrl.components(p, v).force
        np.testing.assert_allclose(force, -fd_grad(ref_U, p), rtol=1e-6, atol=1e-6)


def test_gnron_force_vanishes_at_slot_inside_influence(ctrl):
    """Criterion 1: the slot of agent 0 lies inside rho0 of the sphere, yet is an equilibrium."""
    rho0_slot = ref_rho(SLOTS[0], SPHERE)[0]
    assert 0 < rho0_slot < CFG.rho0
    comp = ctrl.components(SLOTS, np.zeros_like(SLOTS))
    np.testing.assert_allclose(comp.force, 0.0, atol=1e-12)
    np.testing.assert_allclose(ctrl(0.0, SLOTS, np.zeros_like(SLOTS)), 0.0, atol=1e-12)


def test_pair_forces_equal_and_opposite(rng):
    """Criterion 2: internal (pair) forces sum to zero, so total force = attraction only."""
    c = APFController(SLOTS, EDGES, [])
    for p, v in random_states(rng, 10):
        force = c.components(p, v).force
        np.testing.assert_allclose(force.sum(axis=0), -CFG.ka * (p - SLOTS).sum(axis=0), atol=1e-10)


def test_barrier_blows_up_and_repels():
    """Criterion 2: psi_b -> inf as r -> d_min+, force pushes the pair apart."""
    slots = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    c = APFController(slots, [], [])
    prev = 0.0
    for gap in (0.5, 0.1, 1e-2, 1e-3):
        p = np.array([[0.0, 0.0, 0.0], [CFG.d_min + gap, 0.0, 0.0]])
        u = c.potential(p)
        assert u > prev
        prev = u
        f = c.components(p, np.zeros_like(p)).force
        assert f[1, 0] > 0 > f[0, 0]
    assert prev > 1e6


def test_obstacle_force_points_away_close_to_surface(ctrl):
    p = SLOTS.copy()
    p[0] = [-0.55, 0.0, 2.0]  # 0.05 m from the sphere, slot farther away on +x
    f = ctrl.components(p, np.zeros_like(p)).force
    assert f[0, 0] > 0  # pushed along +n (away from the obstacle)


# --------------------------------------------------------------------------- #
# Boundary rho = rho0
# --------------------------------------------------------------------------- #
def test_obstacle_terms_vanish_at_rho0():
    slots = np.array([[10.0, 0.0, 2.0]])
    p = np.array([[SPHERE.center[0] + SPHERE.radius + CFG.rho0, 0.0, 2.0]])
    v = np.array([[0.3, -0.2, 0.1]])
    with_obs = APFController(slots, [], [SPHERE]).components(p, v)
    without = APFController(slots, [], []).components(p, v)
    np.testing.assert_allclose(with_obs.force, without.force, atol=1e-14)
    np.testing.assert_allclose(with_obs.damping, without.damping, atol=1e-14)
    np.testing.assert_allclose(with_obs.omega, 0.0, atol=1e-14)  # beta(rho0) = 0, bias too


def test_force_continuous_across_rho0():
    slots = np.array([[10.0, 0.0, 2.0]])
    c = APFController(slots, [], [SPHERE])
    x0 = SPHERE.center[0] + SPHERE.radius + CFG.rho0
    f_in = c.components([[x0 - 1e-7, 0, 2]], np.zeros((1, 3))).force
    f_out = c.components([[x0 + 1e-7, 0, 2]], np.zeros((1, 3))).force
    np.testing.assert_allclose(f_in, f_out, atol=1e-9)


# --------------------------------------------------------------------------- #
# Damping (T4) and gyroscopic force (T3), criterion 4
# --------------------------------------------------------------------------- #
def test_damping_matches_numerical_projected_hessians(ctrl, rng):
    """Eq. (12): every local Hessian, PSD-projected, independently of the implementation."""
    for p, v in random_states(rng, 8):
        np.testing.assert_allclose(
            ctrl.components(p, v).damping, ref_Dv(p, v), rtol=2e-4, atol=2e-5
        )


def test_damping_is_dissipative_on_many_states(ctrl, rng):
    """T09 criterion: v^T D v >= 0 (Lemma psd) on 10,000 random states, incl. near contact."""
    worst = np.inf
    for p, v in random_states(rng, 10_000, spread=0.95):
        dv = ctrl.components(p, v).damping
        worst = min(worst, float(np.sum(v * dv)) / float(np.sum(v * v)))
    assert worst >= -1e-12


def test_gyroscopic_matches_reference_and_does_no_work(ctrl, rng):
    for p, v in random_states(rng, 20):
        comp = ctrl.components(p, v)
        np.testing.assert_allclose(comp.omega, ref_omega(p), rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(comp.gyroscopic, np.cross(comp.omega, v), atol=1e-12)
        work = np.sum(v * comp.gyroscopic, axis=1)
        np.testing.assert_allclose(work, 0.0, atol=1e-12)


def test_gyroscopic_turns_head_on_approach_towards_goal_side():
    """Eq. (14): heading into the obstacle, the force is the goal direction tangent to it."""
    slots = np.array([[0.0, 3.0, 2.0]])
    c = APFController(slots, [], [SPHERE], APFConfig(mu=1e-9))
    p = np.array([[-0.4, 0.0, 2.0]])  # 0.2 m from surface, n = +x
    v = np.array([[-1.0, 0.0, 0.0]])  # into the obstacle
    gyro = c.components(p, v).gyroscopic[0]
    g = slots[0] - p[0]
    n = np.array([1.0, 0.0, 0.0])
    tangent = g - (g @ n) * n
    cos = gyro @ tangent / (np.linalg.norm(gyro) * np.linalg.norm(tangent))
    assert cos > 0.999


def test_control_law_assembly(ctrl, rng):
    """Eq. (15): u = -grad U - Kd v - Kr D v + omega x v."""
    for p, v in random_states(rng, 5):
        comp = ctrl.components(p, v)
        expected = comp.force - CFG.kd * v - CFG.kr * comp.damping + comp.gyroscopic
        np.testing.assert_allclose(ctrl(0.0, p, v), expected, rtol=1e-14, atol=1e-14)


def test_permutation_equivariance(rng):
    """Vectorised vs relabelled agents: outputs permute with the agents."""
    perm = np.array([2, 0, 3, 1])
    inv = np.argsort(perm)
    edges_perm = [(int(inv[i]), int(inv[j])) for i, j in EDGES]
    a = APFController(SLOTS, EDGES, OBSTACLES)
    b = APFController(SLOTS[perm], edges_perm, OBSTACLES)
    for p, v in random_states(rng, 5):
        np.testing.assert_allclose(b(0.0, p[perm], v[perm]), a(0.0, p, v)[perm], atol=1e-12)


# --------------------------------------------------------------------------- #
# Closed loop (criterion 5) and energy V = |v|^2/2 + U (Theorem safety, eq. Vdot)
# --------------------------------------------------------------------------- #
def test_closed_loop_energy_nonincreasing_and_safe():
    slots = np.array([[5.0, -2.0, 2.0], [5.0, 0.0, 2.0], [5.0, 2.0, 2.0]])
    p0 = slots.copy()
    p0[:, 0] = -5.0
    c = APFController(slots, [(0, 1), (1, 2)], [Sphere((0.0, 0.0, 2.0), 0.8)])
    traj = simulate(c, p0, np.zeros_like(p0), SimConfig(dt=0.01, t_final=12.0))
    assert np.isfinite(traj.p).all()
    energy = np.array(
        [0.5 * np.sum(v * v) + c.potential(p) for p, v in zip(traj.p, traj.v, strict=True)]
    )
    assert np.all(np.diff(energy) <= 1e-9 * energy[0])
    rho, _ = c.geometry.distance(traj.p.reshape(-1, 3))
    assert rho.min() > 0
    r = np.linalg.norm(traj.p[:, :, None] - traj.p[:, None], axis=-1)[:, *np.triu_indices(3, 1)]
    assert r.min() > CFG.d_min


# --------------------------------------------------------------------------- #
# Validation (spec section 4)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    ("slots", "edges", "obstacles"),
    [
        (SLOTS, EDGES, [Sphere(tuple(SLOTS[0]), 0.5)]),  # slot inside obstacle
        (np.array([[0.0, 0, 0], [1.0, 0, 0]]), [], []),  # slot pair closer than d_s
        (SLOTS, [(0, 0)], []),  # self loop
        (SLOTS, [(0, 7)], []),  # bad index
        (SLOTS, [(0, 1), (1, 0)], []),  # duplicate edge
        (SLOTS, [], [Cylinder((9.0, 9.0), 0.5, 0.0, 5.0)]),  # capped cylinder
    ],
)
def test_invalid_setup_raises(slots, edges, obstacles):
    with pytest.raises(ValueError):
        APFController(slots, edges, obstacles)


def test_numpy_integer_edges_accepted():
    """Edges often come from NumPy (np.argwhere, YAML -> array); they are valid indices."""
    edges = [tuple(e) for e in np.array(EDGES, dtype=np.int64)]
    a = APFController(SLOTS, edges, OBSTACLES)
    b = APFController(SLOTS, EDGES, OBSTACLES)
    p = SLOTS + 0.1
    np.testing.assert_array_equal(a(0.0, p, np.zeros_like(p)), b(0.0, p, np.zeros_like(p)))


def test_single_agent_no_obstacles():
    c = APFController([[1.0, 2.0, 3.0]], [], [])
    u = c(0.0, [[0.0, 2.0, 3.0]], [[0.0, 0.0, 0.0]])
    np.testing.assert_allclose(u, [[CFG.ka, 0.0, 0.0]])


# --------------------------------------------------------------------------- #
# Hybrid reset, eq. (16) (criterion 6)
# --------------------------------------------------------------------------- #
def trap_setup():
    """Symmetric trap: slot directly behind the sphere along +x, agent in front of it."""
    slots = np.array([[3.0, 0.0, 2.0]])
    c = APFController(slots, [], [Sphere((0.0, 0.0, 2.0), 0.8)])
    p = np.array([[-1.5, 0.0, 2.0]])  # rho = 0.7 m < rho0, n = -x, goal along +x
    return c, p


def test_reset_waits_for_sustained_stall_then_kicks_by_delta():
    c, p = trap_setup()
    reset = StallReset(c)
    v0 = np.zeros((1, 3))
    times = np.arange(0.0, 1.0, 0.01)
    for t in times:
        np.testing.assert_array_equal(reset(float(t), p, v0), v0)
    v1 = reset(1.0, p, v0)
    assert np.linalg.norm(v1 - v0) == pytest.approx(ResetConfig().delta)
    assert reset.count[0] == 1


def test_reset_direction_primary_branch():
    """t = dir(omega x (-n_c)); in the symmetric trap omega = kg*beta*mu*e_z."""
    c, p = trap_setup()
    reset = StallReset(c)
    reset(0.0, p, np.zeros((1, 3)))
    dv = reset(1.0, p, np.zeros((1, 3)))[0]
    _, n = ref_rho(p[0], Sphere((0.0, 0.0, 2.0), 0.8))
    omega = ref_omega(p, slots=c.slots, obstacles=[Sphere((0.0, 0.0, 2.0), 0.8)])[0]
    t_dir = np.cross(omega, -n)
    np.testing.assert_allclose(dv, ResetConfig().delta * t_dir / np.linalg.norm(t_dir), atol=1e-12)


def test_reset_direction_fallback_without_sensed_obstacle():
    slots = np.array([[3.0, 0.0, 2.0]])
    c = APFController(slots, [], [Sphere((-20.0, 0.0, 2.0), 0.8)])  # far beyond rho0
    p = np.array([[0.0, 1.0, 2.0]])
    reset = StallReset(c)
    reset(0.0, p, np.zeros((1, 3)))
    dv = reset(1.0, p, np.zeros((1, 3)))[0]
    t_dir = np.cross(E_Z, slots[0] - p[0])
    np.testing.assert_allclose(dv, 0.5 * t_dir / np.linalg.norm(t_dir), atol=1e-12)


def test_reset_not_fired_when_moving_or_near_slot():
    c, p = trap_setup()
    reset = StallReset(c)
    moving = np.array([[0.2, 0.0, 0.0]])
    for t in np.arange(0.0, 5.0, 0.1):
        np.testing.assert_array_equal(reset(float(t), p, moving), moving)
    near_slot = c.slots + 0.1
    reset2 = StallReset(c)
    for t in np.arange(0.0, 5.0, 0.1):
        np.testing.assert_array_equal(reset2(float(t), near_slot, np.zeros((1, 3))), 0.0)


def test_reset_respects_dwell_and_count_limits():
    c, p = trap_setup()
    cfg = ResetConfig(stall_time=0.5, dwell_time=2.0, max_resets=3)
    reset = StallReset(c, cfg)
    fired = []
    for k in range(2001):  # 20 s at 10 ms, always stalled (we feed v = 0 back)
        t = k * 0.01
        if np.any(reset(t, p, np.zeros((1, 3))) != 0):
            fired.append(t)
    assert len(fired) == 3
    assert fired[0] == pytest.approx(0.5)
    assert np.all(np.diff(fired) >= 2.0 - 1e-9)


def test_reset_releases_pure_apf_trap_in_simulator():
    """With T3 off (kg = 0) the symmetric trap is a stall; the reset hook must fire."""
    slots = np.array([[3.0, 0.0, 2.0]])
    c = APFController(slots, [], [Sphere((0.0, 0.0, 2.0), 0.8)], APFConfig(kg=0.0))
    p0 = np.array([[-1.5, 0.0, 2.0]])
    reset = StallReset(c)
    traj = simulate(c, p0, np.zeros((1, 3)), SimConfig(dt=0.01, t_final=30.0), reset=reset)
    assert np.isfinite(traj.v).all()
    assert reset.count[0] >= 1
    assert np.abs(traj.p[:, 0, 1]).max() > 0.1  # kicked sideways (fallback dir e_z x goal)


def test_stall_clock_restarts_after_reset():
    """A reset breaks the stall (|v| jumps to Delta), so the next one needs a fresh T_s."""
    c, p = trap_setup()
    cfg = ResetConfig(stall_time=1.0, dwell_time=0.5, max_resets=5)
    reset = StallReset(c, cfg)
    fired = [k * 0.01 for k in range(401) if np.any(reset(k * 0.01, p, np.zeros((1, 3))) != 0)]
    assert len(fired) >= 2
    assert np.all(np.diff(fired) >= cfg.stall_time - 1e-9)
