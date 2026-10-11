# T03 — Double-integrator swarm simulator (RK4, saturation, logging)

- Branch: `T03-sim`
- Spec + tests: Claude · Implementation: Claude (on the user's request) · Review: **Codex**
- Tests: `tests/test_t03_sim.py`
- Done when: an undamped conservative system conserves energy with relative
  error below 1e-6 over the run.

## 1. Scope
In: the plant model, a fixed-step RK4 integrator, acceleration and speed
saturation, the trajectory container and `.npz` I/O.
Out: every controller and force (T06+), scenario loading (T05), plotting (T05),
adaptive time stepping (see D002 in `docs/decisions.md`).

## 2. Model
Agent i, i = 1..N, position p_i ∈ R³ (m), velocity v_i ∈ R³ (m/s):

    ṗ_i = v_i
    v̇_i = sat(u_i(t, p, v), a_max)                                   (M1)

with the norm saturation, applied row by row,

    sat(x, L) = x · min(1, L / ‖x‖)          (sat(0, L) = 0)          (M2)

The controller u sees the **whole** swarm state (p, v), shape (N, 3) each, and
returns an (N, 3) acceleration command in m/s². It is evaluated at each RK4
stage (continuous-time control, not zero-order hold).

Optional speed limit: after each full RK4 step, v_i ← sat(v_i, v_max). This is a
projection, not part of the ODE; it breaks energy conservation, so tests of
conservation run with `v_max=None`.

## 3. Public API (`uavapf.sim`)

```python
Controller = Callable[[float, FloatArray, FloatArray], FloatArray]
StopFn = Callable[[float, FloatArray, FloatArray], bool]


def saturate_norm(x: ArrayLike, limit: float | None) -> FloatArray: ...


@dataclass(frozen=True, slots=True)
class SimConfig:
    dt: float  # s, > 0
    t_final: float  # s, > 0
    a_max: float | None = None  # m/s^2, > 0 or None (no limit)
    v_max: float | None = None  # m/s, > 0 or None
    log_every: int = 1  # store every k-th step (>= 1); the final state is always stored


def rk4_step(accel, t, p, v, dt) -> tuple[FloatArray, FloatArray]: ...


@dataclass(frozen=True, slots=True)
class Trajectory:
    t: FloatArray  # (K,)       s
    p: FloatArray  # (K, N, 3)  m
    v: FloatArray  # (K, N, 3)  m/s
    u: FloatArray  # (K, N, 3)  m/s^2, saturated command at t[k], state (p[k], v[k])

    def save(self, path: Path, **meta) -> Path: ...
    @classmethod
    def load(cls, path: Path) -> Trajectory: ...


def simulate(controller, p0, v0, config, stop=None) -> Trajectory: ...
```

Details:
- `saturate_norm` works on any `(..., 3)` array (norm over the last axis) and
  returns a new float64 array. `limit=None` → copy of `x`.
- `rk4_step(accel, ...)`: classical RK4 on the state (p, v) where
  `accel(t, p, v)` already includes saturation. Pure: inputs are not modified.
- `simulate`:
  - `n_steps = ceil(t_final / dt)` (with a 1e-9 relative tolerance so 1.0/0.1
    gives 10 steps); the last step ends exactly at `n_steps · dt`.
  - Logged times are `t[k] = k_step · dt`, steps 0, log_every, 2·log_every, …,
    plus the last step if not already included.
  - `stop(t, p, v)` is checked after every step; when it returns `True` the
    current state is logged and the run ends.
  - p0, v0 are not modified. Output arrays are float64 and C-contiguous.
- `Trajectory.save(path, **meta)` writes a compressed `.npz` with keys t, p, v,
  u and `meta_<key>` for every extra keyword (stored as given by NumPy).
  Parent directories are created. `load` restores t, p, v, u.

## 4. Edge cases / validation (raise `ValueError`)
- p0 or v0 not of shape (N, 3), N ≥ 1, or shapes differ.
- non-finite values in p0 or v0.
- dt ≤ 0, t_final ≤ 0, log_every < 1, a_max ≤ 0, v_max ≤ 0.
- controller output not of shape (N, 3) → `ValueError` at the first call.
- N = 1 must work.

## 5. Acceptance criteria
1. Harmonic swarm u = −k p, no saturation: |E(t) − E(0)| / E(0) < 1e-6 for all
   logged t (E = ½‖v‖² + ½k‖p‖² summed over agents), dt = 0.01 s, 20 s.
2. RK4 is 4th order: halving dt divides the end-point error vs the analytic
   solution by 16 ± 3.
3. ‖u_i‖ ≤ a_max and ‖v_i‖ ≤ v_max at every logged instant when limits are set.
4. Free flight (u = 0) is exact: p(t) = p0 + v0·t.
5. Logging rules, stop rule, input purity, and save/load round trip as in §3.

## 6. Notes for the reviewer
- The logged `u[k]` must be the RK4 first-stage value at step k (no extra
  controller call per step); only the final logged state costs one extra call.
- Check that saturation happens inside every RK4 stage, not only on the logged value.
