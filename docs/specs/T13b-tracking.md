# T13b — Moving formation centre (main.tex Remark "Moving formation")

- Spec + tests: Claude · Implementation: Codex · Review: Claude
- Tests: `tests/test_t13b_tracking.py` (do not edit). The file skips itself until
  `uavapf.control.reference` exists; done = `uv run pytest -q` with **0 failed, 0 skipped**.
- Decisions (user, 2026-10-11): **D009** the formation centre moves along a reference with
  bounded speed and acceleration (Q1); **D010** the T1 goal factor is measured to the final
  slot while the attraction tracks the moving slot. Both are recorded in `docs/decisions.md`.
- Done when: the tests pass and `scripts/pilot.py` (tracking on by default) reproduces the
  prototype result on S1–S4 (APF, a_max = 5 m/s²: no collisions; report the table in the handoff).

## 1. Reference path (`src/uavapf/control/reference.py`, new)

Straight line from `start` to `goal` (both (3,) in m) with a minimum-jerk time scaling:

    s(τ) = 10τ³ − 15τ⁴ + 6τ⁵,  τ = clip(t / T, 0, 1)
    c(t) = start + s(τ) (goal − start)
    ċ(t) = s'(τ)/T · (goal − start),   c̈(t) = s''(τ)/T² · (goal − start)

Peak speed = 1.875 L / T and peak acceleration = (10/√3) L / T² with L = ‖goal − start‖,
so the duration is the smallest T meeting both limits:

    T = max(1.875 L / v_ref, sqrt((10/√3) L / a_ref))        (T = 0 when L = 0)

```python
@dataclass(frozen=True, slots=True)
class StraightPath:
    start: tuple[float, float, float]  # m
    goal: tuple[float, float, float]  # m
    v_ref: float  # m/s, > 0
    a_ref: float  # m/s², > 0

    @property
    def duration(self) -> float: ...  # s
    def position(self, t: float | ArrayLike) -> FloatArray: ...  # (3,) or (K, 3), m
    def velocity(self, t) -> FloatArray: ...  # m/s
    def acceleration(self, t) -> FloatArray: ...  # m/s²
```
For t ≤ 0 the centre is at `start` with zero velocity; for t ≥ T at `goal` (zero velocity
and acceleration). Validation: finite start/goal, v_ref > 0, a_ref > 0 (`ValueError`).

## 2. Controllers

Formation offsets δ_i = slots_i − goal (slots are the **final** slots, as today), so
p_i*(t) = c(t) + δ_i, ṗ_i* = ċ, p̈_i* = c̈ for every agent.

`APFController` (existing) gains, without changing any current output:
- `components(p, v, *, slots=None)`: if `slots` (N, 3) is given, the **attraction**
  (eq. 4 and its k_a v damping row) and the **gyroscopic goal term** (p* − p) × n of eq. (13)
  use it. The **T1 goal factor g_i = ‖p_i − p_i^goal‖ always uses the final slots
  `self.slots`** (decision D010, below). Morse desired distances stay those of `self.slots`.
- `slots_at(t) -> FloatArray` returns `self.slots`; `slot_velocity(t) -> FloatArray` returns
  zeros (N, 3). (Static case.)

**Why D010** (pilot 2026-10-11, `docs/pilot-2026-10-11.md` §Tracking): with g_i measured to
the *moving* slot, an agent that rides its slot has g_i ≈ 0, so the T1 repulsion switches
off and the agent follows a slot that passes **through** an obstacle (S2: clearance −0.36 m);
an agent held back by an obstacle sees g_i grow and the repulsion and the damping blow up
(speeds 1e12 m/s, also with dt = 0.001 s). Measuring g_i to the final slot keeps the GNRON
fix exactly where it is needed (at the final formation), keeps U_r time-invariant (so the
∂U/∂t term of Remark 2 vanishes for U_r), and gave 75 % success and 0 % collisions on
S1–S4 (N = 5, seeds 0–2, a_max = 5 m/s² and unsaturated) versus 8 % / 92 %.

New `TrackingController(slots, edges, obstacles, path, config=None)` in
`src/uavapf/control/tracking.py`, re-exported from `uavapf.control`:
- has `.base` (an `APFController` built from the final slots), `.path`, `.config`,
  `.geometry`, `.slots` (final), `validate_state`, `components(p, v, *, slots=None)`
  delegating to `.base`;
- `slots_at(t) = path.position(t) + δ`, `slot_velocity(t) = path.velocity(t)` broadcast to (N, 3);
- `__call__(t, p, v)` implements Remark 2 exactly:

      e_i = v_i − ċ(t)
      u_i = c̈(t) + F_i(p; p*(t)) − K_d e_i − K_r [D(p; p*(t)) e]_i + ω_i(p; p*(t)) × e_i

  where F, D·e and ω are `base.components(p, e, slots=slots_at(t))`.
- With `path.start == path.goal` its output equals `APFController.__call__` exactly.

`StallReset` uses `controller.slots_at(t)` for the position test (‖p_i − p_i*(t)‖ ≥ ε_p);
the speed test stays on the absolute speed ‖v_i‖ ≤ ε_v (an agent held behind an obstacle
while the formation moves on is exactly the stall the reset is for). Static controllers
behave exactly as now. The reset direction keeps eq. (16) with p*(t).

`KhatibController` (B1) gets the same tracking treatment for a fair comparison: optional
argument `path: StraightPath | None = None`; with a path,
`u_i = c̈ − ∇U(p; p*(t)) − kd (v_i − ċ)`; `slots_at` / `slot_velocity` as above.

## 3. Scenario, generator, metrics, run

- YAML v1 gains an optional top-level block (strict keys):
  `reference: {start: [x, y, z], speed: 1.5, accel: 0.5}`. `Scenario.reference:
  StraightPath | None` with `goal` = centroid of the final slots (so δ = slots − centroid). Round trip as today.
  `Scenario.build_controller()` returns a `TrackingController` when `reference` is set.
- `GeneratorParams` gains `ref_speed: float | None = 1.5` (m/s) and `ref_accel: float = 0.5`
  (m/s²). When `ref_speed` is set, `make_scenario` sets `reference` from the centroid of the
  nominal start positions to the centroid of the slots (S5: per group the two centres are different, so S5 keeps
  static slots: `reference = None`), and `sim.t_final = path.duration + 20 s`.
- `evaluate_run(traj, slots, …)` accepts `slots` of shape (N, 3) (as now) **or** (K, N, 3)
  (slots per logged sample). Formation error uses the per-sample slots; completion time
  uses the last sample's slots for every sample (arrival at the final formation).
- `run_scenario` / `scripts/pilot.py` pass `controller.slots_at(traj.t)` stacked to
  (K, N, 3) to `evaluate_run`; the pilot gains `--static` to reproduce the old behaviour.

## 4. Acceptance (tests)
1. Path: endpoints, zero end velocities, limits respected (sampled), ċ and c̈ equal finite
   differences of c and ċ, constant before 0 and after T, L = 0 gives T = 0.
2. Static path ⇒ TrackingController ≡ APFController (bit-for-bit).
3. Remark-2 assembly against `components(..., slots=…)` on random states.
3b. D010: shifting `slots` changes the force by exactly k_a · shift **with obstacles present**
   (T1 and pair terms do not move with the slots).
3c. An agent whose reference passes straight through a sphere is repelled: clearance stays
   > 0 for the whole run (the failure that motivated D010).
4. No obstacles, p(0) = p*(0), v(0) = ċ(0): the closed loop follows p*(t) exactly
   (error < 1e-8 m over the whole run), also with Morse edges.
5. `components(p, v, slots=self.slots)` ≡ `components(p, v)`.
6. StallReset does not fire for an agent that rides close to the reference or moves with it;
   it fires for an agent standing still while its slot moves away.
7. YAML round trip with `reference`; generator sets it (S1–S4) and not for S5; t_final rule.
8. evaluate_run with (K, N, 3) slots matches a brute-force loop.
9. B1 with a path follows p*(t) exactly in free space (as 4).
