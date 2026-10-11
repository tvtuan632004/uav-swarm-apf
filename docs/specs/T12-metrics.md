# T12 — Evaluation metrics (Table "Evaluation metrics", main.tex §6.4)

- Spec + tests + implementation: Claude (overnight, on the user's request) · Review: **Codex**
- Tests: `tests/test_t12_metrics.py`
- Done when: every metric matches a hand computation on synthetic trajectories.

## 1. Definitions (per run)
A run is a `Trajectory` with samples t_0 < … < t_K, positions p, velocities v and logged
(saturated) commands u. T = t_K − t_0. Time integrals use the trapezoidal rule on the
logged samples, so their accuracy follows `log_every`; use `log_every = 1` for the
paper's numbers.

| Field | Definition | Unit |
|---|---|---|
| `collided` | any sample outside the free set F (T11b) | bool |
| `min_separation` | min over t, i ≠ j of r_ij | m |
| `min_clearance` | min over t, i, k of rho_ik | m |
| `completion_time` | first t with ‖p_i − p_i*‖ ≤ ε_p for **all** i; `inf` if never | s |
| `success` | not `collided` and `completion_time ≤ t_max` (t_max defaults to t_K) | bool |
| `formation_error` | e_F = (1/T) ∫ (1/N) Σ_i ‖p_i − p_i*‖ dt | m |
| `control_effort` | J_u = ∫ Σ_i ‖u_i‖² dt | m²/s³ |
| `rms_jerk` | sqrt( (1/(N T)) ∫ Σ_i ‖u̇_i‖² dt ), u̇ by finite differences of u between samples (piecewise-constant on each interval) | m/s³ |
| `saturation_fraction` | (1/(N T)) ∫ Σ_i 1[‖u_i‖ ≥ (1 − 1e-9) a_max] dt; 0 if a_max is None | 1 |

Notes: the formation centre is stationary (Assumption 3), so p_i* is the slot.
Success uses the first hitting time, as in the paper's wording ("within ε_p before T_max").

## 2. API (`uavapf.metrics`)

```python
@dataclass(frozen=True, slots=True)
class RunMetrics:
    success: bool
    collided: bool
    completion_time: float
    min_separation: float
    min_clearance: float
    formation_error: float
    control_effort: float
    rms_jerk: float
    saturation_fraction: float

    def as_dict(self) -> dict[str, float | bool]: ...


def evaluate_run(traj, slots, obstacles, *, d_min, eps_p, a_max=None, t_max=None) -> RunMetrics: ...
def controller_cost(controller, p, v, *, reps=200, warmup=10) -> tuple[float, float]: ...
```

`controller_cost` returns (mean, 99th percentile) wall time per control step **per
agent** in µs (the paper's computational-cost metric), timing `controller(0, p, v)`.

Validation (`ValueError`): slots shape ≠ (N, 3) of the trajectory, fewer than 2 samples,
eps_p ≤ 0, d_min ≤ 0, a_max ≤ 0.

## 3. Acceptance
Hand-computed values for: straight-line constant-acceleration motion (e_F, J_u, jerk 0),
a linear ramp of u (constant jerk), completion and success with t_max, saturation half
of the time, collisions; brute-force loop agreement on a random trajectory; dict keys.
