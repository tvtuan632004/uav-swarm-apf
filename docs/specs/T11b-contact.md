# T11b — Contact is an outcome: finite controller + safety metrics

- Branch: `T11b-contact`
- Spec + tests: Claude · Implementation: Codex · Review: Claude
- Tests: `tests/test_t11b_contact.py` (do not edit). The file skips itself until the new module
  exists; the task is done only when `uv run pytest -q` reports **0 skipped**.
- Done when: a head-on swap with a_max = 5 m/s² runs to the end (or to a collision
  stop) without raising, and the collision is reported by the safety metrics.
- Decision: D005 in `docs/decisions.md`. Origin: review finding S2 (T06).

## 1. Scope
In: (a) `APFController` stays finite when a state leaves the free set during a run;
(b) new module `src/uavapf/metrics/safety.py`.
Out: other metrics (T12), Monte-Carlo runner (T16).

## 2. Controller change (`src/uavapf/control/apf.py`)
- New field `APFConfig.contact_eps: float = 1e-3` (m), must be > 0 and
  `< min(rho1, d_s - d_min)`.
- `components(p, v)` (and therefore `__call__` and `potential`) must **not** raise
  when an agent is inside / on an obstacle (rho ≤ 0) or a pair is at r ≤ d_min.
  Use the clamped values

      rho_c = max(rho, contact_eps),        r_c = max(r, d_min + contact_eps)

  in every obstacle and pair term (potential, force, damping, omega). Normals and
  pair unit vectors keep their geometric direction; if r = 0 exactly, use +e_x.
- For states with rho ≥ contact_eps and r ≥ d_min + contact_eps the output is
  **bit-for-bit unchanged** (existing T06 tests must stay green).
- Shape / finiteness checks on p and v still raise `ValueError`.
- `APFController.__init__` checks stay strict (slots outside obstacles, slot pairs
  farther than d_s).

## 3. Safety metrics (`src/uavapf/metrics/safety.py`)

```python
def min_clearance(p: ArrayLike, obstacles: ObstacleSet) -> float: ...
def min_separation(p: ArrayLike) -> float: ...
def collision_mask(p, obstacles: ObstacleSet, d_min: float) -> NDArray[np.bool_]: ...
def collision_stop(obstacles: ObstacleSet, d_min: float) -> StopFn: ...
```

- `p` has shape `(N, 3)` (one state) **or** `(K, N, 3)` (a trajectory); both
  functions return the minimum over everything given, in m.
  `min_clearance` with zero obstacles → `+inf`; `min_separation` with N = 1 → `+inf`.
- `collision_mask(p, obstacles, d_min)`: for `(K, N, 3)` returns shape `(K,)`,
  for `(N, 3)` returns a 0-d bool array; True where any rho ≤ 0 or any r ≤ d_min
  (the complement of the free set F in main.tex §3.3).
- `collision_stop(obstacles, d_min)` returns `stop(t, p, v) -> bool`, True as soon
  as the state is in collision; pass it to `simulate(..., stop=...)`.
- Pure functions, vectorised (no Python loop over agents or time steps).
- Re-export the four names from `uavapf.metrics`.

## 4. Acceptance criteria
1. Controller finite (no exception, all outputs finite) for states with rho ∈
   {0.5·eps, 0, −0.3} m and pairs with r ∈ {d_min, d_min − 0.1} m.
2. Unchanged output in the free set (existing T06 tests + an explicit check).
3. Head-on swap, a_max = 5: `simulate` completes; with `collision_stop` it ends
   early and the last logged state is in collision.
4. Metrics match a brute-force loop on random trajectories; `+inf` edge cases.

## 5. Notes for the reviewer
Clamping changes the potential only outside F, so Theorem 1 (which lives in F) is
unaffected. Check that the clamp is applied before computing 1/rho and 1/(r−d_min),
not after.
