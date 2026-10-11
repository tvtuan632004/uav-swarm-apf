# T11c — Controller speed-up (sparse evaluation)

- Spec + tests + implementation: Claude (overnight, on the user's request) · Review: **Codex**
- Tests: `tests/test_t11c_speed.py` · Benchmark: `scripts/bench_controller.py`
- Done when: N = 50, M = 20 ≤ 1 ms per controller call, and outputs equal the dense
  (T06/T11b) evaluation to rtol 1e-11.

## 1. Idea
Every obstacle term (phi, its derivatives, beta) is identically zero for rho > rho0, and
every pair term is zero unless the pair is a formation edge (Morse) or r ≤ d_s
(barrier). So only the *active* agent–obstacle pairs and *selected* agent–agent pairs
are evaluated; everything else is skipped exactly, not approximated.

## 2. Changes (`src/uavapf/control/apf.py`)
- Clearances via `ObstacleSet.clearance` (no normals); normals only for active pairs.
- One 3×3 `eigh` per active pair; `[Q]_+ v = U max(w,0) Uᵀ v` without forming `[Q]_+`.
- `np.add.at` → `np.bincount` scatter; `np.cross` → explicit row-wise cross.
- New public attribute `APFController.edges` (sorted pairs i < j).

## 3. Acceptance
1. `test_sparse_equals_dense` on N ∈ {1, 4, 12, 30, 50}: potential, force, damping,
   gyroscopic, omega equal to the frozen dense reference (rtol 1e-11).
2. Same outside the free set (clamped states, T11b).
3. Faster than the dense reference by ≥ 40 % (loose guard); absolute numbers from
   the benchmark script (Linux, this session: 2.4 → 0.38 ms for N = 50, M = 20).
