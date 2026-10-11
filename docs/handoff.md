# Handoff log

Newest entry on top. Every agent adds one entry before ending a session
(format in AGENTS.md §5).

## 2026-10-11 (night) · T05, T06, T11b, T11c, T12, T13-draft, pilot · Claude → Codex (review)
- Context: the user went to sleep and asked Claude to do as much as possible; Codex was
  not running, so Claude wrote spec, tests AND code for every task below. **Each needs
  Codex's independent review** (prompt in docs/plan.md §8); add tests in new files only.
- Done:
  - T11b `metrics/safety.py` (min_clearance, min_separation, collision_mask,
    collision_stop), `APFConfig.contact_eps`, clamp in `components`, public
    `validate_state`, `ObstacleSet.clearance` (+ tests/test_t04_clearance.py).
  - T05 `scenarios/io.py` (strict YAML v1), `scenarios/run.py` (+ CLI
    `scripts/run_scenario.py`), `viz.py` (plots, GIF with time stamp).
  - T11c sparse controller (active pairs only, bincount scatter): N=50, M=20
    2.4 → 0.38–0.44 ms/call; equals the frozen dense version (tests/test_t11c_speed.py).
    Benchmark: `scripts/bench_controller.py`.
  - T12 `metrics/run.py`: evaluate_run (9 fields) + controller_cost.
  - T06 B1 `baselines/khatib.py`: GNRON offset 0.435 m (APF 0.000), symmetric trap
    stalls at x = −1.35 m (APF escapes by gyro alone, t = 12.3 s).
  - T13 draft `scenarios/generate.py`: S1–S5 with variants, formations line/wedge/grid,
    `GeneratorParams`; all 240 combinations (×3 seeds) generate and validate.
  - Pilot `scripts/pilot.py` + `docs/pilot-2026-10-11.md` (168 runs).
  - Docs: plan v2.1, D008 (no GitHub), AGENTS.md rules 1 and 5 for local git, README.
- Tests: 295/295 green (`uv run pytest -q`, Linux), ruff check + format clean. Every new
  test file was mutation-checked (injected bugs fail it). Not yet run on Windows.
- Open issues (need the user, see plan §6): Q1 approach speed — with static far slots the
  swarm reaches ~10 m/s and with a_max = 5 almost every run collides (both methods); Q2
  RK4 dt vs the stiff barrier (S5 groups unsaturated blew up numerically); Q3 B1 pair
  repulsion on r vs r − d_min; Q4 T13 geometry parameters. Khatib FIRAS form should be
  checked against the original paper (cited by name, no equation number).
- Next: user runs setup.cmd (expect 295 passed) and commits; Codex reviews the night's
  work; user answers Q1–Q4; then T13b (moving centre) if Q1(a).

## 2026-10-11 · Plan v2, T11b + T05 specs · Claude → Codex
- Done: rewrote the plan as `docs/plan.md` (task table with status, schedule to
  31/1/2027, roles, prompts). Decisions D005 (contact is an outcome), D006 (two
  saturation regimes + a_max sweep), D007 (turn-taking, Codex in `.venv-codex` and
  `.pytest_tmp/codex`); AGENTS.md updated (rule 9, section 5). Specs + tests:
  `docs/specs/T11b-contact.md` + `tests/test_t11b_contact.py` (15 tests),
  `docs/specs/T05-scenarios-yaml.md` + `tests/test_t05_scenarios.py` (41 tests) +
  fixture `configs/scenarios/demo_sphere.yaml`. T06 PSD test raised to 10,000 states.
- Tests: on the current code 111 passed, 2 files skipped (new modules missing). Both
  new suites were validated on Linux against a throwaway implementation that is NOT
  in the repo (56/56 green) and catch the obvious mutations (e.g. missing clamp).
- Open issues: none blocking. T11b must land before T05 (T05 uses collision_stop).
- Next: Codex implements T11b, then T05 (prompts in docs/plan.md section 8). Done =
  0 failed, 0 skipped. Then Claude reviews.

## 2026-10-11 · T06 S1 fix · Claude → Codex (review)
- Done: Codex had stopped, so on the user's request Claude fixed S1 in
  `src/uavapf/control/apf.py` (edge validation): indices go through `operator.index`,
  so `int` and NumPy integers are accepted; floats, bools and pairs of the wrong
  length are still rejected with `ValueError`. No other code or test changed.
- Tests: 111/111 green, ruff check + format clean (Linux). User's Windows run
  before the fix: 110/111 (only S1 red).
- Open issues: S2 (contact raises inside RK4) and S3 (a_max vs Proposition 2) are
  still open and wait for the user's decision. Codex should review this small fix
  (Claude wrote the test and the fix, so it is not cross-checked yet).
- Next: user runs scripts/setup.cmd → expect 111/111 and "ALL DONE".

## 2026-10-11 · T06 review · Claude → Codex
- Done: independent review of `src/uavapf/control/` against
  `docs/specs/T06-base-controller.md` and `paper/main.tex` (4)–(16), plus
  `tests/test_t06_apf.py` (30 tests, written from a loop-based reference transcribed
  from main.tex, not from the implementation). Mutation-checked: 12/12 injected bugs
  (Hessian cross terms, phi'' coefficient, projection, signs, activation, barrier
  scope, reset direction/dwell/count/stall clock) make the suite fail.
  Restored Claude's `scripts/setup.ps1` health check and README rows that were
  overwritten while both agents wrote to this folder at 00:29–00:30.
- Tests: 110/111 green (`uv run pytest -q`, Linux). The red one is finding S1.
  Verified: potential = reference; force = −FD gradient (rtol 1e-6); D·v = numerically
  differentiated local Hessians, PSD-projected (rtol 2e-4); vᵀDv ≥ 0 on 2000 states;
  gyro does no work; V = ½|v|² + U nonincreasing on the unsaturated demo.
- Review findings:
  - blocking: none. Signs, units and every formula in (4)–(16) match main.tex.
  - should fix S1 `src/uavapf/control/apf.py:121` — edges given as NumPy integers
    are rejected (`isinstance(i, int)`); use `operator.index`. Test:
    `test_numpy_integer_edges_accepted`.
  - should fix S2 `src/uavapf/control/apf.py:179,232` — contact raises `ValueError`
    inside RK4 stages, so one colliding run aborts a Monte-Carlo batch. Reproduced:
    head-on swap of 2 UAVs with a_max = 5 m/s² (Table 4 value) raises at t≈2 s.
    Proposal (needs user OK, then D005): contact is an outcome, not an exception —
    `components` clamps rho and r − d_min at a small epsilon and stays finite, and a
    `collision` stop/metric records it (T12/T16). Keep the strict check in `__init__`.
  - should fix S3 (paper, not code) — Proposition 2 needs a_max ≥ ū(c̄); the head-on
    case shows Table 4's a_max = 5 m/s² can violate it. Either report saturation as
    an ablation or state ū for the tested scenarios.
  - nit N1 `apf.py:215` — eigh runs on all N×M pairs, including inactive (rho > rho0)
    ones; mask active pairs first. N=50, M=20: 2.6 ms/call ≈ 63 s per 60 s run.
  - nit N2 `apf.py:250` — `np.add.at` is slow; `np.bincount`/`np.subtract.at` on
    flattened indices or a dense (N,N) accumulation is faster.
  - nit N3 `apf.py:203` — Hessian of rho is not an equation in main.tex; cite it as
    "derived from eq. (2)" or add it to the appendix.
  - nit N4 `src/uavapf/control/reset.py:72` — uses private `controller._state`;
    make a public `validate_state`.
- Verdict: changes requested (S1 required; S2 before T16; S3 for the paper).
- Open issues: two agents wrote to the same folder concurrently (README.md and
  setup.ps1 were overwritten). Proposal: Codex uses `UV_PROJECT_ENVIRONMENT=.venv-codex`
  and agents take turns; git init before the next task so overwrites show in diffs.
- Next: Codex fixes S1 (and S2 if the user approves the proposal), runs
  `uv run pytest -q` until 111/111, adds a handoff entry. Then T05 scenarios.

## 2026-10-11 · T06 base · Codex → Claude (review)
- Done: restored `paper/main.tex` and bibliography from the research ZIP; reconciled
  eq. (1) and Proposition 2 to per-agent norm saturation (user-delegated choice);
  implemented the stationary-slot T1–T4 controller (eqs. 4–15), sampled reset
  (eq. 16), simulator reset hook, and `scripts/base_demo.py`. Moved CI to
  `.github/workflows/ci.yml`; updated README and D004.
- Tests: 81/81 green (`uv run pytest -q --tb=short`, with
  `PYTEST_DEBUG_TEMPROOT=C:\Setup\UAV\results` because the host's default pytest
  temp directory denies access); `uv run ruff check .` and
  `uv run ruff format --check .` green. Manual gradient error 2.3e-8 near the
  pair barrier, gyroscopic work ~0, PSD damping power nonnegative; demo final
  max slot error 0.042 m, minimum obstacle clearance 0.872 m, minimum pair
  distance 1.520 m, and strictly decreasing logged energy.
- Open issues: this directory has no `.git`, so no task branch, PR, or two-OS CI
  run exists yet. New controller/reset code has no independent committed tests
  (implementer must not edit `tests/`). Capped-cylinder geometry in T04 is an
  extension beyond the paper's sphere/infinite-cylinder Hessian, so the controller
  rejects capped cylinders. Active acceleration saturation can invalidate the
  paper's safety conditions; a 5 m/s² cap caused contact in a direct-crossing
  trial. The supplied PDF still has the old elementwise wording; the corrected
  `paper/main.tex` is now the formula source.
- Next: independent review against `docs/specs/T06-base-controller.md` and
  `paper/main.tex`, add new tests for T1–T4 and reset, then create a Git repository
  and PR once a remote is available. Follow with T05 scenarios and metrics.

## 2026-10-11 · T01, T03, T04 · Claude → Codex (review)
- Done: on the user's request Claude implemented T01 (repo, uv, CI, paths.py),
  T03 (src/uavapf/sim: RK4, saturation, Trajectory .npz I/O) and T04
  (src/uavapf/geometry: sphere/capped-cylinder rho and n-hat), each with spec and
  tests; plus README.md, scripts/setup.ps1, scripts/setup.cmd, scripts/demo.py,
  decisions D002–D003.
- Tests: 81/81 green, ruff clean (uv run pytest -q, Linux). Energy drift 9e-11
  (criterion 1e-6). Every test file was mutation-checked: deliberate bugs make it fail.
- Open issues: roles swapped vs. the plan (Claude wrote code AND tests), so Codex
  must review with fresh eyes and may ADD tests (not edit/delete existing ones).
  `paper/main.tex` still missing: eq. labels (M1), (M2), (G1), (G2) live in the specs
  until the paper defines them.
- Next: Codex reviews T01, T03, T04 against their specs (format: AGENTS.md §6).
  Then Claude writes spec + tests for T05/T06.

## 2026-10-11 · T01 · Claude → Codex
- Done: AGENTS.md, CLAUDE.md, docs/specs/T01-repo-setup.md, docs/specs/_template.md,
  docs/decisions.md, tests/test_t01_repo.py (26 tests).
- Tests: checked against a throwaway reference implementation on Linux, all green.
  Not run on Windows yet.
- Open issues: none. `paper/main.tex` not in the repo yet (needed from T03 on).
- Next: Codex implements T01 on branch `T01-repo-setup` and opens a PR.
