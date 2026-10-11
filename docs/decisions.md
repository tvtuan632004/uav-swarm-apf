# Design decisions

3–5 lines per entry, newest on top.

## 2026-10-11 · D008 · No GitHub: local git only
The user does not want a remote. History lives in local git; the human commits between
agent turns. Verification = `scripts\setup.cmd` on Windows + Claude's Linux run.
`.github/workflows/ci.yml` stays in the repo (harmless, ready if a remote is added later).

## 2026-10-11 · D007 · Agents take turns; Codex runs in an isolated environment
Codex's Windows sandbox created a `.venv` and a `%TEMP%\pytest-of-Admin` that the user's
account could not use. Codex now sets `UV_PROJECT_ENVIRONMENT=.venv-codex` and
`PYTEST_DEBUG_TEMPROOT=.pytest_tmp/codex`. Only one agent works in the folder at a
time; every turn ends with a handoff entry and a git commit by the user.

## 2026-10-11 · D006 · Two saturation regimes in every experiment
Proposition 2 needs a_max ≥ ū(c̄); Table 4's a_max = 5 m/s² can violate it (head-on
swap). Layer-1 experiments run (a) unsaturated, to check the theory, and (b) with
a_max = 5 m/s², reporting the fraction of time saturated and collisions. An a_max
sweep {2, 5, 10, ∞} m/s² is added to the ablation table (T18).

## 2026-10-11 · D005 · Contact is an outcome, not an exception
Inside a run the controller never raises on contact: it evaluates rho and r − d_min
clamped at `contact_eps` (default 1e-3 m) and stays finite. Collisions are detected
by `uavapf.metrics.safety` and end the run through a stop function, so one colliding
run cannot abort a Monte-Carlo batch. Construction-time checks stay strict (T11b).

## 2026-10-11 · D004 · Norm saturation for the plant
Use per-agent Euclidean-norm acceleration saturation, matching the existing
simulator, T03 spec, and README. The user delegated this choice after the PDF
draft used elementwise saturation. `paper/main.tex` now defines norm saturation
in (1) and uses the same norm in Proposition 2.

## 2026-10-11 · D003 · Obstacle primitives: sphere + vertical capped cylinder
Two primitives cover S1–S5 (trees/poles/buildings = cylinders, balloons/blobs =
spheres). Cylinders are z-aligned with optional infinite ends, and ρ is the exact
signed distance, so n̂ = ∇ρ is C¹ everywhere outside. Degenerate normals (centre,
axis) are fixed unit vectors (+e_z, +e_x) so repulsion never vanishes.

## 2026-10-11 · D002 · Fixed-step RK4, continuous-time control
The plant is integrated with fixed-step RK4 and the controller is evaluated at
every stage (no zero-order hold). Fixed steps keep Monte-Carlo runs comparable
and reproducible; adaptive stepping is out of scope unless T11 shows stiffness.
Speed limit is a post-step projection and is disabled in energy tests.

## 2026-10-11 · D001 · Results live outside the repo
Simulation output is large and machine-specific. All writers resolve their output
root through `uavapf.paths.results_root()`, controlled by env var `UAVAPF_RESULTS`
(point it at the HDD). Default `./results/`, which is git-ignored.

## 2026-10-11 · D000 · Single source of truth for formulas
`paper/main.tex` defines every equation. Code cites equation numbers; tests are
derived from the paper, not from the implementation. A mismatch is reported in
the handoff log, never silently "fixed" on one side.
