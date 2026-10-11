# T13 — Seeded scenario generators S1–S5 (draft v0)

- Spec + tests + implementation: Claude (overnight, on the user's request) · Review: **Codex**
- Tests: `tests/test_t13_generate.py`
- Done when: the same (scenario, N, formation, seed) always gives the same `Scenario`,
  every generated scenario is feasible, and each one exhibits the deficiency it probes
  (main.tex Table "Evaluation scenarios").
- **Draft:** geometry sizes and densities below are first guesses, collected in
  `GeneratorParams`; they need the user's (and the advisor's) confirmation before T17.

## 1. Common layout
Travel along +x at altitude z0 = 5 m. Formation centre starts at (0, 0, z0); the goal
centre is (L + D, 0, z0), where D is the formation depth along x, so a free gap of
L = 20 m separates the start formation's front from the goal formation's back; S1, S3
and S4 place their obstacles in that gap. Slots = goal centre + formation offsets; start positions = start
centre + the same offsets + N(0, σ = 0.2 m) per axis (re-drawn until the start is in the
free set). Velocities start at zero. Obstacles are infinite vertical cylinders unless
stated. `seed` fully determines the scenario through `numpy.random.default_rng`.

Formations (spacing s = 2 m > d_s, so Assumption 1 holds):
- `line`: across the direction of travel (along y), edges = chain.
- `wedge`: leader in front, two arms at ±45° behind, edges = arms (chain from leader).
- `grid`: ⌈√N⌉ columns along y, rows behind along −x, edges = 4-neighbour grid.

## 2. Scenarios

| ID | Construction | Property checked by tests |
|---|---|---|
| S1 | Forest: Poisson number of cylinders, density ∈ {sparse 3, medium 6, dense 10} per 100 m², radius U(0.3, 0.6) m, in the gap (2 m inside it), |y| ≤ half-width + 4 m; ≥ 1.0 m surface gap between trees and ≥ 1.0 m clearance to every start and slot | at least one tree is on some agent's straight path |
| S2 | For ⌈N/2⌉ random agents, a cylinder or sphere whose surface is 0.4–1.5 m from the slot (inside rho0) | those slots lie inside rho0 of an obstacle and outside every obstacle |
| S3 | `line`: sphere radius 1.5 m centred on the centre line in the middle of the gap. `u`: concave U of overlapping cylinders (r = 0.4 m, pitch 0.7 m), opening towards the start, centred on the path, inner width = formation width + 3 m, depth 4 m | the straight start→slot segment of at least one agent intersects the obstacle |
| S4 | Wall across the path in the middle of the gap with a corridor of length 8 m and width w = max(3 m, 0.5 · formation width), built from overlapping cylinders | w < formation width; no straight path for the outer agents |
| S5 | `groups`: two groups head-on, group A (⌈N/2⌉) goes +x, group B goes −x, line formations at y-offsets that overlap. `swap`: agents on a horizontal circle of radius max(5, s / (2 sin(π/N))) (neighbours ≥ s apart) swap to the antipodal point | straight paths of the two groups cross / all paths pass the circle centre |

`GeneratorParams` holds every number above. Defaults for `sim`: dt = 0.01 s,
t_final = 60 s, a_max = 5 m/s² (D006 realistic regime), log_every = 1.

## 3. API (`uavapf.scenarios.generate`, re-exported from `uavapf.scenarios`)

```python
FORMATIONS = ("line", "wedge", "grid")
SCENARIOS = ("S1", "S2", "S3", "S4", "S5")


def formation_offsets(kind: str, n: int, spacing: float = 2.0) -> tuple[FloatArray, tuple]: ...
def make_scenario(
    scenario_id, n, seed, *, formation="wedge", variant=None, params=None
) -> Scenario: ...
```

Variants: S1 → "sparse" | "medium" (default) | "dense"; S3 → "line" (default) | "u";
S5 → "groups" (default) | "swap"; others take none. Name = f"{id}-{variant}-{shape}-N{n}-s{seed}" (shape = formation; "line" / "ring" for S5).
Invalid id / formation / variant / n < 1 → `ValueError`.

## 4. Open questions for the user
1. L = 20 m: with the static-slot quadratic attraction (main.tex Assumption 3) the initial
   command is k_a·L = 20 m/s², so the unsaturated regime of D006 is very aggressive.
   Either keep L small, or move the formation centre (Remark "tracking").
2. Densities and corridor ratio are guesses; S6 (Layer 2) is out of scope here.
