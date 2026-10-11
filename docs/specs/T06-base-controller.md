# T06 — Functional APF base controller

- Spec + implementation: Codex on the user's direct request · Review: independent agent
- Tests: existing tests remain untouched; verify analytic identities and a runnable demo
- Done when: equations (4)–(15) in `paper/main.tex` evaluate on a valid swarm state.

## 1. Scope
Implement the stationary-slot controller: attraction, goal-normalized
obstacle potential, Morse plus barrier pair potential, PSD-projected local Hessian
damping, gyroscopic force, and sampled hybrid reset. Use the existing `ObstacleSet`
geometry and RK4 plant. Baselines B1–B5, scenarios S1–S6, and PX4 are later tasks.

## 2. Formulas
`paper/main.tex` (4)–(15) is authoritative. `p`, `v`, `slots` have shape `(N, 3)`
and SI units m, m/s, m. Obstacle clearance is in m; active terms require `rho>0`.
Pair distances require `r>d_min`. Barrier applies to every unordered pair;
Morse applies only to graph edges. Damping uses one PSD projection per local
obstacle Hessian and the closed-form radial pair projection in (11)–(12).

## 3. Public API
`APFConfig` stores positive gains and radii; `APFController(slots, edges, obstacles,
config)` is callable as `controller(t, p, v) -> acceleration (N,3) in m/s²`.
`potential(p) -> float` returns the total nonnegative potential in m²/s²
under normalized mass. `components(p, v)` returns force and dissipation arrays
to support physical checks. Public arrays are float64.
`StallReset(controller, config)` is an explicit stateful callback passed to the
simulator's optional `reset(t, p, v) -> v_plus` hook after each RK4 step.

## 4. Edge cases
Reject invalid gains, radii, slots, graph edges, obstacles at slots, pairwise
slots with distance at/below `d_s`, overlapping agents, and obstacle contact.
Support one UAV, zero obstacles, and zero graph edges. The controller accepts
spheres and infinite vertical cylinders; capped cylinders need a separate
Hessian derivation and remain outside this task. At `rho=rho_0` all
obstacle terms except gyroscopic bias vanish; the activation is zero there.

## 5. Acceptance criteria
1. Goal-normalized force vanishes at a valid slot inside an influence region.
2. Pair forces are equal and opposite; barrier grows near `d_min`.
3. Analytic gradient matches finite differences away from boundaries.
4. `v·gyro=0` and `v·D v>=0` for representative states.
5. Controller runs in the existing simulator and produces finite trajectories.
6. Reset fires only after sustained stall, obeys dwell and count limits, and
   changes velocity by `Delta` along eq. (16) when a direction exists.

## 6. Notes for reviewer
The signs of `phi'`, barrier derivative, and pair force are easy to reverse.
Check that obstacle Hessians include the goal factor product rule. Paper safety
theorems assume unsaturated continuous flow and apply only under their stated
conditions; numerical RK4 with active saturation is not itself a proof.
