# Notes for paper/main.tex (proposals, not applied)

## Remark 2 (Moving formation), after D010 — proposed wording

> For a moving centre c(t), the slots p_i*(t) = c(t) + δ_i are tracked by the attraction
> and the gyroscopic goal term, while the goal factor of the obstacle potential keeps the
> final slots, g_i = ‖p_i − p_i^goal‖. The law becomes
> u_i = p̈_i* − ∇_{p_i}U − K_d e_i − K_r [D e]_i + ω_i × e_i with e_i = v_i − ṗ_i*.
> U_r is then time-invariant; with V = ½‖e‖² + U the only cross term is
> Σ_i ∇_{p_i}U_r · ċ, which vanishes outside obstacle influence regions and once the
> reference stops (t ≥ T), after which the closed loop is exactly the static controller
> and Theorems 1–2 apply from the state reached at T.

Why (simulation evidence, docs/pilot-2026-10-11.md §Tracking): with g_i measured to the
moving slot, an agent riding its slot has g_i ≈ 0, which switches T1 off, so it followed
its slot through an obstacle; agents held back by an obstacle saw g_i^n grow without bound.

## Limitation sentence to keep honest
The safety theorem holds in continuous time; the RK4 simulation at dt = 0.01 s does not
resolve the barrier when two groups close at ~20 m/s (S5 two groups, static slots).
