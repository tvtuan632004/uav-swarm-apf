# T06 — Baseline B1: Khatib APF with formation attraction and pairwise repulsion

- Spec + tests + implementation: Claude (overnight, on the user's request) · Review: **Codex**
- Tests: `tests/test_t06b_khatib.py`
- Done when: B1 reproduces the GNRON offset and the symmetric local-minimum trap, and
  the proposed APF fixes both in the same scenes (main.tex §6.3, B1; deficiencies D1, D2).

## 1. Model
Same plant and units as the proposed controller. With g_i = p_i − p_i*, clearance rho_ik
and pair distance r_ij:

    U_a = Σ_i ka/2 ‖p_i − p_i*‖²                                       (attraction, eq. (4))
    U_o = Σ_i Σ_k ½ η (1/rho_ik − 1/rho0)²     for rho_ik ≤ rho0, else 0   (Khatib FIRAS)
    U_p = Σ_{i<j} ½ k_b (1/r_ij − 1/d_s)²      for r_ij ≤ d_s, else 0       (pair repulsion)
    u_i = −∇_{p_i}(U_a + U_o + U_p) − K_d v_i

The obstacle term is the FIRAS potential of Khatib (1986): note **no goal factor**, so the
repulsion does not vanish at a slot inside rho0 (GNRON), and there is no gyroscopic force
or reset (local minima). The pair term is the same Khatib form on r (not r − d_min).
*Reviewer: check the FIRAS form and constants against the original paper, Section 3
(AGENTS.md §3); the code cites "Khatib 1986, FIRAS".*

## 2. API (`uavapf.baselines`)

```python
@dataclass(frozen=True, slots=True)
class KhatibConfig:
    ka: float = 1.0  # 1/s²   (Table 4, shared with the proposed method)
    eta: float = 1.0  # m^4/s² … keeps the Table-4 value for a like-for-like start
    rho0: float = 2.0  # m
    kb: float = 0.5
    d_s: float = 1.2  # m
    kd: float = 1.5  # 1/s
    contact_eps: float = 1e-3  # m, clamp outside the free set (D005)


class KhatibController:
    def __init__(self, slots, obstacles, config=None): ...
    def potential(self, p) -> float: ...  # m²/s²
    def __call__(self, t, p, v) -> FloatArray: ...  # m/s², shape (N, 3)
```

Spheres and cylinders of any kind (capped too) are allowed: only rho and n̂ are needed.
Validation: positive finite gains, slots (N, 3) finite and outside every obstacle,
contact_eps < min(rho0, d_s). States with rho ≤ contact_eps or r ≤ contact_eps are
evaluated at the clamp (finite, D005); coincident agents use +e_x.

## 3. Acceptance
1. Force = −finite-difference gradient of an independent loop implementation of U.
2. Obstacle term zero beyond rho0, pair term zero beyond d_s; pair forces cancel.
3. **GNRON**: one UAV, slot 0.6 m from a sphere surface (inside rho0): B1 settles
   ≥ 0.1 m away from the slot; the proposed APF settles within 0.01 m.
4. **Local minimum**: slot directly behind a sphere: B1 stalls in front of it (never
   passes the obstacle centre plane); APF + StallReset reaches the slot (< 0.1 m).
5. Finite at contact; invalid configs raise `ValueError`.
