# T04 — Obstacle geometry: distance ρ and outward normal n̂

- Branch: `T04-geometry`
- Spec + tests: Claude · Implementation: Claude (on the user's request) · Review: **Codex**
- Tests: `tests/test_t04_geometry.py`
- Done when: n̂ matches the central finite-difference gradient of ρ.

## 1. Scope
Signed distance ρ (m) from points to obstacle surfaces and the unit outward
normal n̂ = ∇ρ, for spheres and vertical cylinders (optionally capped).
Vectorised over N points × M obstacles. Repulsive forces are T06/T07.

## 2. Formulas
Sphere, centre c, radius r:

    ρ(p) = ‖p − c‖ − r,      n̂(p) = (p − c) / ‖p − c‖                    (G1)

Vertical cylinder, axis through (c_x, c_y) parallel to z, radius r, extent
z ∈ [z_min, z_max] (either bound may be infinite). With q = (p_x − c_x, p_y − c_y),
e_r = (q / ‖q‖, 0), s = sign of the nearer cap (+1 above, −1 below):

    d_r = ‖q‖ − r,     d_z = max(z_min − p_z, p_z − z_max)
    outside (d_r > 0 or d_z > 0):
        ρ = √(max(d_r,0)² + max(d_z,0)²)
        n̂ = (max(d_r,0)·e_r + max(d_z,0)·s·e_z) / ρ
    inside (d_r ≤ 0 and d_z ≤ 0):
        ρ = max(d_r, d_z),  n̂ = e_r if d_r ≥ d_z else s·e_z                  (G2)

ρ is the exact signed distance (negative inside). For an infinite cylinder
(z_min = −∞, z_max = +∞) (G2) reduces to ρ = ‖q‖ − r, n̂ = e_r.

Degenerate directions (∇ρ undefined): at the sphere centre n̂ = +e_z; on the
cylinder axis e_r = +e_x. n̂ is always a unit vector and always finite.

## 3. Public API (`uavapf.geometry`)

```python
@dataclass(frozen=True, slots=True)
class Sphere:
    center: tuple[float, float, float]  # m
    radius: float  # m, > 0


@dataclass(frozen=True, slots=True)
class Cylinder:
    center_xy: tuple[float, float]  # m
    radius: float  # m, > 0
    z_min: float = -inf  # m
    z_max: float = +inf  # m, z_max > z_min


def sphere_distance(points, centers, radii) -> tuple[rho, n]: ...
def cylinder_distance(points, centers_xy, radii, z_min, z_max) -> tuple[rho, n]: ...


class ObstacleSet:
    def __init__(self, obstacles: Sequence[Sphere | Cylinder]): ...
    def __len__(self) -> int: ...
    def distance(self, points) -> tuple[rho, n]: ...
```

Shapes: points (N, 3); per-type parameter arrays have leading dim M;
returns ρ (N, M) and n̂ (N, M, 3), float64. `ObstacleSet.distance` keeps the
order of the obstacles as given (column j ↔ obstacles[j]); an empty set returns
shapes (N, 0) and (N, 0, 3).

## 4. Edge cases / validation (`ValueError`)
- radius ≤ 0, non-finite centre, z_max ≤ z_min, wrong point shape.

## 5. Acceptance criteria
1. ‖n̂‖ = 1 everywhere (including degenerate points).
2. n̂ = central-difference gradient of ρ (h = 1e-6, rtol 1e-6) at random points
   away from the non-smooth set (axis, centre, medial set, cylinder rim edges).
3. Hand-checked values: inside, outside, side, cap, rim corner, infinite cylinder.
4. `ObstacleSet` matches a per-obstacle loop; mixed order preserved.

## 6. Notes for the reviewer
Sign convention: ρ > 0 outside, n̂ points **away** from the obstacle, so a
repulsive force is along +n̂.
