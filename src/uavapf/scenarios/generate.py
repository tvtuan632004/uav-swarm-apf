"""Seeded generators for scenarios S1-S5 (draft v0; spec: docs/specs/T13-scenarios.md).

Every number that shapes a scenario lives in :class:`GeneratorParams` so it can be tuned
in one place. The same (scenario, n, formation, variant, seed, params) always produces
the same :class:`~uavapf.scenarios.io.Scenario`.
"""

from __future__ import annotations

import math
import zlib
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from uavapf.control import APFConfig, StraightPath
from uavapf.geometry import Cylinder, ObstacleSet, Sphere
from uavapf.scenarios.io import Scenario, _check_free_start
from uavapf.sim import SimConfig

FloatArray = NDArray[np.float64]
Edges = tuple[tuple[int, int], ...]

FORMATIONS = ("line", "wedge", "grid")
SCENARIOS = ("S1", "S2", "S3", "S4", "S5")
VARIANTS = {
    "S1": ("medium", "sparse", "dense"),
    "S2": (None,),
    "S3": ("line", "u"),
    "S4": (None,),
    "S5": ("groups", "swap"),
}


@dataclass(frozen=True, slots=True)
class GeneratorParams:
    """All geometry knobs of S1-S5, in m unless noted (draft values, see spec §4)."""

    length: float = 20.0  # free gap between the start formation's front and the goal's back
    altitude: float = 5.0
    spacing: float = 2.0  # formation spacing s (> d_s = 1.2 m)
    start_noise: float = 0.2  # std of the start perturbation per axis
    # S1 forest
    densities: dict[str, float] = field(
        default_factory=lambda: {"sparse": 3.0, "medium": 6.0, "dense": 10.0}
    )  # trees per 100 m^2
    tree_radius: tuple[float, float] = (0.3, 0.6)
    tree_gap: float = 1.0  # min surface gap between trees
    keep_clear: float = 1.0  # min clearance of starts and slots to any obstacle (S1, S3, S4)
    margin_y: float = 4.0  # forest extends this far beyond the formation half-width
    # S2 slots inside influence regions
    s2_clearance: tuple[float, float] = (0.4, 1.5)
    s2_radius: tuple[float, float] = (0.4, 0.8)
    # S3 local minima
    s3_sphere_radius: float = 1.5
    u_cyl_radius: float = 0.4
    u_pitch: float = 0.7
    u_depth: float = 4.0
    u_extra_width: float = 3.0
    # S4 corridor
    corridor_min_width: float = 3.0
    corridor_ratio: float = 0.5
    corridor_length: float = 8.0
    wall_radius: float = 0.5
    wall_pitch: float = 0.8
    wall_extent: float = 6.0  # beyond the formation half-width
    # S5
    swap_min_radius: float = 5.0
    # integration (D006 realistic regime)
    dt: float = 0.01
    t_final: float = 60.0  # s; with a reference: path duration + ref_settle instead
    a_max: float | None = 5.0
    # moving formation centre (T13b, D009); ref_speed None -> static slots
    ref_speed: float | None = 1.5  # m/s
    ref_accel: float = 0.5  # m/s²
    ref_settle: float = 20.0  # s after the reference stops


# --------------------------------------------------------------------------- #
# Formations
# --------------------------------------------------------------------------- #
def formation_offsets(kind: str, n: int, spacing: float = 2.0) -> tuple[FloatArray, Edges]:
    """Slot offsets (N, 3) about the formation centre (travel along +x) and graph edges."""
    if n < 1:
        raise ValueError("n must be >= 1")
    off = np.zeros((n, 3))
    edges: list[tuple[int, int]] = []
    if kind == "line":
        off[:, 1] = spacing * (np.arange(n) - (n - 1) / 2)
        edges = [(i, i + 1) for i in range(n - 1)]
    elif kind == "wedge":
        c = spacing / math.sqrt(2.0)
        for k in range(1, n):
            row, side = (k + 1) // 2, (1.0 if k % 2 else -1.0)
            off[k] = (-row * c, side * row * c, 0.0)
            edges.append((k - 2 if k >= 3 else 0, k))
        off[:, 0] -= off[:, 0].mean()
    elif kind == "grid":
        cols = math.ceil(math.sqrt(n))
        for i in range(n):
            row, col = divmod(i, cols)
            off[i] = (-row * spacing, (col - (cols - 1) / 2) * spacing, 0.0)
            if col > 0:
                edges.append((i - 1, i))
            if row > 0:
                edges.append((i - cols, i))
        off[:, 0] -= off[:, 0].mean()
    else:
        raise ValueError(f"formation must be one of {FORMATIONS}, got {kind!r}")
    return off, tuple(sorted((min(e), max(e)) for e in edges))


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def _segment_point_distance_xy(a: FloatArray, b: FloatArray, c: FloatArray) -> FloatArray:
    """Horizontal distance from point c (2,) to segments a[i]->b[i] (N, 2)."""
    ab = b - a
    denom = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12)
    t = np.clip(np.einsum("ij,ij->i", c - a, ab) / denom, 0.0, 1.0)
    closest = a + t[:, None] * ab
    return np.linalg.norm(closest - c, axis=1)


def blocked_paths(p0: FloatArray, slots: FloatArray, obstacles) -> NDArray[np.bool_]:
    """For each agent: does the straight horizontal start->slot segment hit an obstacle?"""
    hit = np.zeros(len(p0), dtype=bool)
    for ob in obstacles:
        if isinstance(ob, Sphere):
            centre, radius = np.asarray(ob.center[:2]), ob.radius
        else:
            centre, radius = np.asarray(ob.center_xy), ob.radius
        hit |= _segment_point_distance_xy(p0[:, :2], slots[:, :2], centre) < radius
    return hit


def _clear_of(points: FloatArray, centre_xy, radius: float, gap: float) -> bool:
    d = np.linalg.norm(points[:, :2] - np.asarray(centre_xy), axis=1) - radius
    return bool(np.all(d >= gap))


def _wall(x0, y0, x1, y1, radius, pitch) -> list[Cylinder]:
    length = math.hypot(x1 - x0, y1 - y0)
    k = max(1, math.ceil(length / pitch))
    return [
        Cylinder((x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k), radius) for i in range(k + 1)
    ]


# --------------------------------------------------------------------------- #
# Scenario builders
# --------------------------------------------------------------------------- #
def _span(start: FloatArray, slots: FloatArray) -> tuple[float, float]:
    """x-interval between the start formation's front and the goal formation's back."""
    return float(start[:, 0].max()), float(slots[:, 0].min())


def _s1(rng, prm, start, slots, variant):
    density = prm.densities[variant]
    half = 0.5 * float(np.ptp(np.r_[start[:, 1], slots[:, 1]])) + prm.margin_y
    front, back = _span(start, slots)
    x_lo, x_hi = front + prm.keep_clear + 1.0, back - prm.keep_clear - 1.0
    area = (x_hi - x_lo) * 2 * half
    count = rng.poisson(density * area / 100.0)
    nominal = np.r_[start, slots]
    trees: list[Cylinder] = []
    for _ in range(50 * max(count, 1)):
        if len(trees) >= count:
            break
        r = rng.uniform(*prm.tree_radius)
        c = (rng.uniform(x_lo, x_hi), rng.uniform(-half, half))
        if not _clear_of(nominal, c, r, prm.keep_clear):
            continue
        if any(math.dist(c, t.center_xy) - r - t.radius < prm.tree_gap for t in trees):
            continue
        trees.append(Cylinder(c, r))
    if not blocked_paths(start, slots, trees).any():  # guarantee the property S1 probes
        i = int(rng.integers(len(start)))
        for _ in range(200):
            s = rng.uniform(0.35, 0.65)
            c = tuple(start[i, :2] + s * (slots[i, :2] - start[i, :2]))
            r = prm.tree_radius[1]
            if _clear_of(nominal, c, r, prm.keep_clear) and all(
                math.dist(c, t.center_xy) - r - t.radius >= prm.tree_gap for t in trees
            ):
                trees.append(Cylinder(c, r))
                break
    return trees


def _s2(rng, prm, start, slots):
    n = len(slots)
    chosen = rng.choice(n, size=math.ceil(n / 2), replace=False)
    obstacles: list[Sphere | Cylinder] = []
    for i in sorted(chosen):
        for _ in range(500):
            r = rng.uniform(*prm.s2_radius)
            gap = rng.uniform(*prm.s2_clearance)
            ang = rng.uniform(0, 2 * math.pi)
            centre = slots[i, :2] + (r + gap) * np.array([math.cos(ang), math.sin(ang)])
            ob = (
                Sphere((centre[0], centre[1], slots[i, 2]), r)
                if rng.random() < 0.5
                else Cylinder(tuple(centre), r)
            )
            if ObstacleSet([ob]).clearance(slots).min() < prm.s2_clearance[0] - 1e-9:
                continue  # must not crowd any other slot either
            if ObstacleSet([ob]).clearance(start).min() < prm.keep_clear:
                continue
            probe = np.array([[centre[0], centre[1], slots[i, 2]]])
            if obstacles and ObstacleSet(obstacles).clearance(probe).min() < r + 0.5:
                continue  # keep obstacles apart
            obstacles.append(ob)
            break
    return obstacles


def _s3(prm, variant, start, slots):
    z0, mid = prm.altitude, 0.5 * sum(_span(start, slots))
    if variant == "line":
        return [Sphere((mid, 0.0, z0), prm.s3_sphere_radius)]
    width = float(np.ptp(np.r_[start[:, 1], slots[:, 1]])) + prm.u_extra_width
    y_arm = width / 2 + prm.u_cyl_radius
    x_open, x_bottom = mid - prm.u_depth / 2, mid + prm.u_depth / 2
    cyl = _wall(x_open, y_arm, x_bottom, y_arm, prm.u_cyl_radius, prm.u_pitch)
    cyl += _wall(x_open, -y_arm, x_bottom, -y_arm, prm.u_cyl_radius, prm.u_pitch)
    cyl += _wall(x_bottom, -y_arm, x_bottom, y_arm, prm.u_cyl_radius, prm.u_pitch)
    return cyl


def _s4(prm, start, slots):
    width = float(np.ptp(slots[:, 1]))
    w = max(prm.corridor_min_width, prm.corridor_ratio * width)
    if w >= width:
        raise ValueError(
            f"S4 needs a formation wider than the corridor ({width:.2f} m <= {w:.2f} m); "
            "use more agents or a wider formation"
        )
    mid, r, pitch = 0.5 * sum(_span(start, slots)), prm.wall_radius, prm.wall_pitch
    y_in = w / 2 + r
    y_out = 0.5 * float(np.ptp(np.r_[start[:, 1], slots[:, 1]])) + prm.wall_extent
    x0, x1 = mid - prm.corridor_length / 2, mid + prm.corridor_length / 2
    walls = _wall(x0, y_in, x1, y_in, r, pitch) + _wall(x0, -y_in, x1, -y_in, r, pitch)
    walls += _wall(x0, y_in + pitch, x0, y_out, r, pitch)
    walls += _wall(x0, -y_in - pitch, x0, -y_out, r, pitch)
    return walls


# --------------------------------------------------------------------------- #
# Public entry point
# --------------------------------------------------------------------------- #
def _rng(scenario_id: str, n: int, formation: str, variant, seed: int) -> np.random.Generator:
    key = f"{scenario_id}|{n}|{formation}|{variant}"
    return np.random.default_rng([seed, zlib.crc32(key.encode())])


def make_scenario(
    scenario_id: str,
    n: int,
    seed: int,
    *,
    formation: str = "wedge",
    variant: str | None = None,
    params: GeneratorParams | None = None,
    controller: APFConfig | None = None,
) -> Scenario:
    """Generate scenario ``scenario_id`` ∈ S1..S5 for ``n`` UAVs from ``seed``.

    Raises
    ------
    ValueError
        Unknown id / formation / variant, n < 1, or an infeasible combination (S4 with a
        formation narrower than the minimum corridor).
    """
    if scenario_id not in SCENARIOS:
        raise ValueError(f"scenario_id must be one of {SCENARIOS}, got {scenario_id!r}")
    if formation not in FORMATIONS:
        raise ValueError(f"formation must be one of {FORMATIONS}, got {formation!r}")
    if variant is None:
        variant = VARIANTS[scenario_id][0]
    if variant not in VARIANTS[scenario_id]:
        raise ValueError(f"{scenario_id} variant must be one of {VARIANTS[scenario_id]}")
    if n < 1:
        raise ValueError("n must be >= 1")
    prm = GeneratorParams() if params is None else params
    cfg = APFConfig() if controller is None else controller
    rng = _rng(scenario_id, n, formation, variant, seed)
    z0, length = prm.altitude, prm.length

    if scenario_id == "S5" and variant == "swap":
        radius = prm.swap_min_radius
        if n > 1:
            radius = max(radius, prm.spacing / (2 * math.sin(math.pi / n)) * 1.0001)
        ang = 2 * math.pi * np.arange(n) / n
        ring = np.c_[radius * np.cos(ang), radius * np.sin(ang), np.zeros(n)]
        centre = np.array([length / 2, 0.0, z0])
        slots, nominal = centre + ring, centre - ring
        edges: Edges = tuple((i, i + 1) for i in range(n - 1)) + (((0, n - 1),) if n >= 3 else ())
    elif scenario_id == "S5":
        n_a = math.ceil(n / 2)
        off_a, e_a = formation_offsets("line", n_a, prm.spacing)
        if n > n_a:
            off_b, e_b = formation_offsets("line", n - n_a, prm.spacing)
        else:
            off_b, e_b = np.zeros((0, 3)), ()
        goal_a, goal_b = np.array([length, 0.0, z0]), np.array([0.0, 0.0, z0])
        slots = np.r_[goal_a + off_a, goal_b + off_b]
        nominal = np.r_[goal_b + off_a, goal_a + off_b]
        edges = tuple(e_a) + tuple((i + n_a, j + n_a) for i, j in e_b)
    else:
        off, edges = formation_offsets(formation, n, prm.spacing)
        depth = float(np.ptp(off[:, 0]))  # the goal formation starts `length` m ahead
        slots = np.array([length + depth, 0.0, z0]) + off
        nominal = np.array([0.0, 0.0, z0]) + off

    if scenario_id == "S1":
        obstacles = _s1(rng, prm, nominal, slots, variant)
    elif scenario_id == "S2":
        obstacles = _s2(rng, prm, nominal, slots)
    elif scenario_id == "S3":
        obstacles = _s3(prm, variant, nominal, slots)
    elif scenario_id == "S4":
        obstacles = _s4(prm, nominal, slots)
    else:
        obstacles = []

    geo = ObstacleSet(obstacles)
    for _ in range(1000):
        p0 = nominal + rng.normal(scale=prm.start_noise, size=nominal.shape)
        try:
            _check_free_start(p0, geo, cfg.d_min + 0.1)
        except ValueError:
            continue
        break
    else:  # pragma: no cover - only with absurd parameters
        raise ValueError("could not draw a collision-free start state")

    for arr in (slots, p0):
        arr.flags.writeable = False
    v0 = np.zeros_like(p0)
    v0.flags.writeable = False
    reference = None
    t_final = prm.t_final
    if prm.ref_speed is not None and scenario_id != "S5":  # S5 groups: two centres
        reference = StraightPath(
            start=tuple(nominal.mean(axis=0)),
            goal=tuple(slots.mean(axis=0)),
            v_ref=prm.ref_speed,
            a_ref=prm.ref_accel,
        )
        t_final = reference.duration + prm.ref_settle
    shape = formation if scenario_id != "S5" else ("ring" if variant == "swap" else "line")
    name = f"{scenario_id}-{variant or 'default'}-{shape}-N{n}-s{seed}"
    scenario = Scenario(
        name=name,
        slots=slots,
        p0=p0,
        v0=v0,
        edges=tuple(edges),
        obstacles=tuple(obstacles),
        sim=SimConfig(dt=prm.dt, t_final=t_final, a_max=prm.a_max),
        controller=cfg,
        seed=seed,
        reference=reference,
    )
    scenario.build_controller()  # validates slots (outside obstacles, pairs > d_s)
    return scenario
