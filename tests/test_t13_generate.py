"""T13 acceptance tests: seeded S1-S5 generators. Spec: docs/specs/T13-scenarios.md."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

from uavapf.geometry import ObstacleSet
from uavapf.scenarios import (
    FORMATIONS,
    SCENARIOS,
    VARIANTS,
    GeneratorParams,
    blocked_paths,
    formation_offsets,
    make_scenario,
    scenario_from_dict,
    scenario_to_dict,
)

PRM = GeneratorParams()
RHO0 = 2.0


def all_cases():
    for sid in SCENARIOS:
        for var in VARIANTS[sid]:
            forms = ("line",) if sid == "S5" else FORMATIONS
            for form in forms:
                yield sid, var, form


@pytest.mark.parametrize(("sid", "var", "form"), list(all_cases()))
def test_reproducible_and_feasible(sid, var, form):
    a = make_scenario(sid, 10, 7, formation=form, variant=var)
    b = make_scenario(sid, 10, 7, formation=form, variant=var)
    assert scenario_to_dict(a) == scenario_to_dict(b)
    # round trip through the strict YAML validator: start in F, shapes, edges ...
    again = scenario_from_dict(scenario_to_dict(a))
    np.testing.assert_array_equal(again.p0, a.p0)
    a.build_controller()  # slots outside obstacles, slot pairs farther than d_s
    assert a.seed == 7 and a.n_agents == 10 and a.name.startswith(sid)
    assert a.sim.a_max == PRM.a_max and a.sim.dt == PRM.dt


def test_seed_changes_scenario():
    a = make_scenario("S1", 10, 1)
    b = make_scenario("S1", 10, 2)
    assert scenario_to_dict(a) != scenario_to_dict(b)


@pytest.mark.parametrize("n", [1, 2, 5, 10, 20, 50])
@pytest.mark.parametrize("kind", FORMATIONS)
def test_formation_offsets(kind, n):
    off, edges = formation_offsets(kind, n, 2.0)
    assert off.shape == (n, 3)
    if n > 1:
        d = np.linalg.norm(off[:, None] - off[None], axis=-1)[np.triu_indices(n, 1)]
        assert d.min() >= 2.0 - 1e-9  # Assumption 1: spacing > d_s
        assert len(edges) >= n - 1  # connected graph has at least N-1 edges
    for i, j in edges:
        assert 0 <= i < j < n
    assert len(set(edges)) == len(edges)


@pytest.mark.parametrize("seed", range(5))
@pytest.mark.parametrize("variant", VARIANTS["S1"])
def test_s1_forest_properties(seed, variant):
    s = make_scenario("S1", 10, seed, variant=variant)
    assert len(s.obstacles) >= 1
    assert blocked_paths(s.p0, s.slots, s.obstacles).any()
    geo = ObstacleSet(s.obstacles)
    assert geo.clearance(s.slots).min() >= PRM.keep_clear - 1e-9
    for a, b in itertools.combinations(s.obstacles, 2):
        gap = math.dist(a.center_xy, b.center_xy) - a.radius - b.radius
        assert gap >= PRM.tree_gap - 1e-9 or b is s.obstacles[-1]  # last may be the forced tree


def test_s1_density_increases_obstacle_count():
    counts = {
        v: np.mean([len(make_scenario("S1", 10, k, variant=v).obstacles) for k in range(10)])
        for v in ("sparse", "dense")
    }
    assert counts["dense"] > counts["sparse"]


@pytest.mark.parametrize("seed", range(5))
@pytest.mark.parametrize("n", [5, 10, 20])
def test_s2_slots_inside_influence(seed, n):
    s = make_scenario("S2", n, seed)
    rho = ObstacleSet(s.obstacles).clearance(s.slots).min(axis=1)
    assert np.all(rho >= PRM.s2_clearance[0] - 1e-9)  # every slot outside every obstacle
    assert np.sum(rho <= RHO0) >= math.ceil(n / 2)  # D1: at least half sit inside rho0


@pytest.mark.parametrize("form", FORMATIONS)
def test_s3_obstacle_on_straight_paths(form):
    line = make_scenario("S3", 10, 0, formation=form, variant="line")
    assert blocked_paths(line.p0, line.slots, line.obstacles).any()
    u = make_scenario("S3", 10, 0, formation=form, variant="u")
    assert blocked_paths(u.p0, u.slots, u.obstacles).all()  # concave trap spans the formation


@pytest.mark.parametrize("form", FORMATIONS)
def test_s4_corridor_narrower_than_formation(form):
    s = make_scenario("S4", 10, 0, formation=form)
    width = float(np.ptp(s.slots[:, 1]))
    xs = np.array([ob.center_xy[0] for ob in s.obstacles])
    ys = np.array([ob.center_xy[1] for ob in s.obstacles])
    exit_wall = np.isclose(xs, xs.max())  # the two corridor walls end at the largest x
    corridor = 2 * (np.abs(ys[exit_wall]).min() - PRM.wall_radius)
    assert 0 < corridor < width
    outer = np.abs(s.slots[:, 1]) > corridor / 2
    assert outer.any()
    assert blocked_paths(s.p0, s.slots, s.obstacles)[outer].all()


def test_s4_rejects_too_narrow_formation():
    with pytest.raises(ValueError, match="corridor"):
        make_scenario("S4", 2, 0, formation="line")


def test_s5_groups_head_on():
    s = make_scenario("S5", 10, 0, variant="groups")
    a, b = slice(0, 5), slice(5, 10)
    assert np.all(s.slots[a, 0] > s.p0[a, 0] + 10)  # group A goes +x
    assert np.all(s.slots[b, 0] < s.p0[b, 0] - 10)  # group B goes -x
    assert all(i < 5 and j < 5 or i >= 5 and j >= 5 for i, j in s.edges)


@pytest.mark.parametrize("n", [3, 10, 50])
def test_s5_swap_through_centre(n):
    s = make_scenario("S5", n, 0, variant="swap")
    centre = s.slots.mean(axis=0)
    mid = 0.5 * (s.p0 + s.slots)
    assert np.all(np.linalg.norm(mid - centre, axis=1) < 3 * PRM.start_noise + 1e-9)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"scenario_id": "S9", "n": 5, "seed": 0},
        {"scenario_id": "S1", "n": 0, "seed": 0},
        {"scenario_id": "S1", "n": 5, "seed": 0, "formation": "circle"},
        {"scenario_id": "S1", "n": 5, "seed": 0, "variant": "jungle"},
        {"scenario_id": "S2", "n": 5, "seed": 0, "variant": "x"},
    ],
)
def test_invalid_arguments(kwargs):
    with pytest.raises(ValueError):
        make_scenario(**kwargs)
