"""T04 acceptance tests: obstacle distance and normal. Spec: docs/specs/T04-geometry.md."""

from __future__ import annotations

import math

import numpy as np
import pytest

from uavapf.geometry import Cylinder, ObstacleSet, Sphere, cylinder_distance, sphere_distance

INF = math.inf
H = 1e-6  # finite-difference step, m


@pytest.fixture
def rng():
    return np.random.default_rng(2026)


def fd_gradient(rho_fn, pts: np.ndarray) -> np.ndarray:
    """Central differences of a scalar field rho_fn: (N,3) -> (N,), result (N,3)."""
    grad = np.empty_like(pts)
    for k in range(3):
        e = np.zeros(3)
        e[k] = H
        grad[:, k] = (rho_fn(pts + e) - rho_fn(pts - e)) / (2 * H)
    return grad


def sphere_rho(c, r):
    return lambda q: sphere_distance(q, np.array([c]), np.array([r]))[0][:, 0]


def cyl_rho(cxy, r, z0, z1):
    return lambda q: cylinder_distance(
        q, np.array([cxy]), np.array([r]), np.array([z0]), np.array([z1])
    )[0][:, 0]


# --------------------------------------------------------------------------- #
# Hand-checked values (criterion 3)
# --------------------------------------------------------------------------- #
def test_sphere_values():
    pts = np.array([[3.0, 0, 0], [1.0, 2.4, 3.2], [1.5, 0, 0]])
    rho, n = sphere_distance(pts, np.array([[1.0, 0, 0]]), np.array([1.0]))
    np.testing.assert_allclose(rho[:, 0], [1.0, 3.0, -0.5])
    np.testing.assert_allclose(n[0, 0], [1, 0, 0])
    np.testing.assert_allclose(n[1, 0], [0, 0.6, 0.8])
    np.testing.assert_allclose(n[2, 0], [1, 0, 0])


def test_infinite_cylinder_values():
    pts = np.array([[3.0, 4.0, 100.0], [0.5, 0.0, -7.0]])
    rho, n = cylinder_distance(pts, np.array([[0.0, 0.0]]), np.array([2.0]), [-INF], [INF])
    np.testing.assert_allclose(rho[:, 0], [3.0, -1.5])
    np.testing.assert_allclose(n[0, 0], [0.6, 0.8, 0.0])
    np.testing.assert_allclose(n[1, 0], [1.0, 0.0, 0.0])


def test_capped_cylinder_regions():
    c, r, z0, z1 = (0.0, 0.0), 1.0, 0.0, 10.0
    pts = np.array(
        [
            [3.0, 0.0, 5.0],  # side
            [0.5, 0.0, 12.0],  # above the top cap
            [0.0, 0.5, -1.0],  # below the bottom cap
            [4.0, 0.0, 14.0],  # beyond the top rim: corner (3, 4)
            [0.0, 0.8, 5.0],  # inside, nearest to the side
            [0.0, 0.0, 9.9],  # inside, nearest to the top cap (on axis)
        ]
    )
    rho, n = cylinder_distance(pts, np.array([c]), np.array([r]), [z0], [z1])
    np.testing.assert_allclose(rho[:, 0], [2.0, 2.0, 1.0, 5.0, -0.2, -0.1], atol=1e-12)
    np.testing.assert_allclose(n[0, 0], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(n[1, 0], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(n[2, 0], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(n[3, 0], [0.6, 0, 0.8], atol=1e-12)
    np.testing.assert_allclose(n[4, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(n[5, 0], [0, 0, 1], atol=1e-12)


def test_half_infinite_cylinder_building():
    # a "building": ground (z = -inf) up to z = 30 m
    pts = np.array([[0.0, 0.0, 35.0], [5.0, 0.0, -1000.0]])
    rho, n = cylinder_distance(pts, np.array([[0.0, 0.0]]), np.array([2.0]), [-INF], [30.0])
    np.testing.assert_allclose(rho[:, 0], [5.0, 3.0])
    np.testing.assert_allclose(n[0, 0], [0, 0, 1])
    np.testing.assert_allclose(n[1, 0], [1, 0, 0])


# --------------------------------------------------------------------------- #
# Degenerate points (criterion 1)
# --------------------------------------------------------------------------- #
def test_degenerate_normals_are_fixed_unit_vectors():
    _, ns = sphere_distance(np.array([[1.0, 2.0, 3.0]]), np.array([[1.0, 2.0, 3.0]]), [1.0])
    np.testing.assert_array_equal(ns[0, 0], [0.0, 0.0, 1.0])
    _, nc = cylinder_distance(
        np.array([[1.0, 2.0, 50.0]]), np.array([[1.0, 2.0]]), [1.0], [-INF], [INF]
    )
    np.testing.assert_array_equal(nc[0, 0], [1.0, 0.0, 0.0])


def test_normals_unit_and_finite_everywhere(rng):
    pts = rng.uniform(-6, 6, size=(500, 3))
    pts[:5] = [[0, 0, 0], [0, 0, 3], [1, 0, 0], [0, 0, -2], [0, 1, 2]]  # centre, axis, surface
    obs = ObstacleSet(
        [Sphere((0, 0, 0), 1.0), Cylinder((0, 0), 1.0, -2.0, 2.0), Cylinder((0, 0), 1.0)]
    )
    rho, n = obs.distance(pts)
    assert np.all(np.isfinite(rho)) and np.all(np.isfinite(n))
    np.testing.assert_allclose(np.linalg.norm(n, axis=-1), 1.0, rtol=1e-12)


# --------------------------------------------------------------------------- #
# Gradient consistency (criterion 2, task-table "done" criterion)
# --------------------------------------------------------------------------- #
def test_sphere_normal_matches_fd_gradient(rng):
    c, r = (0.5, -1.0, 2.0), 1.5
    pts = np.array(c) + rng.uniform(-5, 5, size=(400, 3))
    pts = pts[np.linalg.norm(pts - c, axis=1) > 0.1]
    _, n = sphere_distance(pts, np.array([c]), np.array([r]))
    np.testing.assert_allclose(n[:, 0], fd_gradient(sphere_rho(c, r), pts), rtol=1e-6, atol=1e-6)


@pytest.mark.parametrize(("z0", "z1"), [(-INF, INF), (0.0, 8.0), (-INF, 5.0)])
def test_cylinder_normal_matches_fd_gradient(rng, z0, z1):
    cxy, r = (1.0, -2.0), 2.0
    pts = rng.uniform(-8, 12, size=(2000, 3)) + [cxy[0], cxy[1], 0.0]
    dr = np.hypot(pts[:, 0] - cxy[0], pts[:, 1] - cxy[1]) - r
    dz = np.maximum(z0 - pts[:, 2], pts[:, 2] - z1)
    rr = np.hypot(pts[:, 0] - cxy[0], pts[:, 1] - cxy[1])
    smooth = (
        (rr > 0.1)  # away from the axis
        & (np.abs(dr - dz) > 0.05)  # away from the interior medial set
        & ~((np.abs(dr) < 0.05) | (np.abs(dz) < 0.05))  # away from the surface itself
    )
    # inside, the two caps meet at the mid-plane: also non-smooth
    if math.isfinite(z0) and math.isfinite(z1):
        smooth &= np.abs(pts[:, 2] - 0.5 * (z0 + z1)) > 0.05
    pts = pts[smooth]
    assert len(pts) > 500
    _, n = cylinder_distance(pts, np.array([cxy]), np.array([r]), [z0], [z1])
    np.testing.assert_allclose(
        n[:, 0], fd_gradient(cyl_rho(cxy, r, z0, z1), pts), rtol=1e-6, atol=1e-6
    )


# --------------------------------------------------------------------------- #
# Vectorisation (criterion 4)
# --------------------------------------------------------------------------- #
def test_obstacle_set_matches_loop_and_keeps_order(rng):
    obstacles = [
        Cylinder((2.0, 0.0), 0.5, 0.0, 6.0),
        Sphere((0.0, 0.0, 3.0), 1.0),
        Cylinder((-3.0, 1.0), 0.8),
        Sphere((5.0, 5.0, 5.0), 2.0),
    ]
    pts = rng.uniform(-8, 8, size=(64, 3))
    rho, n = ObstacleSet(obstacles).distance(pts)
    assert rho.shape == (64, 4) and n.shape == (64, 4, 3)
    assert rho.dtype == np.float64 and n.dtype == np.float64
    for j, ob in enumerate(obstacles):
        rho_j, n_j = ObstacleSet([ob]).distance(pts)
        np.testing.assert_array_equal(rho[:, j], rho_j[:, 0])
        np.testing.assert_array_equal(n[:, j], n_j[:, 0])
        for i in range(0, 64, 9):  # point-by-point loop
            r1, n1 = ObstacleSet([ob]).distance(pts[i : i + 1])
            np.testing.assert_allclose(r1[0, 0], rho[i, j], rtol=1e-14)
            np.testing.assert_allclose(n1[0, 0], n[i, j], rtol=1e-14)


def test_empty_set():
    rho, n = ObstacleSet([]).distance(np.zeros((5, 3)))
    assert rho.shape == (5, 0) and n.shape == (5, 0, 3)
    assert len(ObstacleSet([])) == 0


def test_points_not_modified(rng):
    pts = rng.normal(size=(10, 3))
    before = pts.copy()
    ObstacleSet([Sphere((0, 0, 0), 1.0), Cylinder((0, 0), 1.0, 0, 1)]).distance(pts)
    np.testing.assert_array_equal(pts, before)


# --------------------------------------------------------------------------- #
# Validation
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    "make",
    [
        lambda: Sphere((0, 0, 0), 0.0),
        lambda: Sphere((0, 0, INF), 1.0),
        lambda: Sphere((0, 0), 1.0),
        lambda: Cylinder((0, 0), -1.0),
        lambda: Cylinder((0, 0), 1.0, 5.0, 5.0),
        lambda: Cylinder((0, math.nan), 1.0),
    ],
)
def test_bad_obstacle_raises(make):
    with pytest.raises(ValueError):
        make()


def test_bad_points_raise():
    with pytest.raises(ValueError):
        ObstacleSet([Sphere((0, 0, 0), 1.0)]).distance(np.zeros((4, 2)))
