"""ObstacleSet.clearance (added for T11b metrics) must equal distance()[0]."""

from __future__ import annotations

import math

import numpy as np

from uavapf.geometry import Cylinder, ObstacleSet, Sphere


def test_clearance_equals_distance_rho():
    rng = np.random.default_rng(44)
    obs = ObstacleSet(
        [
            Cylinder((2.0, 0.0), 0.5, 0.0, 6.0),
            Sphere((0.0, 0.0, 3.0), 1.0),
            Cylinder((-3.0, 1.0), 0.8),
            Cylinder((1.0, 4.0), 1.2, -math.inf, 2.0),
        ]
    )
    pts = rng.uniform(-6, 8, size=(3000, 3))
    pts[:4] = [[2.0, 0.0, 3.0], [0.0, 0.0, 3.0], [-3.0, 1.0, 0.0], [1.0, 4.0, 1.9]]
    np.testing.assert_allclose(obs.clearance(pts), obs.distance(pts)[0], rtol=0, atol=1e-12)


def test_clearance_empty_set():
    assert ObstacleSet([]).clearance(np.zeros((4, 3))).shape == (4, 0)
