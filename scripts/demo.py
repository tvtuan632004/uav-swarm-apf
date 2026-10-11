"""Smoke demo for T03 + T04: a PD swarm flies to its goals past a building.

This is NOT the paper's controller (that starts at T06). It only shows how the simulator,
the obstacle geometry and the results folder fit together.

Run:  uv run python scripts/demo.py
"""

from __future__ import annotations

import math

import numpy as np

from uavapf.geometry import Cylinder, ObstacleSet, Sphere
from uavapf.paths import ensure_dir, results_root
from uavapf.sim import SimConfig, simulate

SEED = 0
N_AGENTS = 5
KP, KD = 0.6, 1.4  # 1/s^2, 1/s  (plain PD, no avoidance)


def main() -> None:
    rng = np.random.default_rng(SEED)
    p0 = np.column_stack([np.zeros(N_AGENTS), np.linspace(-4, 4, N_AGENTS), np.full(N_AGENTS, 10)])
    p0 += rng.normal(scale=0.2, size=p0.shape)
    v0 = np.zeros_like(p0)
    goals = p0 + np.array([40.0, 0.0, 0.0])

    obstacles = ObstacleSet(
        [Cylinder((20.0, 6.0), 2.0, -math.inf, 30.0), Sphere((20.0, -6.0, 10.0), 1.5)]
    )

    def controller(t: float, p: np.ndarray, v: np.ndarray) -> np.ndarray:
        return KP * (goals - p) - KD * v

    def arrived(t: float, p: np.ndarray, v: np.ndarray) -> bool:
        return bool(np.all(np.linalg.norm(p - goals, axis=1) < 0.1))

    cfg = SimConfig(dt=0.02, t_final=60.0, a_max=3.0, v_max=5.0, log_every=5)
    traj = simulate(controller, p0, v0, cfg, stop=arrived)

    rho, _ = obstacles.distance(traj.p.reshape(-1, 3))
    print(f"steps logged      : {len(traj)}  (t_end = {traj.t[-1]:.2f} s)")
    print(f"max |u| (m/s^2)   : {np.linalg.norm(traj.u, axis=-1).max():.3f}  (a_max = 3)")
    print(f"max |v| (m/s)     : {np.linalg.norm(traj.v, axis=-1).max():.3f}  (v_max = 5)")
    print(f"min obstacle gap  : {rho.min():.3f} m")

    out = traj.save(ensure_dir(results_root() / "demo") / "pd_swarm.npz", seed=SEED)
    print(f"saved             : {out}")


if __name__ == "__main__":
    main()
