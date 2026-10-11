"""Benchmark one APF controller call (T11c target: N=50, M=20 <= 1 ms on the desktop).

Run: uv run python scripts/bench_controller.py
"""

from __future__ import annotations

import time

import numpy as np

from uavapf.control import APFController
from uavapf.geometry import Cylinder, Sphere


def scene(n_agents: int, n_obs: int, seed: int = 0):
    rng = np.random.default_rng(seed)
    side = int(np.ceil(np.sqrt(n_agents)))
    slots = np.array([[2.0 * (i % side), 2.0 * (i // side), 5.0] for i in range(n_agents)])
    obstacles = []
    while len(obstacles) < n_obs:
        c = rng.uniform(-2.0, 2.0 * side, size=2)
        if np.min(np.linalg.norm(slots[:, :2] - c, axis=1)) < 1.6:
            continue
        obstacles.append(
            Sphere((c[0], c[1], 5.0), 0.5) if len(obstacles) % 2 else Cylinder((c[0], c[1]), 0.5)
        )
    ctrl = APFController(slots, [(i, i + 1) for i in range(n_agents - 1)], obstacles)
    p = slots + rng.normal(scale=0.15, size=slots.shape)
    return ctrl, p, rng.normal(size=slots.shape)


def main() -> None:
    print(f"{'N':>4} {'M':>4} {'ms/call':>9} {'60 s run (est.)':>16}")
    for n, m in [(5, 5), (10, 10), (20, 10), (50, 20)]:
        ctrl, p, v = scene(n, m)
        for _ in range(20):
            ctrl(0.0, p, v)
        reps = 300
        t0 = time.perf_counter()
        for _ in range(reps):
            ctrl(0.0, p, v)
        ms = (time.perf_counter() - t0) / reps * 1e3
        print(f"{n:>4} {m:>4} {ms:>9.3f} {ms * 4 * 6000 / 1e3:>14.1f} s")  # RK4, dt = 0.01 s


if __name__ == "__main__":
    main()
