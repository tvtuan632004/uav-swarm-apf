"""Run the paper's stationary-slot APF controller through a simple sphere scene.

Run: uv run python scripts/base_demo.py
The safety analysis in paper/main.tex assumes unsaturated continuous flow; this
numeric RK4 example uses no acceleration or speed saturation.
"""

from __future__ import annotations

import numpy as np

from uavapf.control import APFController, StallReset
from uavapf.geometry import Sphere
from uavapf.paths import ensure_dir, results_root
from uavapf.sim import SimConfig, simulate


def main() -> None:
    """Simulate three UAVs and save their trajectory outside the source tree."""
    slots = np.array([[5.0, -2.0, 2.0], [5.0, 0.0, 2.0], [5.0, 2.0, 2.0]])
    p0 = slots.copy()
    p0[:, 0] = -5.0
    obstacles = [Sphere((0.0, 0.0, 2.0), 0.8)]
    controller = APFController(slots, [(0, 1), (1, 2)], obstacles)
    reset = StallReset(controller)
    trajectory = simulate(
        controller,
        p0,
        np.zeros_like(p0),
        SimConfig(dt=0.01, t_final=12.0, log_every=10),
        reset=reset,
    )
    rho, _ = controller.geometry.distance(trajectory.p.reshape(-1, 3))
    distances = np.linalg.norm(trajectory.p[:, :, None, :] - trajectory.p[:, None, :, :], axis=-1)
    upper = np.triu_indices(len(slots), 1)
    print(f"final max slot error : {np.linalg.norm(trajectory.p[-1] - slots, axis=1).max():.3f} m")
    print(f"minimum clearance    : {rho.min():.3f} m")
    print(f"minimum separation   : {distances[:, upper[0], upper[1]].min():.3f} m")
    print(f"hybrid resets        : {reset.count.sum()}")
    output = trajectory.save(
        ensure_dir(results_root() / "base_demo") / "apf_swarm.npz",
        controller="APF equations 4-16, main.tex",
    )
    print(f"trajectory           : {output}")


if __name__ == "__main__":
    main()
