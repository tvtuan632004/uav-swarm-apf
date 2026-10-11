"""Run one scenario end to end: simulate, save, plot (spec: docs/specs/T05-scenarios-yaml.md).

Command line::

    uv run python scripts/run_scenario.py configs/scenarios/demo_sphere.yaml [--out DIR] [--no-anim]
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from uavapf.control import StallReset
from uavapf.metrics import collision_mask, collision_stop, min_clearance, min_separation
from uavapf.paths import results_root
from uavapf.scenarios.io import Scenario, load_scenario
from uavapf.sim import Trajectory, simulate
from uavapf.viz import animate_trajectory, plot_trajectory

_MAX_GIF_FRAMES = 300


@dataclass(frozen=True, slots=True)
class RunResult:
    """Outcome of :func:`run_scenario`; distances in m."""

    trajectory: Trajectory
    collided: bool
    min_clearance: float
    min_separation: float
    final_slot_error: float
    files: dict[str, Path] = field(default_factory=dict)


def run_scenario(scenario: Scenario, out_dir: Path | str, *, animate: bool = True) -> RunResult:
    """Simulate ``scenario`` with the APF controller and stall reset, then save outputs.

    Writes ``trajectory.npz`` (with meta ``name`` and ``seed``), ``xy.png`` and, if
    ``animate``, ``anim.gif`` (at most ~300 frames) into ``out_dir``. The run stops at
    the first collision (D005).
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    controller = scenario.build_controller()
    obstacles = scenario.obstacle_set()
    d_min = controller.config.d_min

    traj = simulate(
        controller,
        scenario.p0,
        scenario.v0,
        scenario.sim,
        stop=collision_stop(obstacles, d_min),
        reset=StallReset(controller),
    )

    files: dict[str, Path] = {}
    files["trajectory"] = traj.save(
        out_dir / "trajectory.npz",
        name=scenario.name,
        seed=-1 if scenario.seed is None else scenario.seed,
    )
    fig = plot_trajectory(traj, scenario, view="xy")
    fig.suptitle(scenario.name)
    files["figure"] = out_dir / "xy.png"
    fig.savefig(files["figure"], dpi=120)
    if animate:
        stride = max(1, -(-len(traj) // _MAX_GIF_FRAMES))
        files["animation"] = animate_trajectory(traj, out_dir / "anim.gif", scenario, stride=stride)

    return RunResult(
        trajectory=traj,
        collided=bool(collision_mask(traj.p, obstacles, d_min).any()),
        min_clearance=min_clearance(traj.p, obstacles),
        min_separation=min_separation(traj.p),
        final_slot_error=float(np.linalg.norm(traj.p[-1] - scenario.slots, axis=1).max()),
        files=files,
    )


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point; returns 0 on a collision-free run, 1 otherwise."""
    parser = argparse.ArgumentParser(description="Run one uavapf scenario YAML file.")
    parser.add_argument("scenario", type=Path, help="path to a scenario .yaml file")
    parser.add_argument("--out", type=Path, default=None, help="output folder")
    parser.add_argument("--no-anim", action="store_true", help="skip the GIF animation")
    args = parser.parse_args(argv)

    scenario = load_scenario(args.scenario)
    out = args.out if args.out is not None else results_root() / "scenarios" / scenario.name
    result = run_scenario(scenario, out, animate=not args.no_anim)

    t_end = result.trajectory.t[-1]
    print(f"scenario          : {scenario.name}  (N = {scenario.n_agents}, t_end = {t_end:.2f} s)")
    print(f"collision         : {'YES' if result.collided else 'no'}")
    print(f"min clearance     : {result.min_clearance:.3f} m")
    print(f"min separation    : {result.min_separation:.3f} m")
    print(f"final slot error  : {result.final_slot_error:.3f} m   -> {out}")
    return 1 if result.collided else 0
