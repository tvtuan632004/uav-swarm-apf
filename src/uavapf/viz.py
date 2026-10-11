"""Trajectory figures and GIF animations (spec: docs/specs/T05-scenarios-yaml.md).

Uses the object-oriented matplotlib API only: no ``pyplot`` state, no ``plt.show()``,
no backend changes, so it is safe in scripts, tests and parallel workers.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Patch, Rectangle

from uavapf.geometry import Sphere
from uavapf.sim import Trajectory

if TYPE_CHECKING:
    from uavapf.scenarios.io import Scenario

VIEWS = ("xy", "xz", "yz", "3d")
_AXES = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}
_LABELS = ("x [m]", "y [m]", "z [m]")
_OBSTACLE_STYLE = {"facecolor": "0.75", "edgecolor": "0.35", "alpha": 0.6, "zorder": 0}
_FAR = 1e4  # m, stands in for an infinite cylinder end; clipped by the axes
_GIF_DPI = 60  # GIF frames are rendered small: file size and time scale with dpi^2


def _check_view(view: str) -> None:
    if view not in VIEWS:
        raise ValueError(f"view must be one of {VIEWS}, got {view!r}")


def _obstacle_patch(ob, view: str) -> Patch:
    a, b = _AXES[view]
    if isinstance(ob, Sphere):
        return Circle((ob.center[a], ob.center[b]), ob.radius, **_OBSTACLE_STYLE)
    if view == "xy":
        return Circle(ob.center_xy, ob.radius, **_OBSTACLE_STYLE)
    lo = ob.z_min if math.isfinite(ob.z_min) else -_FAR
    hi = ob.z_max if math.isfinite(ob.z_max) else _FAR
    return Rectangle((ob.center_xy[a] - ob.radius, lo), 2 * ob.radius, hi - lo, **_OBSTACLE_STYLE)


def _data_limits(traj: Trajectory, scenario: Scenario | None, view: str) -> tuple:
    """Axis limits from trajectory, slots and finite obstacle extents (2-D views)."""
    a, b = _AXES[view]
    pts = [traj.p.reshape(-1, 3)]
    if scenario is not None:
        pts.append(scenario.slots)
    xy = np.concatenate(pts)[:, [a, b]]
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    if scenario is not None:
        for ob in scenario.obstacles:
            if isinstance(ob, Sphere):
                c = np.array([ob.center[a], ob.center[b]])
                lo, hi = np.minimum(lo, c - ob.radius), np.maximum(hi, c + ob.radius)
            elif view == "xy":
                c = np.asarray(ob.center_xy)
                lo, hi = np.minimum(lo, c - ob.radius), np.maximum(hi, c + ob.radius)
            else:
                lo[0] = min(lo[0], ob.center_xy[a] - ob.radius)
                hi[0] = max(hi[0], ob.center_xy[a] + ob.radius)
    pad = 0.05 * max(float(np.max(hi - lo)), 1.0)
    return (lo[0] - pad, hi[0] + pad), (lo[1] - pad, hi[1] + pad)


def plot_trajectory(
    traj: Trajectory,
    scenario: Scenario | None = None,
    *,
    view: str = "xy",
    ax: Axes | None = None,
) -> Figure:
    """Draw every agent's path, its start (dot) and, with a scenario, slots and obstacles.

    Parameters
    ----------
    traj : Trajectory
        Logged run.
    scenario : Scenario, optional
        Adds slots ("x") and one obstacle patch per obstacle in 2-D views.
    view : {"xy", "xz", "yz", "3d"}
        Projection.
    ax : matplotlib Axes, optional
        Draw into this axes (must be 3-D for ``view="3d"``); otherwise a new figure.

    Returns
    -------
    matplotlib.figure.Figure
    """
    _check_view(view)
    if ax is None:
        fig = Figure(figsize=(7, 6), layout="constrained")
        ax = fig.add_subplot(projection="3d" if view == "3d" else None)
    fig = ax.figure

    n = traj.p.shape[1]
    colors = [f"C{i % 10}" for i in range(n)]
    if view == "3d":
        for i in range(n):
            ax.plot(*traj.p[:, i].T, color=colors[i], label=f"UAV {i}")
            ax.scatter(*traj.p[0, i], color=colors[i], s=15)
            if scenario is not None:
                ax.scatter(*scenario.slots[i], color=colors[i], marker="x", s=40)
        ax.set_xlabel(_LABELS[0])
        ax.set_ylabel(_LABELS[1])
        ax.set_zlabel(_LABELS[2])
        return fig

    a, b = _AXES[view]
    if scenario is not None:
        for ob in scenario.obstacles:
            ax.add_patch(_obstacle_patch(ob, view))
    for i in range(n):
        ax.plot(traj.p[:, i, a], traj.p[:, i, b], color=colors[i], lw=1.4, label=f"UAV {i}")
        ax.plot(traj.p[0, i, a], traj.p[0, i, b], "o", color=colors[i], ms=4, label="_start")
        if scenario is not None:
            ax.plot(*scenario.slots[i, [a, b]], "x", color=colors[i], ms=7, mew=2, label="_slot")
    xlim, ylim = _data_limits(traj, scenario, view)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(_LABELS[a])
    ax.set_ylabel(_LABELS[b])
    ax.grid(True, lw=0.3, alpha=0.5)
    if n <= 10:
        ax.legend(loc="best", fontsize="small")
    return fig


def animate_trajectory(
    traj: Trajectory,
    path: Path | str,
    scenario: Scenario | None = None,
    *,
    view: str = "xy",
    fps: int = 20,
    stride: int = 1,
) -> Path:
    """Write a GIF with one frame per ``stride`` logged samples.

    Each frame shows the agents' current positions, their tails, and the time stamp
    ``t = {t:.2f} s`` (so consecutive frames always differ and Pillow keeps them all).

    Returns
    -------
    pathlib.Path
        The written file.
    """
    _check_view(view)
    path = Path(path)
    if path.suffix.lower() != ".gif":
        raise ValueError(f"only .gif output is supported, got {path.suffix!r}")
    if stride < 1 or fps < 1:
        raise ValueError("stride and fps must be >= 1")
    path.parent.mkdir(parents=True, exist_ok=True)

    fig = plot_trajectory(traj, scenario, view=view)
    ax = fig.axes[0]
    for line in ax.get_lines():  # fade the full paths; the animation draws the tails
        line.set_alpha(0.25)
    n = traj.p.shape[1]
    frames = range(0, len(traj), stride)
    title = ax.set_title("")
    if view == "3d":
        heads = ax.scatter(*traj.p[0].T, c=[f"C{i % 10}" for i in range(n)], s=25)
    else:
        a, b = _AXES[view]
        heads = ax.scatter(
            traj.p[0, :, a], traj.p[0, :, b], c=[f"C{i % 10}" for i in range(n)], s=25, zorder=3
        )

    def update(k: int):
        title.set_text(f"t = {traj.t[k]:.2f} s")
        if view == "3d":
            heads._offsets3d = tuple(traj.p[k].T)  # noqa: SLF001 (matplotlib 3-D scatter API)
        else:
            heads.set_offsets(traj.p[k][:, [a, b]])
        return heads, title

    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    anim.save(path, writer=PillowWriter(fps=fps), dpi=_GIF_DPI)
    return path
