"""Scenario files: strict YAML schema, loader and saver (spec: docs/specs/T05-scenarios-yaml.md).

A scenario fixes everything one run needs: formation slots and graph, initial state,
obstacles, integration settings and controller gains. Validation is strict so that a
typo in a hand-written file fails loudly instead of silently using a default.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

from uavapf.control import APFConfig, APFController
from uavapf.geometry import Cylinder, ObstacleSet, Sphere
from uavapf.sim import SimConfig

FloatArray = NDArray[np.float64]
Obstacle = Sphere | Cylinder

FORMAT_VERSION = 1
_TOP_KEYS = {"version", "name", "seed", "sim", "formation", "start", "obstacles", "controller"}
_TOP_REQUIRED = {"version", "name", "sim", "formation", "start"}
_EDGE_SHORTHANDS = ("none", "chain", "ring", "complete")


# --------------------------------------------------------------------------- #
# Scenario
# --------------------------------------------------------------------------- #
@dataclass(frozen=True, slots=True)
class Scenario:
    """One fully specified run.

    Attributes
    ----------
    name : str
        Identifier, also the default output folder name.
    slots : ndarray, shape (N, 3)
        Formation slots p_i* in m (read-only).
    p0, v0 : ndarray, shape (N, 3)
        Initial positions in m and velocities in m/s (read-only).
    edges : tuple of (int, int)
        Formation graph E with i < j.
    obstacles : tuple of Sphere or Cylinder
        Static obstacles.
    sim : SimConfig
        Integration settings.
    controller : APFConfig
        Controller gains (Table 4 defaults unless overridden).
    seed : int or None
        Seed the scenario was generated from (T13), if any.
    """

    name: str
    slots: FloatArray
    p0: FloatArray
    v0: FloatArray
    edges: tuple[tuple[int, int], ...]
    obstacles: tuple[Obstacle, ...]
    sim: SimConfig
    controller: APFConfig
    seed: int | None = None

    @property
    def n_agents(self) -> int:
        """Number of UAVs N."""
        return int(self.slots.shape[0])

    def obstacle_set(self) -> ObstacleSet:
        """Vectorised obstacle collection for geometry and metrics."""
        return ObstacleSet(self.obstacles)

    def build_controller(self) -> APFController:
        """APF controller for this scenario (validates slot feasibility)."""
        return APFController(self.slots, self.edges, self.obstacles, self.controller)


# --------------------------------------------------------------------------- #
# Validation helpers
# --------------------------------------------------------------------------- #
def _mapping(value: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"'{where}' must be a mapping, got {type(value).__name__}")
    return value


def _check_keys(
    data: Mapping[str, Any], allowed: Iterable[str], required: Iterable[str], where: str
) -> None:
    allowed = set(allowed)
    for key in data:
        if key not in allowed:
            raise ValueError(f"unknown key '{key}' in '{where}' (allowed: {sorted(allowed)})")
    for key in required:
        if key not in data:
            raise ValueError(f"missing required key '{key}' in '{where}'")


def _points(value: Any, name: str, n: int | None = None) -> FloatArray:
    try:
        arr = np.array(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"'{name}' must be a list of [x, y, z] numbers") from exc
    if arr.ndim != 2 or arr.shape[1] != 3 or arr.shape[0] < 1:
        raise ValueError(f"'{name}' must have shape (N, 3), got {arr.shape}")
    if n is not None and arr.shape[0] != n:
        raise ValueError(f"'{name}' has {arr.shape[0]} rows, expected N = {n}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"'{name}' contains non-finite numbers")
    arr.flags.writeable = False
    return arr


def _edges(value: Any, n: int) -> tuple[tuple[int, int], ...]:
    if isinstance(value, str):
        if value == "none":
            return ()
        if value == "chain":
            return tuple((i, i + 1) for i in range(n - 1))
        if value == "ring":
            if n < 3:
                raise ValueError("edges 'ring' needs N >= 3")
            return tuple((i, i + 1) for i in range(n - 1)) + ((0, n - 1),)
        if value == "complete":
            return tuple((i, j) for i in range(n) for j in range(i + 1, n))
        raise ValueError(f"unknown edges shorthand '{value}' (use {_EDGE_SHORTHANDS} or a list)")
    if not isinstance(value, list | tuple):
        raise ValueError("'formation.edges' must be a shorthand string or a list of [i, j]")
    out: list[tuple[int, int]] = []
    for pair in value:
        if (
            not isinstance(pair, list | tuple)
            or len(pair) != 2
            or not all(isinstance(k, int) and not isinstance(k, bool) for k in pair)
        ):
            raise ValueError(f"edge {pair!r} must be a pair of integer indices")
        i, j = pair
        if not (0 <= i < n and 0 <= j < n) or i == j:
            raise ValueError(f"edge {pair!r} must join two different agents in [0, {n - 1}]")
        edge = (min(i, j), max(i, j))
        if edge in out:
            raise ValueError(f"duplicate edge {pair!r}")
        out.append(edge)
    return tuple(out)


def _bound(value: Any, default: float, name: str) -> float:
    if value is None:
        return default
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"'{name}' must be a number or null")
    return float(value)


def _obstacle(item: Any, k: int) -> Obstacle:
    where = f"obstacles[{k}]"
    data = _mapping(item, where)
    kind = data.get("type")
    if kind == "sphere":
        _check_keys(data, {"type", "center", "radius"}, {"center", "radius"}, where)
        return Sphere(tuple(data["center"]), data["radius"])
    if kind == "cylinder":
        _check_keys(
            data, {"type", "center_xy", "radius", "z_min", "z_max"}, {"center_xy", "radius"}, where
        )
        return Cylinder(
            tuple(data["center_xy"]),
            data["radius"],
            _bound(data.get("z_min"), -math.inf, f"{where}.z_min"),
            _bound(data.get("z_max"), math.inf, f"{where}.z_max"),
        )
    raise ValueError(f"unknown obstacle type '{kind}' in '{where}' (use 'sphere' or 'cylinder')")


def _check_free_start(p0: FloatArray, obstacles: ObstacleSet, d_min: float) -> None:
    """Assumption 4 of main.tex: p(0) in the free set."""
    if len(obstacles):
        rho = obstacles.clearance(p0)
        if np.any(rho <= 0):
            i, k = np.argwhere(rho <= 0)[0]
            raise ValueError(f"start position of agent {i} is inside obstacle {k}")
    if len(p0) > 1:
        i, j = np.triu_indices(len(p0), 1)
        r = np.linalg.norm(p0[i] - p0[j], axis=1)
        if np.any(r <= d_min):
            a = int(np.argmin(r))
            raise ValueError(
                f"start positions of agents {i[a]} and {j[a]} are {r[a]:.3f} m apart "
                f"(must exceed d_min = {d_min} m)"
            )


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #
def scenario_from_dict(data: Mapping[str, Any]) -> Scenario:
    """Build and validate a :class:`Scenario` from the parsed YAML mapping.

    Raises
    ------
    ValueError
        On a wrong version, unknown or missing keys, wrong shapes, non-finite numbers,
        bad edges or obstacles, or a start state outside the free set.
    """
    data = _mapping(data, "scenario")
    _check_keys(data, _TOP_KEYS, _TOP_REQUIRED, "scenario")
    if data["version"] != FORMAT_VERSION:
        raise ValueError(f"unsupported version {data['version']!r}; this code reads version 1")

    sim = _mapping(data["sim"], "sim")
    _check_keys(sim, (f.name for f in fields(SimConfig)), ("dt", "t_final"), "sim")
    formation = _mapping(data["formation"], "formation")
    _check_keys(formation, ("slots", "edges"), ("slots",), "formation")
    start = _mapping(data["start"], "start")
    _check_keys(start, ("positions", "velocities"), ("positions",), "start")
    gains = _mapping(data.get("controller") or {}, "controller")
    _check_keys(gains, (f.name for f in fields(APFConfig)), (), "controller")

    slots = _points(formation["slots"], "formation.slots")
    n = slots.shape[0]
    p0 = _points(start["positions"], "start.positions", n)
    velocities = start.get("velocities", "zeros")
    if velocities == "zeros":
        velocities = np.zeros((n, 3))
    v0 = _points(velocities, "start.velocities", n)
    edges = _edges(formation.get("edges", "none"), n)
    obstacles = tuple(_obstacle(item, k) for k, item in enumerate(data.get("obstacles") or []))
    controller = APFConfig(**gains)
    seed = data.get("seed")
    if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int)):
        raise ValueError("'seed' must be an integer or null")

    scenario = Scenario(
        name=str(data["name"]),
        slots=slots,
        p0=p0,
        v0=v0,
        edges=edges,
        obstacles=obstacles,
        sim=SimConfig(**sim),
        controller=controller,
        seed=seed,
    )
    _check_free_start(p0, scenario.obstacle_set(), controller.d_min)
    return scenario


def _obstacle_to_dict(ob: Obstacle) -> dict[str, Any]:
    if isinstance(ob, Sphere):
        return {"type": "sphere", "center": list(ob.center), "radius": ob.radius}
    return {
        "type": "cylinder",
        "center_xy": list(ob.center_xy),
        "radius": ob.radius,
        "z_min": None if math.isinf(ob.z_min) else ob.z_min,
        "z_max": None if math.isinf(ob.z_max) else ob.z_max,
    }


def scenario_to_dict(scenario: Scenario) -> dict[str, Any]:
    """Plain, YAML-safe mapping; ``scenario_from_dict`` of it gives back an equal scenario."""
    return {
        "version": FORMAT_VERSION,
        "name": scenario.name,
        "seed": scenario.seed,
        "sim": asdict(scenario.sim),
        "formation": {
            "slots": scenario.slots.tolist(),
            "edges": [list(edge) for edge in scenario.edges],
        },
        "start": {"positions": scenario.p0.tolist(), "velocities": scenario.v0.tolist()},
        "obstacles": [_obstacle_to_dict(ob) for ob in scenario.obstacles],
        "controller": asdict(scenario.controller),
    }


def load_scenario(path: Path | str) -> Scenario:
    """Read and validate a scenario YAML file."""
    path = Path(path)
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    try:
        return scenario_from_dict(data)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def save_scenario(scenario: Scenario, path: Path | str) -> Path:
    """Write ``scenario`` as YAML (parent folders are created) and return the path."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(scenario_to_dict(scenario), sort_keys=False, allow_unicode=True)
    path.write_text(text, encoding="utf-8")
    return path
