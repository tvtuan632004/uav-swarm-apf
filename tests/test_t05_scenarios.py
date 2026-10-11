"""T05 acceptance tests: YAML scenarios, run, plots. Spec: docs/specs/T05-scenarios-yaml.md."""

from __future__ import annotations

import copy
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless; the library itself must never switch backends

import numpy as np  # noqa: E402
import pytest  # noqa: E402
import yaml  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
from PIL import Image  # noqa: E402

# Skipped until Codex creates the modules (T05); "done" requires 0 skipped tests.
_run = pytest.importorskip("uavapf.scenarios.run", reason="T05 not implemented yet")
_viz = pytest.importorskip("uavapf.viz", reason="T05 not implemented yet")
RunResult, main, run_scenario = _run.RunResult, _run.main, _run.run_scenario
animate_trajectory, plot_trajectory = _viz.animate_trajectory, _viz.plot_trajectory

from uavapf.control import APFConfig, APFController  # noqa: E402
from uavapf.geometry import Cylinder, Sphere  # noqa: E402
from uavapf.scenarios import (  # noqa: E402
    Scenario,
    load_scenario,
    save_scenario,
    scenario_from_dict,
    scenario_to_dict,
)
from uavapf.sim import SimConfig, Trajectory  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
FIXTURE = REPO / "configs" / "scenarios" / "demo_sphere.yaml"


def base_dict() -> dict:
    with FIXTURE.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def with_(data: dict, path: str, value) -> dict:
    """Copy of data with data[a][b]... = value (path 'a.b'); value None deletes."""
    out = copy.deepcopy(data)
    *parents, leaf = path.split(".")
    node = out
    for key in parents:
        node = node[key]
    if value is None:
        del node[leaf]
    else:
        node[leaf] = value
    return out


def same_scenario(a: Scenario, b: Scenario) -> None:
    assert a.name == b.name and a.seed == b.seed
    for f in ("slots", "p0", "v0"):
        np.testing.assert_array_equal(getattr(a, f), getattr(b, f))
    assert a.edges == b.edges
    assert a.obstacles == b.obstacles
    assert a.sim == b.sim
    assert a.controller == b.controller


# --------------------------------------------------------------------------- #
# Loading (criteria 1-2)
# --------------------------------------------------------------------------- #
def test_fixture_loads():
    s = load_scenario(FIXTURE)
    assert s.name == "demo_sphere" and s.seed == 0 and s.n_agents == 3
    np.testing.assert_array_equal(s.slots[:, 0], [5.0, 5.0, 5.0])
    np.testing.assert_array_equal(s.p0[:, 0], [-5.0, -5.0, -5.0])
    np.testing.assert_array_equal(s.v0, np.zeros((3, 3)))
    assert s.edges == ((0, 1), (1, 2))
    assert s.obstacles == (Sphere((0.0, 0.0, 2.0), 0.8),)
    assert s.sim == SimConfig(dt=0.01, t_final=12.0, log_every=10)
    assert s.controller == APFConfig()
    for arr in (s.slots, s.p0, s.v0):
        assert arr.dtype == np.float64 and arr.shape == (3, 3)
    assert len(s.obstacle_set()) == 1
    assert isinstance(s.build_controller(), APFController)


def test_load_accepts_str_path():
    same_scenario(load_scenario(str(FIXTURE)), load_scenario(FIXTURE))


@pytest.mark.parametrize(
    ("edges", "expected"),
    [
        ("none", ()),
        ("chain", ((0, 1), (1, 2), (2, 3))),
        ("ring", ((0, 1), (1, 2), (2, 3), (0, 3))),
        ("complete", ((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3))),
        ([[1, 0], [3, 2]], ((0, 1), (2, 3))),
    ],
)
def test_edge_shorthands(edges, expected):
    d = base_dict()
    d["formation"]["slots"] = [[5.0, 2.0 * k, 2.0] for k in range(4)]
    d["start"]["positions"] = [[-5.0, 2.0 * k, 2.0] for k in range(4)]
    d["formation"]["edges"] = edges
    assert scenario_from_dict(d).edges == expected


def test_ring_needs_three_agents():
    d = base_dict()
    d["formation"]["slots"] = d["formation"]["slots"][:2]
    d["start"]["positions"] = d["start"]["positions"][:2]
    d["formation"]["edges"] = "ring"
    with pytest.raises(ValueError):
        scenario_from_dict(d)


def test_cylinder_bounds_default_infinite_and_inf_literals():
    d = base_dict()
    d["obstacles"] = [
        {"type": "cylinder", "center_xy": [0.0, 10.0], "radius": 0.5},
        {"type": "cylinder", "center_xy": [0.0, -10.0], "radius": 1.0, "z_max": 30.0},
        {"type": "cylinder", "center_xy": [9.0, 9.0], "radius": 1.0, "z_min": None},
    ]
    yaml_text = yaml.safe_dump(d).replace("z_min: null", "z_min: -.inf")
    s = scenario_from_dict(yaml.safe_load(yaml_text))
    assert s.obstacles[0] == Cylinder((0.0, 10.0), 0.5)
    assert s.obstacles[1] == Cylinder((0.0, -10.0), 1.0, -math.inf, 30.0)
    assert s.obstacles[2].z_min == -math.inf


def test_defaults_and_overrides():
    d = base_dict()
    for key in ("start.velocities", "obstacles", "controller", "seed"):
        d = with_(d, key, None)
    d["sim"] = {"dt": 0.02, "t_final": 5.0}
    s = scenario_from_dict(d)
    np.testing.assert_array_equal(s.v0, np.zeros((3, 3)))
    assert s.obstacles == () and s.controller == APFConfig() and s.seed is None
    assert s.sim == SimConfig(dt=0.02, t_final=5.0)
    d["controller"] = {"kg": 0.0, "rho0": 3.0}
    assert scenario_from_dict(d).controller == APFConfig(kg=0.0, rho0=3.0)
    d["start"]["velocities"] = [[0.1, 0.0, 0.0]] * 3
    np.testing.assert_array_equal(scenario_from_dict(d).v0[:, 0], [0.1] * 3)


# --------------------------------------------------------------------------- #
# Round trip (criterion 1)
# --------------------------------------------------------------------------- #
def test_round_trip_dict_and_file(tmp_path):
    d = base_dict()
    d["obstacles"].append({"type": "cylinder", "center_xy": [3.0, 3.0], "radius": 0.5})
    d["obstacles"].append(
        {"type": "cylinder", "center_xy": [9.0, 0.0], "radius": 1.0, "z_max": 30.0}
    )
    d["controller"] = {"kd": 2.0}
    s = scenario_from_dict(d)
    same_scenario(scenario_from_dict(scenario_to_dict(s)), s)
    out = save_scenario(s, tmp_path / "sub" / "copy.yaml")
    assert out.is_file()
    with out.open(encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)  # plain YAML types only
    assert raw["version"] == 1
    assert isinstance(raw["formation"]["edges"], list)
    assert raw["obstacles"][1]["z_min"] is None
    same_scenario(load_scenario(out), s)


# --------------------------------------------------------------------------- #
# Validation (criterion 3)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    ("path", "value", "needle"),
    [
        ("version", 2, "version"),
        ("version", None, "version"),
        ("name", None, "name"),
        ("sim.dt", None, "dt"),
        ("sim.dtt", 0.1, "dtt"),  # typo
        ("formation", None, "formation"),
        ("formation.edges", "star", "star"),
        ("formation.edges", [[0, 5]], "edge"),
        ("start.positions", [[0.0, 0.0, 0.0]], "positions"),  # wrong N
        ("start.velocities", [[0.0, 0.0]] * 3, "velocities"),
        ("controller", {"kgg": 1.0}, "kgg"),
        ("obstacles", [{"type": "cube", "center": [0, 0, 0], "radius": 1}], "cube"),
        ("obstacles", [{"type": "sphere", "center": [0, 0, 0], "raduis": 1}], "raduis"),
        ("surprise", 1, "surprise"),
    ],
)
def test_invalid_files_raise_with_message(path, value, needle):
    d = with_(base_dict(), path, value)
    with pytest.raises(ValueError, match=needle):
        scenario_from_dict(d)


def test_start_state_must_be_in_free_set():
    inside = with_(base_dict(), "start.positions", [[0.0, 0.0, 2.0], [-5, 0, 2], [-5, 2, 2]])
    with pytest.raises(ValueError):
        scenario_from_dict(inside)
    too_close = with_(base_dict(), "start.positions", [[-5, 0, 2], [-5, 0.3, 2], [-5, 2, 2]])
    with pytest.raises(ValueError):
        scenario_from_dict(too_close)


def test_non_finite_numbers_rejected():
    d = base_dict()
    d["formation"]["slots"][0][0] = float("nan")
    with pytest.raises(ValueError):
        scenario_from_dict(d)


# --------------------------------------------------------------------------- #
# Running (criterion 4)
# --------------------------------------------------------------------------- #
def test_run_scenario_writes_outputs(tmp_path):
    result = run_scenario(load_scenario(FIXTURE), tmp_path / "out")
    assert isinstance(result, RunResult)
    assert not result.collided
    assert result.min_clearance > 0 and result.min_separation > APFConfig().d_min
    assert result.final_slot_error < 0.1
    assert set(result.files) == {"trajectory", "figure", "animation"}
    for f in result.files.values():
        assert f.is_file() and f.parent == tmp_path / "out"
    back = Trajectory.load(result.files["trajectory"])
    np.testing.assert_array_equal(back.p, result.trajectory.p)
    with np.load(result.files["trajectory"]) as z:
        assert str(z["meta_name"]) == "demo_sphere"


def test_run_without_animation(tmp_path):
    result = run_scenario(load_scenario(FIXTURE), tmp_path, animate=False)
    assert set(result.files) == {"trajectory", "figure"}


def test_cli_main(tmp_path, capsys):
    code = main([str(FIXTURE), "--out", str(tmp_path / "cli"), "--no-anim"])
    assert code == 0
    assert (tmp_path / "cli" / "trajectory.npz").is_file()
    assert (tmp_path / "cli" / "xy.png").is_file()
    assert not (tmp_path / "cli" / "anim.gif").exists()
    assert capsys.readouterr().out.strip()


def test_cli_returns_1_on_collision(tmp_path):
    d = base_dict()
    d["formation"]["slots"] = [[5.0, 0.0, 2.0], [-5.0, 0.0, 2.0]]
    d["start"]["positions"] = [[-5.0, 0.001, 2.0], [5.0, 0.0, 2.0]]
    d["formation"]["edges"] = "none"
    d["obstacles"] = []
    d["sim"] = {"dt": 0.01, "t_final": 20.0, "a_max": 5.0}  # S2 reproduction (D005/D006)
    f = tmp_path / "swap.yaml"
    f.write_text(yaml.safe_dump(d), encoding="utf-8")
    assert main([str(f), "--out", str(tmp_path / "swap"), "--no-anim"]) == 1


# --------------------------------------------------------------------------- #
# Plots (criteria 5-6)
# --------------------------------------------------------------------------- #
def two_d_patches(ax):
    return [p for p in ax.patches if isinstance(p, Circle | Rectangle)]


@pytest.fixture(scope="module")
def run():
    s = load_scenario(FIXTURE)
    s = scenario_from_dict(
        {
            **scenario_to_dict(s),
            "obstacles": [
                {"type": "sphere", "center": [0.0, 0.0, 2.0], "radius": 0.8},
                {"type": "cylinder", "center_xy": [0.0, 6.0], "radius": 0.5},
            ],
        }
    )
    from uavapf.sim import simulate

    traj = simulate(s.build_controller(), s.p0, s.v0, SimConfig(dt=0.02, t_final=4.0, log_every=5))
    return s, traj


@pytest.mark.parametrize("view", ["xy", "xz", "yz", "3d"])
def test_plot_views(run, view):
    s, traj = run
    fig = plot_trajectory(traj, s, view=view)
    assert isinstance(fig, Figure)
    ax = fig.axes[0]
    labels = [ln.get_label() for ln in ax.get_lines() if ln.get_label().startswith("UAV ")]
    assert labels == [f"UAV {i}" for i in range(s.n_agents)]
    if view != "3d":
        assert len(two_d_patches(ax)) == len(s.obstacles)


def test_plot_into_existing_axes_and_without_scenario(run):
    import matplotlib.pyplot as plt

    s, traj = run
    fig, ax = plt.subplots()
    out = plot_trajectory(traj, ax=ax)
    assert out is fig
    assert len(two_d_patches(ax)) == 0
    plt.close(fig)


def test_plot_rejects_unknown_view(run):
    with pytest.raises(ValueError):
        plot_trajectory(run[1], view="top")


def test_backend_untouched(run):
    before = matplotlib.get_backend()
    plot_trajectory(run[1], run[0])
    assert matplotlib.get_backend() == before


@pytest.mark.parametrize("stride", [1, 4])
def test_animation_frame_count(run, tmp_path, stride):
    s, traj = run
    out = animate_trajectory(traj, tmp_path / f"a{stride}.gif", s, stride=stride, fps=10)
    assert out.is_file()
    with Image.open(out) as im:
        assert im.n_frames == math.ceil(len(traj) / stride)


def test_animation_rejects_other_suffix(run, tmp_path):
    with pytest.raises(ValueError):
        animate_trajectory(run[1], tmp_path / "a.avi", run[0])
