# T05 — Scenario files (YAML), trajectory plots and animation

- Branch: `T05-scenarios-yaml`
- Spec + tests: Claude · Implementation: Codex · Review: Claude
- Tests: `tests/test_t05_scenarios.py` (do not edit). The file skips itself until the new module
  exists; the task is done only when `uv run pytest -q` reports **0 skipped**.
- Fixture (part of the spec, do not edit): `configs/scenarios/demo_sphere.yaml`
- Depends on: T11b (`collision_stop`, safety metrics)
- Done when: `uv run python scripts/run_scenario.py configs/scenarios/demo_sphere.yaml`
  writes a trajectory, a figure and an animation.

## 1. Scope
In: the YAML format of one scenario, loader/saver with strict validation, building
the controller, one-command run, static plots and a GIF animation.
Out: random generation of S1–S5 (T13), metrics beyond safety (T12), Monte Carlo (T16).

## 2. File format (version 1)

```yaml
version: 1                      # required, must be 1
name: demo_sphere               # required, used as output folder name
seed: 0                         # optional int (kept for T13), default null
sim:                            # -> uavapf.sim.SimConfig
  dt: 0.01                      # required, s
  t_final: 12.0                 # required, s
  a_max: null                   # optional, m/s^2, null = no limit
  v_max: null                   # optional, m/s
  log_every: 10                 # optional, default 1
formation:
  slots: [[5, -2, 2], [5, 0, 2], [5, 2, 2]]     # required, (N, 3) m
  edges: chain                  # list of [i, j] or one of: none | chain | ring | complete
start:
  positions: [[-5, -2, 2], [-5, 0, 2], [-5, 2, 2]]   # required, (N, 3) m
  velocities: zeros             # optional: zeros (default) or (N, 3) list, m/s
obstacles:                      # optional, default []
  - {type: sphere, center: [0, 0, 2], radius: 0.8}
  - {type: cylinder, center_xy: [3, 3], radius: 0.5}               # infinite
  - {type: cylinder, center_xy: [6, 0], radius: 1.0, z_max: 30}    # building
controller: {}                  # optional overrides of APFConfig (Table 4 defaults)
```

- Edge shorthands: `none` → no edges; `chain` → (i, i+1); `ring` → chain + (N−1, 0)
  (needs N ≥ 3); `complete` → every pair i < j. Lists are normalised to sorted
  pairs (i < j), in the order given.
- Cylinder `z_min` / `z_max`: absent or `null` → −∞ / +∞; YAML `.inf` / `-.inf` accepted.
- Every mapping is **strict**: an unknown key raises `ValueError` whose message
  contains the key (catches typos such as `raduis`). Missing required keys too.

## 3. Public API

`src/uavapf/scenarios/io.py`, re-exported from `uavapf.scenarios`:

```python
@dataclass(frozen=True, slots=True)
class Scenario:
    name: str
    slots: FloatArray  # (N, 3) m, read-only
    p0: FloatArray  # (N, 3) m
    v0: FloatArray  # (N, 3) m/s
    edges: tuple[tuple[int, int], ...]
    obstacles: tuple[Sphere | Cylinder, ...]
    sim: SimConfig
    controller: APFConfig
    seed: int | None = None

    @property
    def n_agents(self) -> int: ...
    def obstacle_set(self) -> ObstacleSet: ...
    def build_controller(self) -> APFController: ...


def scenario_from_dict(data: Mapping[str, Any]) -> Scenario: ...
def scenario_to_dict(scenario: Scenario) -> dict[str, Any]: ...  # plain YAML-safe types
def load_scenario(path: Path | str) -> Scenario: ...
def save_scenario(scenario: Scenario, path: Path | str) -> Path: ...
```

Validation in `scenario_from_dict` (all `ValueError`): version, strict keys, shapes
(slots, positions, velocities all (N, 3), N ≥ 1), finite numbers, edge indices,
start state in the free set (every rho > 0 and every pair r > `controller.d_min`,
Assumption 4 of main.tex). Slot feasibility is checked by `build_controller`
(the `APFController` constructor).

`scenario_to_dict` writes edges as an explicit list, infinite cylinder bounds as
`null`, and omits nothing: `scenario_from_dict(scenario_to_dict(s))` equals `s`.

`src/uavapf/scenarios/run.py`:

```python
@dataclass(frozen=True, slots=True)
class RunResult:
    trajectory: Trajectory
    collided: bool
    min_clearance: float  # m
    min_separation: float  # m
    final_slot_error: float  # m, max over agents at the last logged state
    files: dict[str, Path]  # keys: "trajectory", "figure", and "animation" if made


def run_scenario(scenario: Scenario, out_dir: Path, *, animate: bool = True) -> RunResult: ...
def main(argv: Sequence[str] | None = None) -> int: ...
```

- `run_scenario` builds the controller and a fresh `StallReset`, calls `simulate`
  with `stop=collision_stop(...)`, saves `trajectory.npz` (with meta `name`, `seed`),
  `xy.png` and, if `animate`, `anim.gif` into `out_dir` (created if missing).
- `main(["file.yaml", "--out", DIR, "--no-anim"])`: `--out` defaults to
  `results_root() / "scenarios" / <name>`. Prints a 5-line summary, returns 0, or
  1 if the run collided.
- `scripts/run_scenario.py` is a thin wrapper: `raise SystemExit(main())`.

`src/uavapf/viz.py`:

```python
def plot_trajectory(traj, scenario=None, *, view="xy", ax=None) -> Figure: ...
def animate_trajectory(traj, path, scenario=None, *, view="xy", fps=20, stride=1) -> Path: ...
```

- `view` ∈ {"xy", "xz", "yz", "3d"}; anything else → `ValueError`.
- One line per agent with label `"UAV {i}"`; start as a dot, slot as an "x".
- With a scenario, obstacles are drawn as patches in 2-D views: one patch per
  obstacle (sphere → circle; cylinder → circle in "xy", rectangle in "xz"/"yz",
  clipped to the axes when infinite). "3d" draws no obstacle patches.
- Never calls `plt.show()` and never changes the matplotlib backend.
- `animate_trajectory` writes a GIF (Pillow writer) with
  `ceil(len(traj) / stride)` frames; other suffixes → `ValueError`. Every frame shows
  the time stamp `t = {t:.2f} s`, so consecutive frames always differ (Pillow merges
  identical consecutive GIF frames, which would break the frame count).

## 4. Acceptance criteria
1. The fixture loads to the expected `Scenario`; round trip dict ↔ YAML ↔ Scenario.
2. Shorthands, defaults, infinite bounds, controller overrides as in §2.
3. Every validation rule in §3 raises `ValueError` with a useful message.
4. `run_scenario` on the fixture: no collision, files exist, `final_slot_error` < 0.1 m.
5. Plot: N labelled lines and M patches in "xy"; all four views work.
6. GIF frame count as specified.
