"""Pilot comparison APF (T1-T4 + reset) vs B1 (Khatib) on generated S1-S5 scenarios.

NOT the paper experiment (that is T16-T18 with 500 runs per cell and tuned gains). This
is a quick sanity check with untuned Table-4 gains on a few seeds, in both saturation
regimes of D006. Writes one CSV row per run to $UAVAPF_RESULTS/pilot/pilot.csv.

Run: uv run python scripts/pilot.py [--seeds 3] [--n 5 10] [--workers 2]
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from uavapf.baselines import KhatibController
from uavapf.control import StallReset
from uavapf.metrics import collision_mask, evaluate_run
from uavapf.paths import ensure_dir, results_root
from uavapf.scenarios import GeneratorParams, make_scenario
from uavapf.sim import simulate

CASES = [
    ("S1", "medium"),
    ("S2", None),
    ("S3", "line"),
    ("S3", "u"),
    ("S4", None),
    ("S5", "groups"),
    ("S5", "swap"),
]
EPS_P = 0.3  # m, Table 4


def one_run(job: tuple) -> dict:
    sid, variant, method, n, seed, a_max = job
    params = dataclasses.replace(GeneratorParams(), a_max=a_max)
    scn = make_scenario(sid, n, seed, variant=variant, formation="wedge", params=params)
    geo = scn.obstacle_set()
    d_min = scn.controller.d_min
    if method == "apf":
        ctrl = scn.build_controller()
        reset = StallReset(ctrl)
    else:
        ctrl = KhatibController(scn.slots, scn.obstacles)
        reset = None

    def stop(t, p, v):
        if bool(collision_mask(p, geo, d_min)):
            return True
        done = np.linalg.norm(p - scn.slots, axis=1).max() <= EPS_P
        return done and np.linalg.norm(v, axis=1).max() < 0.05

    t0 = time.perf_counter()
    traj = simulate(ctrl, scn.p0, scn.v0, scn.sim, stop=stop, reset=reset)
    wall = time.perf_counter() - t0
    m = evaluate_run(traj, scn.slots, geo, d_min=d_min, eps_p=EPS_P, a_max=a_max)
    row = {
        "scenario": sid if variant is None else f"{sid}-{variant}",
        "method": method,
        "n": n,
        "seed": seed,
        "a_max": "inf" if a_max is None else a_max,
        **m.as_dict(),
        "max_speed": float(np.linalg.norm(traj.v, axis=-1).max()),
        "wall_s": wall,
    }
    return row


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--n", type=int, nargs="+", default=[5, 10])
    ap.add_argument("--workers", type=int, default=2)
    args = ap.parse_args()

    jobs = [
        (sid, var, method, n, seed, a_max)
        for sid, var in CASES
        for method in ("apf", "b1")
        for n in args.n
        for seed in range(args.seeds)
        for a_max in (5.0, None)
    ]
    out = ensure_dir(results_root() / "pilot") / "pilot.csv"
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(one_run, jobs, chunksize=2))
    with Path(out).open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"{len(rows)} runs -> {out}\n")
    print(
        f"{'scenario':<11} {'a_max':>5} | {'APF succ':>8} {'coll':>5} {'e_F':>6} {'v_max':>6} |"
        f" {'B1 succ':>7} {'coll':>5} {'e_F':>6} {'v_max':>6}"
    )
    for sid, var in CASES:
        name = sid if var is None else f"{sid}-{var}"
        for a_max in ("5.0", "inf"):
            cells = []
            for method in ("apf", "b1"):
                sel = [
                    r
                    for r in rows
                    if r["scenario"] == name and r["method"] == method and str(r["a_max"]) == a_max
                ]
                clean = [r for r in sel if not r["collided"]]  # metrics after a crash are void
                succ = np.mean([r["success"] for r in sel])
                coll = np.mean([r["collided"] for r in sel])
                ef = np.mean([r["formation_error"] for r in clean]) if clean else float("nan")
                vmax = np.median([r["max_speed"] for r in clean]) if clean else float("nan")
                cells.append(f"{succ:>8.0%} {coll:>5.0%} {ef:>6.2f} {vmax:>6.1f}")
            print(f"{name:<11} {a_max:>5} | {cells[0]} | {cells[1]}")
    print("\ne_F (m) and v_max (median, m/s) over collision-free runs only.")


if __name__ == "__main__":
    main()
