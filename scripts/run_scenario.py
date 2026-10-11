"""Run one scenario YAML file: simulate, save trajectory, figure and GIF.

Usage:
    uv run python scripts/run_scenario.py configs/scenarios/demo_sphere.yaml [--out DIR] [--no-anim]
Format of the YAML file: docs/specs/T05-scenarios-yaml.md.
"""

from uavapf.scenarios.run import main

if __name__ == "__main__":
    raise SystemExit(main())
