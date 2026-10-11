"""Double-integrator swarm simulator: RK4 integration, saturation, trajectory logging (T03)."""

from uavapf.sim.core import (
    Controller,
    ResetFn,
    SimConfig,
    StopFn,
    Trajectory,
    rk4_step,
    saturate_norm,
    simulate,
)

__all__ = [
    "Controller",
    "ResetFn",
    "SimConfig",
    "StopFn",
    "Trajectory",
    "rk4_step",
    "saturate_norm",
    "simulate",
]
