"""Wheel-odometry noise model.

Every published SLAM number in this repo before 2026-10 fed the filter the *ground-truth*
velocity, which made trajectory error an artefact of the filter's process noise. This
module supplies the realistic alternative: a per-run scale-factor error, a slowly drifting
heading error (gyro bias), and white velocity noise. The same seeded noisy velocity is fed
to dead reckoning and to every SLAM back-end, so comparisons are paired.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OdometryNoise:
    scale_sigma: float = 0.01          # 1 % wheel-scale error (per run)
    heading_sigma_deg: float = 0.5     # initial heading error (per run)
    heading_rw_deg_per_sqrt_s: float = 0.2   # gyro-bias random walk on heading
    white_sigma_mps: float = 0.05      # per-sample velocity noise


def noisy_velocity(velocity: np.ndarray, dt: float, noise: OdometryNoise,
                   rng: np.random.Generator) -> np.ndarray:
    """Corrupt a (n, 2) world-frame velocity sequence with odometry error."""
    v = np.asarray(velocity, dtype=float)
    n = v.shape[0]
    scale = 1.0 + rng.normal(0.0, noise.scale_sigma)
    h0 = np.deg2rad(rng.normal(0.0, noise.heading_sigma_deg))
    steps = rng.normal(0.0, np.deg2rad(noise.heading_rw_deg_per_sqrt_s) * np.sqrt(dt), n)
    heading = h0 + np.cumsum(steps)
    c, s = np.cos(heading), np.sin(heading)
    out = np.column_stack([c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1]]) * scale
    return out + rng.normal(0.0, noise.white_sigma_mps, v.shape)


def dead_reckon(velocity: np.ndarray, dt: float, init_pose) -> np.ndarray:
    """Integrate velocity from ``init_pose``; same time alignment as the SLAM filters
    (frame 0 is the initial pose, motion applied from frame 1)."""
    v = np.asarray(velocity, dtype=float)
    out = np.zeros((v.shape[0], 3))
    out[0, :2] = np.asarray(init_pose, dtype=float)[:2]
    out[0, 2] = float(init_pose[2]) if len(init_pose) > 2 else 0.0
    for f in range(1, v.shape[0]):
        out[f, :2] = out[f - 1, :2] + v[f] * dt
        out[f, 2] = out[0, 2]
    return out
