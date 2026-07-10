"""LiDAR model B: Sionna ray-traced optical-return proxy (monostatic + diffuse).

Pure helpers here are NumPy-only and test locally. The Sionna PathSolver machinery
(SionnaLidarSensor) lazily imports sionna.rt/mitsuba inside its methods, so this
module imports without Sionna and only *running* the sensor needs the amd server.
"""
from __future__ import annotations
import numpy as np
from ..geometry import RX_HEIGHT_M
from .pointcloud import Scan


def _voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point per `voxel`-sized xy cell (caps point density)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return pts
    seen: dict[tuple[int, int], np.ndarray] = {}
    for p in pts:
        key = (int(round(p[0] / voxel)), int(round(p[1] / voxel)))
        seen.setdefault(key, p)
    return np.array(list(seen.values()))


def vertices_to_scan(world_hits, pose, cfg, rng, scan_voxel: float = 0.2) -> Scan:
    """Convert world-frame hit points to a sensor-local Scan.

    Filters by [min_range, max_range], adds radial Gaussian range noise
    (cfg.range_sigma_m), rotates world->local by -yaw, and voxel-downsamples.
    """
    world_hits = np.asarray(world_hits, dtype=float).reshape(-1, 2)
    px, py = float(pose[0]), float(pose[1])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    if world_hits.shape[0] == 0:
        return Scan.empty()
    rel = world_hits - np.array([px, py])
    r = np.linalg.norm(rel, axis=1)
    keep = (r >= cfg.min_range_m) & (r <= cfg.max_range_m)
    rel, r = rel[keep], r[keep]
    if rel.shape[0] == 0:
        return Scan.empty()
    if cfg.range_sigma_m > 0:
        u = rel / np.maximum(r[:, None], 1e-9)
        rel = rel + u * rng.normal(0, cfg.range_sigma_m, size=r.shape)[:, None]
    c, s = np.cos(-yaw), np.sin(-yaw)          # world -> local: rotate by -yaw
    R = np.array([[c, -s], [s, c]])
    local = rel @ R.T
    return Scan(_voxel_downsample(local, scan_voxel))
