from __future__ import annotations
import numpy as np
from ..geometry import RX_HEIGHT_M
from .pointcloud import Scan


def _ray_segments_scan(segments, pose, cfg, rng) -> Scan:
    """Ray-cast a 2D LiDAR at `pose` against wall segments (S,2,2).

    For each beam bearing, solve o + t*d = a + u*(b-a) for every segment; a valid
    hit needs u in [0,1] and t in [min_range, max_range]; the nearest such t is the
    return (correct occlusion). Ranges get Gaussian noise; misses drop out.
    """
    segments = np.asarray(segments, dtype=float).reshape(-1, 2, 2)
    px, py = float(pose[0]), float(pose[1])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    o = np.array([px, py])
    if segments.shape[0] == 0:
        return Scan.empty()
    a = segments[:, 0, :]
    e = segments[:, 1, :] - a                       # segment direction vectors (S,2)
    ao = a - o                                      # (S,2)

    out_b, out_r = [], []
    for beam in cfg.bearings():
        ang = yaw + beam
        d = np.array([np.cos(ang), np.sin(ang)])
        denom = d[0] * e[:, 1] - d[1] * e[:, 0]     # cross(d, e)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (ao[:, 0] * e[:, 1] - ao[:, 1] * e[:, 0]) / denom   # cross(ao,e)/denom
            u = (ao[:, 0] * d[1] - ao[:, 1] * d[0]) / denom         # cross(ao,d)/denom
        hit = (np.abs(denom) > 1e-12) & (u >= 0.0) & (u <= 1.0) \
            & (t >= cfg.min_range_m) & (t <= cfg.max_range_m)
        if not hit.any():
            continue
        r = float(t[hit].min()) + rng.normal(0, cfg.range_sigma_m)
        out_b.append(beam)
        out_r.append(r)
    if not out_b:
        return Scan.empty()
    return Scan.from_ranges(np.array(out_b), np.array(out_r))
