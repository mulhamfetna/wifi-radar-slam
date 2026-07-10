from __future__ import annotations
import numpy as np
from .pointcloud import Scan


class ReferenceSensor:
    """Analytic 2D LiDAR over a footprint point set (harness reference / test seam).

    For each beam bearing, the nearest GT point within a perpendicular `gate` of the
    beam line and within [min_range, max_range] is returned, with Gaussian range
    noise. This is the substrate's built-in sensor; models A (mesh ray-cast) and B
    (Sionna optical) replace it on later branches via the same call signature.
    """

    def __init__(self, gt_xy: np.ndarray, cfg, rng, gate: float = 0.6):
        self.gt = np.asarray(gt_xy, dtype=float)[:, :2]
        self.cfg = cfg
        self.rng = rng
        self.gate = gate

    def __call__(self, pose) -> Scan:
        px, py = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        rel = self.gt - np.array([px, py])
        rng_m = np.linalg.norm(rel, axis=1)
        ang = np.arctan2(rel[:, 1], rel[:, 0]) - yaw          # bearing in sensor frame
        in_band = (rng_m >= self.cfg.min_range_m) & (rng_m <= self.cfg.max_range_m)
        out_b, out_r = [], []
        for b in self.cfg.bearings():
            dang = np.arctan2(np.sin(ang - b), np.cos(ang - b))
            perp = rng_m * np.abs(np.sin(dang))                # dist from beam line
            cand = in_band & (np.cos(dang) > 0) & (perp < self.gate)
            if not cand.any():
                continue
            i = np.where(cand)[0][np.argmin(rng_m[cand])]      # nearest hit along beam
            out_b.append(b)
            out_r.append(rng_m[i] + self.rng.normal(0, self.cfg.range_sigma_m))
        if not out_b:
            return Scan.empty()
        return Scan.from_ranges(np.array(out_b), np.array(out_r))


def reference_sensor(built, cfg, rng) -> "ReferenceSensor":
    """make_sensor factory: build a ReferenceSensor from a BuiltScene's footprint GT."""
    return ReferenceSensor(built.ground_truth_map[:, :2], cfg, rng)
