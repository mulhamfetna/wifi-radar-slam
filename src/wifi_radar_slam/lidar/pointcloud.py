from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class Scan:
    """A single 2D LiDAR scan: points in the sensor-local frame (+x forward)."""
    points: np.ndarray        # (N, 2)

    def __len__(self) -> int:
        return int(self.points.shape[0])

    def to_world(self, pose) -> np.ndarray:
        """Transform local points to world via pose (x, y, yaw): rotate then translate."""
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        if len(self) == 0:
            return np.empty((0, 2))
        return self.points @ R.T + np.array([x, y])

    @classmethod
    def from_ranges(cls, bearings: np.ndarray, ranges: np.ndarray) -> "Scan":
        """Build a scan from per-beam bearings (rad) and ranges; drop non-finite."""
        bearings = np.asarray(bearings, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        ok = np.isfinite(ranges)
        b, r = bearings[ok], ranges[ok]
        pts = np.stack([r * np.cos(b), r * np.sin(b)], axis=1) if r.size else np.empty((0, 2))
        return cls(pts)

    @staticmethod
    def empty() -> "Scan":
        return Scan(np.empty((0, 2)))
