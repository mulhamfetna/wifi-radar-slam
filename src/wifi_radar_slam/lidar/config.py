from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class LidarConfig:
    """2D horizontal-ring LiDAR model parameters (BEV comparison plane).

    A 3D automotive LiDAR is reduced to a single horizontal ring so its output is
    directly comparable to the WiFi pipeline's xy trajectories and footprint maps.
    """
    angular_res_deg: float   # bearing step between adjacent beams
    fov_deg: float           # total horizontal field of view (360 = full ring)
    max_range_m: float       # beyond this a beam returns no hit
    min_range_m: float       # closer than this is discarded (self-return / blind zone)
    range_sigma_m: float     # per-return Gaussian range noise (std)

    @property
    def n_beams(self) -> int:
        return int(round(self.fov_deg / self.angular_res_deg))

    def bearings(self) -> np.ndarray:
        """Beam bearings in radians, centred on 0 (local +x = forward)."""
        # n_beams samples across the FOV, symmetric about 0
        step = np.deg2rad(self.fov_deg) / self.n_beams
        return (np.arange(self.n_beams) - (self.n_beams - 1) / 2.0) * step


# Preset pinned to the Ouster OS1 datasheet (automotive/mid-range spinning LiDAR):
# range up to ~120 m, range precision ~+/-3 cm, full 360-deg horizontal FOV,
# horizontal angular resolution ~0.35 deg (mid setting). Used for real runs;
# tests may construct coarser LidarConfigs directly.
OUSTER_OS1 = LidarConfig(angular_res_deg=0.35, fov_deg=360.0, max_range_m=120.0,
                         min_range_m=0.5, range_sigma_m=0.03)
