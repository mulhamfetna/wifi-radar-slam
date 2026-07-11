"""WiFi + LiDAR fusion (paper 2, RQ4).

Symmetric fusion, deliberately NOT the literature's "WiFi assists LiDAR" shape: our
RQ3 result shows realistic WiFi is the better *localizer* while LiDAR is the only
modality that *maps*, so demoting WiFi to loop closure would waste its strength.

Tight fusion: one particle filter whose weight is the product of two independent
likelihoods -- WiFi bistatic reprojection x LiDAR scan-match -- with the output map the
union of WiFi-triangulated reflectors and LiDAR points.
Loose fusion: a deliberately naive output-level baseline.
"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree

from .slam.particle_filter import (_triangulate_bistatic, _reproject_bistatic, _cluster)
from .lidar.sensor_sionna import _voxel_downsample


def _lidar_likelihood(particles: np.ndarray, scan_pts: np.ndarray,
                      tree: cKDTree, sigma: float) -> np.ndarray:
    """Per-particle scan-match likelihood: place the scan at each particle's pose and
    score the mean nearest-neighbour distance to the accumulated LiDAR map.

    Fully vectorised: all (particle x point) world positions are queried in one KD-tree
    call (workers=-1), so this stays cheap for 200 particles.
    """
    p = np.asarray(particles, dtype=float)
    s = np.asarray(scan_pts, dtype=float).reshape(-1, 2)
    if s.shape[0] == 0:
        return np.ones(p.shape[0])
    cos, sin = np.cos(p[:, 2]), np.sin(p[:, 2])
    x, y = s[:, 0][None, :], s[:, 1][None, :]                     # (1, S)
    wx = cos[:, None] * x - sin[:, None] * y + p[:, 0][:, None]   # (P, S)
    wy = sin[:, None] * x + cos[:, None] * y + p[:, 1][:, None]   # (P, S)
    pts = np.stack([wx.ravel(), wy.ravel()], axis=1)              # (P*S, 2)
    d, _ = tree.query(pts, workers=-1)
    d = d.reshape(p.shape[0], s.shape[0]).mean(axis=1)            # mean NN dist per particle
    return np.exp(-0.5 * d ** 2 / sigma ** 2)


def fuse_loose(wifi_traj, wifi_map, lidar_traj, lidar_map, voxel: float = 0.5):
    """Naive output-level fusion baseline.

    Trajectory = equal-weight average of the two (x, y); yaw is taken from the LiDAR
    back-end, the only one that actually estimates it. Map = voxel-deduplicated union.
    Deliberately naive: weighting by measured accuracy would leak ground truth. Its role
    is to reveal whether tight coupling beats blind combination.
    """
    w = np.asarray(wifi_traj, dtype=float)
    l = np.asarray(lidar_traj, dtype=float)
    n = min(len(w), len(l))
    traj = np.zeros((n, 3))
    traj[:, :2] = 0.5 * (w[:n, :2] + l[:n, :2])
    traj[:, 2] = l[:n, 2]
    parts = [np.asarray(m, dtype=float).reshape(-1, 2)
             for m in (wifi_map, lidar_map) if np.asarray(m).size]
    merged = np.vstack(parts) if parts else np.empty((0, 2))
    return traj, _voxel_downsample(merged, voxel)
