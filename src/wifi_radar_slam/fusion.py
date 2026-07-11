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


def run_fused_slam(detections, scans, ap_positions, velocity, timestep_s: float, rng,
                   n_particles: int = 200, init_pose=None, map_min_support: int = 1,
                   map_min_excess_m: float = 0.0, sigma_lidar: float = 0.5,
                   scan_subsample: int = 100, voxel: float = 0.5):
    """Tight WiFi+LiDAR fusion: one particle filter, two independent likelihoods.

    weight = w_wifi(bistatic reprojection) x w_lidar(scan match vs the accumulated
    LiDAR map). The scan-match target is the LiDAR map ONLY -- WiFi reflectors are too
    noisy under realistic CSI to be a registration target. The OUTPUT map is the union
    of WiFi-triangulated reflectors and LiDAR points.
    """
    n_frames = len(detections)
    particles = np.zeros((n_particles, 3))
    if init_pose is not None:
        particles[:, 0] = init_pose[0]
        particles[:, 1] = init_pose[1]
        particles[:, 2] = init_pose[2] if len(init_pose) > 2 else 0.0
    weights = np.ones(n_particles) / n_particles
    est_traj = np.zeros((n_frames, 3))
    wifi_points: list[np.ndarray] = []
    lidar_cells: dict[tuple[int, int], np.ndarray] = {}

    def _accumulate_lidar(world_pts: np.ndarray) -> None:
        for p in world_pts:
            key = (int(round(p[0] / voxel)), int(round(p[1] / voxel)))
            lidar_cells.setdefault(key, p)

    pos_noise = 0.05
    for f in range(n_frames):
        if f > 0:
            vx, vy = velocity[f]
            particles[:, 0] += vx * timestep_s + rng.normal(0, pos_noise, n_particles)
            particles[:, 1] += vy * timestep_s + rng.normal(0, pos_noise, n_particles)

        updated = False

        dets = detections[f]                       # --- WiFi bistatic likelihood ---
        if dets.shape[0] > 0:
            mean_pose = np.average(particles, axis=0, weights=weights)
            for path_len, aoa, ap_i in dets:
                ap_xy = np.asarray(ap_positions[int(ap_i)])[:2]
                refl = _triangulate_bistatic(mean_pose[:2], ap_xy, path_len, aoa,
                                             min_excess_m=map_min_excess_m)
                if refl is None:
                    continue
                wifi_points.append(refl)
                pr = np.array([_reproject_bistatic(p[:2], ap_xy, refl) for p in particles])
                err = (pr[:, 0] - path_len) ** 2 + (pr[:, 1] - aoa) ** 2
                weights *= np.exp(-0.5 * err / (0.5 ** 2))
            updated = True

        scan = scans[f]                            # --- LiDAR scan-match likelihood ---
        if len(scan) > 0 and len(lidar_cells) >= 3:
            target = np.array(list(lidar_cells.values()))
            pts = scan.points
            if scan_subsample and pts.shape[0] > scan_subsample:
                pts = pts[rng.choice(pts.shape[0], scan_subsample, replace=False)]
            weights *= _lidar_likelihood(particles, pts, cKDTree(target), sigma_lidar)
            updated = True

        if updated:
            weights += 1e-300
            weights /= weights.sum()
            neff = 1.0 / np.sum(weights ** 2)
            if neff < n_particles / 2:
                idx = rng.choice(n_particles, n_particles, p=weights)
                particles = particles[idx]
                weights = np.ones(n_particles) / n_particles

        est_traj[f] = np.average(particles, axis=0, weights=weights)
        if len(scan) > 0:
            _accumulate_lidar(scan.to_world(est_traj[f]))

    wifi_map = (_cluster(np.array(wifi_points), min_support=map_min_support)
                if wifi_points else np.empty((0, 2)))
    lidar_map = (np.array(list(lidar_cells.values())) if lidar_cells
                 else np.empty((0, 2)))
    parts = [m for m in (wifi_map, lidar_map) if m.size]
    est_map = _voxel_downsample(np.vstack(parts), voxel) if parts else np.empty((0, 2))
    return est_traj, est_map
