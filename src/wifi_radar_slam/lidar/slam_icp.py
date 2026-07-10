from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


def _apply(pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Apply pose (x, y, yaw) to local points: rotate by yaw, then translate."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T + np.array([x, y])


def _rigid_2d(src: np.ndarray, dst: np.ndarray) -> tuple[float, float, float]:
    """Closed-form least-squares rigid transform mapping matched src -> dst (2D Kabsch)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                 # reflect -> proper rotation
        Vt = Vt.copy()
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(t[0]), float(t[1]), float(yaw)


def icp_align(source: np.ndarray, target: np.ndarray,
              init=(0.0, 0.0, 0.0), max_iter: int = 30, tol: float = 1e-5) -> tuple[float, float, float]:
    """Point-to-point ICP: pose (x,y,yaw) mapping source (local) onto target (world).

    Nearest neighbours use a KD-tree built once on the (fixed) target and queried
    with `workers=-1` (all cores) each iteration — exact NN, but O(n log n) instead
    of the O(n*m) brute-force distance matrix, and multi-core. Results are identical
    to brute force, so downstream metrics are unchanged.
    """
    x, y, yaw = float(init[0]), float(init[1]), float(init[2])
    tree = cKDTree(target)
    for _ in range(max_iter):
        src_w = _apply(source, x, y, yaw)
        _, idx = tree.query(src_w, workers=-1)
        nx, ny, nyaw = _rigid_2d(source, target[idx])   # absolute source -> matched target
        if abs(nx - x) + abs(ny - y) + abs(nyaw - yaw) < tol:
            x, y, yaw = nx, ny, nyaw
            break
        x, y, yaw = nx, ny, nyaw
    return x, y, yaw


def run_lidar_slam(scans, velocity, timestep_s: float, rng,
                   init_pose=None, voxel: float = 0.5, progress=None):
    """Scan-to-map ICP SLAM: motion prediction corrected by ICP against a
    voxel-downsampled accumulated map. Returns (est_traj (n,3), est_map (M,2)).

    Motion prior for each frame's ICP init: if `velocity` is given (n,2), predict
    `est[f-1] + velocity[f]*dt` (used by the sim runs, where scans and velocity share
    one world frame). If `velocity is None`, use a **frame-agnostic adaptive
    constant-velocity** model — repeat the previous *estimated* motion
    `est[f-1] + (est[f-1]-est[f-2])` — which is correct in the SLAM's own frame even
    when the ground-truth trajectory lives in a different frame (e.g. real KITTI).

    `progress`, if given, is called after each frame as
    `progress(frame_index, n_frames, n_scan_points, n_map_cells)` for live logging.
    """
    n = len(scans)
    est = np.zeros((n, 3))
    if init_pose is not None:
        est[0, 0], est[0, 1] = float(init_pose[0]), float(init_pose[1])
        est[0, 2] = float(init_pose[2]) if len(init_pose) > 2 else 0.0

    map_cells: dict[tuple[int, int], np.ndarray] = {}

    def _accumulate(world_pts: np.ndarray) -> None:
        for p in world_pts:
            key = (int(round(p[0] / voxel)), int(round(p[1] / voxel)))
            map_cells.setdefault(key, p)                # first point wins the cell

    _accumulate(scans[0].to_world(est[0]))
    for f in range(1, n):
        if velocity is not None:
            vx, vy = velocity[f]
            pred = (est[f - 1, 0] + vx * timestep_s,
                    est[f - 1, 1] + vy * timestep_s,
                    est[f - 1, 2])
        elif f >= 2:                                    # adaptive constant velocity
            pred = (2 * est[f - 1, 0] - est[f - 2, 0],
                    2 * est[f - 1, 1] - est[f - 2, 1],
                    est[f - 1, 2])
        else:                                           # first step: no motion prior
            pred = (est[f - 1, 0], est[f - 1, 1], est[f - 1, 2])
        target = np.array(list(map_cells.values()))
        src = scans[f].points
        if len(src) >= 3 and target.shape[0] >= 3:
            est[f] = icp_align(src, target, init=pred)
        else:                                           # too sparse -> dead-reckon
            est[f] = pred
        _accumulate(scans[f].to_world(est[f]))
        if progress is not None:
            progress(f, n, len(src), len(map_cells))

    est_map = np.array(list(map_cells.values())) if map_cells else np.empty((0, 2))
    return est, est_map
