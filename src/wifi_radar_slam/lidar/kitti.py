"""KITTI odometry external-validity: real velodyne LiDAR through our ICP SLAM.

Pure NumPy loaders + trajectory alignment; no network here (fetch is a separate
experiment script). BEV: velodyne points sliced to a horizontal band -> (x,y);
KITTI GT poses (camera-to-world) + Tr (velodyne->camera) -> velodyne (x,z) track.
"""
from __future__ import annotations
import numpy as np
from .pointcloud import Scan
from .slam_icp import _rigid_2d, _apply
from .sensor_sionna import _voxel_downsample


def load_velodyne_scan(path: str, z_lo: float = -0.5, z_hi: float = 0.5,
                       voxel: float | None = None) -> Scan:
    """Read a KITTI velodyne .bin (N x [x,y,z,reflectance]) and slice a horizontal
    band into a 2D BEV Scan (points already in the sensor-local velodyne frame).

    `voxel` (m), if set, downsamples the sliced points to one per cell — KITTI scans
    are ~10k points/frame after slicing, which makes brute-force ICP NN very slow, so
    the SLAM run passes a voxel size; loaders/tests default to no downsampling.
    """
    pts = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    band = (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
    xy = pts[band, :2].astype(float)
    if voxel is not None:
        xy = _voxel_downsample(xy, voxel)
    return Scan(xy)


def _parse_pose_matrices(text: str) -> np.ndarray:
    rows = [list(map(float, ln.split())) for ln in text.strip().splitlines() if ln.strip()]
    return np.array(rows).reshape(-1, 3, 4)          # (N,3,4) camera-to-world


def _parse_calib_Tr(text: str) -> np.ndarray:
    for ln in text.strip().splitlines():
        if ln.startswith("Tr:"):
            return np.array(list(map(float, ln.split()[1:]))).reshape(3, 4)
    raise ValueError("no 'Tr:' line in calib text")


def load_gt_trajectory(poses_text: str, calib_text: str) -> np.ndarray:
    """BEV ground-truth trajectory (N,2) of the velodyne origin, as KITTI (x,z)."""
    P = _parse_pose_matrices(poses_text)             # (N,3,4)
    Tr = _parse_calib_Tr(calib_text)                 # (3,4) velo->cam
    tr_t = Tr[:, 3]                                   # velo origin in camera frame
    R, t = P[:, :, :3], P[:, :, 3]                   # (N,3,3), (N,3)
    velo_world = np.einsum("nij,j->ni", R, tr_t) + t  # (N,3) in cam-0 world
    return velo_world[:, [0, 2]]                      # BEV ground plane (x, z)


def align_2d_ate(est_xy, gt_xy) -> float:
    """Rigidly align est to GT (2D Kabsch) and return ATE (RMSE of positions)."""
    est_xy = np.asarray(est_xy, dtype=float)[:, :2]
    gt_xy = np.asarray(gt_xy, dtype=float)[:, :2]
    n = min(len(est_xy), len(gt_xy))
    est_xy, gt_xy = est_xy[:n], gt_xy[:n]
    x, y, yaw = _rigid_2d(est_xy, gt_xy)
    aligned = _apply(est_xy, x, y, yaw)
    return float(np.sqrt(np.mean(np.sum((aligned - gt_xy) ** 2, axis=1))))
