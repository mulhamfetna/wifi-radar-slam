from __future__ import annotations
import numpy as np


def ate(est_traj: np.ndarray, gt_traj: np.ndarray) -> float:
    d = est_traj[:, :2] - gt_traj[:, :2]
    return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))


def rpe(est_traj: np.ndarray, gt_traj: np.ndarray, delta: int = 1) -> float:
    de = est_traj[delta:, :2] - est_traj[:-delta, :2]
    dg = gt_traj[delta:, :2] - gt_traj[:-delta, :2]
    d = de - dg
    return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))


def chamfer(est_map: np.ndarray, gt_map_xy: np.ndarray) -> float:
    if est_map.size == 0 or gt_map_xy.size == 0:
        return float("inf")

    def _nn(a, b):
        return np.mean([np.min(np.linalg.norm(b - p, axis=1)) for p in a])

    return 0.5 * (_nn(est_map, gt_map_xy) + _nn(gt_map_xy, est_map))


def occupancy_iou(est_map, gt_map_xy, cell: float = 1.0, bounds=None) -> float:
    if bounds is None:
        allpts = np.vstack([est_map, gt_map_xy])
        xmin, ymin = allpts.min(0) - cell
        xmax, ymax = allpts.max(0) + cell
    else:
        xmin, xmax, ymin, ymax = bounds

    def _grid(pts):
        gx = np.floor((pts[:, 0] - xmin) / cell).astype(int)
        gy = np.floor((pts[:, 1] - ymin) / cell).astype(int)
        return set(zip(gx.tolist(), gy.tolist()))

    a, b = _grid(est_map), _grid(gt_map_xy)
    if not a and not b:
        return 1.0
    return len(a & b) / len(a | b)
