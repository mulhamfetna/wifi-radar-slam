"""Accumulate a world map from scans placed at GROUND-TRUTH poses -- no estimator.

Paper 3 scores every cell this way, deliberately. Sub-project 2 established that our shared
point-based back-end cannot estimate rotation from spinning radar at all (the registration cost
is flat in yaw), so any SLAM-based comparison would have handed radar an artificially weak
trajectory and flattered WiFi by contrast.

Removing the estimator entirely is not a retreat -- it is a STRONGER experiment. With no pose
error in the loop for anyone, every difference between cells is attributable to the sensor's own
physics, which is exactly what the ablation is for.
"""
from __future__ import annotations
import numpy as np


def map_under_gt_poses(scans, poses, voxel: float = 0.5) -> np.ndarray:
    """Place each scan at its true pose and accumulate a voxel-thinned world map.

    Args:
        scans: list of Scan (sensor-local points, +x forward).
        poses: (n, 3) ground-truth (x, y, yaw).
        voxel: cell size (m); one point survives per cell.

    Returns an (M, 2) world-frame map.
    """
    poses = np.asarray(poses, dtype=float)
    if len(scans) != poses.shape[0]:
        raise ValueError(f"{len(scans)} scans but {poses.shape[0]} poses")

    cells: dict[tuple[int, int], np.ndarray] = {}
    for scan, pose in zip(scans, poses):
        if len(scan) == 0:
            continue
        for p in scan.to_world(pose):                    # rotate by yaw, then translate
            cells.setdefault((int(round(p[0] / voxel)), int(round(p[1] / voxel))), p)
    return np.array(list(cells.values())) if cells else np.empty((0, 2))
