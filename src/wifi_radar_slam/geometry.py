from __future__ import annotations
import numpy as np

RX_HEIGHT_M = 1.5


def straight_trajectory(length_m: float, speed_mps: float, timestep_s: float) -> np.ndarray:
    n = int(round((length_m / speed_mps) / timestep_s))
    x = np.arange(n) * speed_mps * timestep_s
    poses = np.zeros((n, 3))
    poses[:, 0] = x
    return poses


def velocity_from_poses(poses: np.ndarray, timestep_s: float) -> np.ndarray:
    vel = np.zeros((poses.shape[0], 2))
    if poses.shape[0] > 1:
        vel[1:] = (poses[1:, :2] - poses[:-1, :2]) / timestep_s
        vel[0] = vel[1]
    return vel


def mirror_image(ap_xyz: np.ndarray, wall_point: np.ndarray, wall_normal: np.ndarray) -> np.ndarray:
    n = wall_normal / np.linalg.norm(wall_normal)
    d = np.dot(ap_xyz - wall_point, n)
    return ap_xyz - 2.0 * d * n


def targets_to_pointmap(targets: list[dict], spacing: float = 0.5) -> np.ndarray:
    pts = []
    for t in targets:
        cx, cy, cz = t["center"]
        sx, sy, sz = t["size"]
        # sample the six faces of the axis-aligned box on a grid
        xs = np.arange(-sx / 2, sx / 2 + 1e-9, spacing)
        ys = np.arange(-sy / 2, sy / 2 + 1e-9, spacing)
        zs = np.arange(-sz / 2, sz / 2 + 1e-9, spacing)
        for x in xs:
            for y in ys:
                pts.append([cx + x, cy + y, cz - sz / 2])
                pts.append([cx + x, cy + y, cz + sz / 2])
        for x in xs:
            for z in zs:
                pts.append([cx + x, cy - sy / 2, cz + z])
                pts.append([cx + x, cy + sy / 2, cz + z])
        for y in ys:
            for z in zs:
                pts.append([cx - sx / 2, cy + y, cz + z])
                pts.append([cx + sx / 2, cy + y, cz + z])
    return np.unique(np.array(pts), axis=0)
