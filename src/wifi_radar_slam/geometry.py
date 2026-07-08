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


def footprint_points(bbmin, bbmax, spacing: float = 1.0) -> np.ndarray:
    """Sample the xy-perimeter (footprint outline) of an axis-aligned bounding box.

    The SLAM map is reconstructed in the ground plane; specular reflections land on
    vertical facades, whose xy locations trace the footprint rectangle of the
    scatterer (a building/car AABB). Returns (M, 2) points on that rectangle's edges
    — the right ground-truth reference for map Chamfer/IoU (facades, not centroids).
    """
    x0, y0 = float(bbmin[0]), float(bbmin[1])
    x1, y1 = float(bbmax[0]), float(bbmax[1])
    xs = np.arange(x0, x1 + 1e-9, spacing)
    ys = np.arange(y0, y1 + 1e-9, spacing)
    pts = []
    for x in xs:
        pts.append([x, y0]); pts.append([x, y1])
    for y in ys:
        pts.append([x0, y]); pts.append([x1, y])
    return np.unique(np.array(pts), axis=0) if pts else np.empty((0, 2))


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
