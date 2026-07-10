from __future__ import annotations
import numpy as np


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


def _nn_idx(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Index into b of the nearest point to each row of a (brute force)."""
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return np.argmin(d, axis=1)


def icp_align(source: np.ndarray, target: np.ndarray,
              init=(0.0, 0.0, 0.0), max_iter: int = 30, tol: float = 1e-5) -> tuple[float, float, float]:
    """Point-to-point ICP: pose (x,y,yaw) mapping source (local) onto target (world)."""
    x, y, yaw = float(init[0]), float(init[1]), float(init[2])
    for _ in range(max_iter):
        src_w = _apply(source, x, y, yaw)
        idx = _nn_idx(src_w, target)
        nx, ny, nyaw = _rigid_2d(source, target[idx])   # absolute source -> matched target
        if abs(nx - x) + abs(ny - y) + abs(nyaw - yaw) < tol:
            x, y, yaw = nx, ny, nyaw
            break
        x, y, yaw = nx, ny, nyaw
    return x, y, yaw
