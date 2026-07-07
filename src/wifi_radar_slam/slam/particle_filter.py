from __future__ import annotations
import numpy as np

# Reflectors farther than this from the vehicle are rejected as non-physical
# (ill-conditioned bistatic solves or array-relative AoA inconsistencies).
MAX_REFLECTOR_RANGE_M = 150.0


def _triangulate_bistatic(pose_xy, ap_xy, path_len, aoa):
    """Locate a reflector from a bistatic path length + angle of arrival.

    The measured path length is |AP->R| + |R->vehicle| (an ellipse with foci AP
    and vehicle). The AoA gives the bearing from the vehicle to R, so
    R = vehicle + s * u(aoa). Solving |AP - R| = path_len - s for s:

        s = (path_len^2 - |AP-vehicle|^2) / (2 (path_len - (AP-vehicle) . u))

    Returns None for the direct/LOS path (s <= 0) or degenerate geometry.
    """
    u = np.array([np.cos(aoa), np.sin(aoa)])
    v2ap = np.asarray(ap_xy, dtype=float) - np.asarray(pose_xy, dtype=float)
    dist_ap = np.linalg.norm(v2ap)
    denom = 2.0 * (path_len - v2ap @ u)
    if abs(denom) < 1.0:               # near-degenerate ellipse -> unstable solve
        return None
    s = (path_len ** 2 - dist_ap ** 2) / denom
    if s <= 0.1 or s > MAX_REFLECTOR_RANGE_M:   # direct path, behind, or implausible
        return None
    return np.asarray(pose_xy, dtype=float) + s * u


def _reproject_bistatic(pose_xy, ap_xy, refl):
    """Predicted (bistatic path length, AoA) of a reflector from a pose."""
    d = np.asarray(refl) - np.asarray(pose_xy)
    path = np.linalg.norm(np.asarray(ap_xy)[:2] - np.asarray(refl)) + np.linalg.norm(d)
    return path, np.arctan2(d[1], d[0])


def run_slam(detections, ap_positions, velocity, timestep_s, rng,
             n_particles: int = 200, init_pose=None):
    n_frames = len(detections)
    particles = np.zeros((n_particles, 3))                 # x, y, yaw
    if init_pose is not None:                              # known start (e.g. GPS prior)
        particles[:, 0] = init_pose[0]
        particles[:, 1] = init_pose[1]
        particles[:, 2] = init_pose[2] if len(init_pose) > 2 else 0.0
    weights = np.ones(n_particles) / n_particles
    est_traj = np.zeros((n_frames, 3))
    mapped_points: list[np.ndarray] = []

    pos_noise = 0.05
    for f in range(n_frames):
        # frame 0 is the initial pose; propagate from frame 1 onward so the
        # estimated trajectory stays time-aligned with ground truth (no lead).
        if f > 0:
            vx, vy = velocity[f]
            particles[:, 0] += vx * timestep_s + rng.normal(0, pos_noise, n_particles)
            particles[:, 1] += vy * timestep_s + rng.normal(0, pos_noise, n_particles)

        dets = detections[f]
        if dets.shape[0] > 0:
            mean_pose = np.average(particles, axis=0, weights=weights)
            for path_len, aoa, ap_i in dets:
                ap_xy = np.asarray(ap_positions[int(ap_i)])[:2]
                refl = _triangulate_bistatic(mean_pose[:2], ap_xy, path_len, aoa)
                if refl is None:                           # direct path / degenerate
                    continue
                mapped_points.append(refl)
                # weight update: bistatic consistency of each particle
                pr = np.array([_reproject_bistatic(p[:2], ap_xy, refl) for p in particles])
                err = (pr[:, 0] - path_len) ** 2 + (pr[:, 1] - aoa) ** 2
                weights *= np.exp(-0.5 * err / (0.5 ** 2))
            weights += 1e-300
            weights /= weights.sum()

            neff = 1.0 / np.sum(weights ** 2)
            if neff < n_particles / 2:
                idx = rng.choice(n_particles, n_particles, p=weights)
                particles = particles[idx]
                weights = np.ones(n_particles) / n_particles

        est_traj[f] = np.average(particles, axis=0, weights=weights)

    est_map = _cluster(np.array(mapped_points)) if mapped_points else np.empty((0, 2))
    return est_traj, est_map


def _cluster(points: np.ndarray, radius: float = 0.5) -> np.ndarray:
    """Greedy merge of nearby mapped points into landmark centroids."""
    kept = []
    used = np.zeros(len(points), dtype=bool)
    for i in range(len(points)):
        if used[i]:
            continue
        d = np.linalg.norm(points - points[i], axis=1)
        group = d < radius
        used |= group
        kept.append(points[group].mean(axis=0))
    return np.array(kept)
