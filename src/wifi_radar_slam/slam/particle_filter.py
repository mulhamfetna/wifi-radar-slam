from __future__ import annotations
import numpy as np


def _reproject(pose, reflector):
    d = reflector - pose[:2]
    return np.linalg.norm(d), np.arctan2(d[1], d[0])


def run_slam(detections, ap_positions, velocity, timestep_s, rng,
             n_particles: int = 200):
    n_frames = len(detections)
    particles = np.zeros((n_particles, 3))                 # x, y, yaw
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
            # anchor reflector triangulation on the current weighted-mean pose
            mean_pose = np.average(particles, axis=0, weights=weights)
            for rng_m, aoa, _ap in dets:
                refl = mean_pose[:2] + rng_m * np.array([np.cos(aoa), np.sin(aoa)])
                mapped_points.append(refl)
                # weight update: consistency of each particle with this detection
                pr = np.array([_reproject(p, refl) for p in particles])
                err = (pr[:, 0] - rng_m) ** 2 + (pr[:, 1] - aoa) ** 2
                weights *= np.exp(-0.5 * err / (0.5 ** 2))
            weights += 1e-300
            weights /= weights.sum()

            # resample if effective sample size collapses
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
