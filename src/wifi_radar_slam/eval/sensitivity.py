"""Estimator-side robustness harness for the two IoT-J reviewer experiments (R1.6, R1.7).

Both experiments perturb what the SLAM *solver* sees — never the propagation — so they run on
the cached WiFiSLAM-Sim dataset with **no Sionna**:

  R1.6  AP-position sensitivity : feed the solver AP positions with injected coordinate error,
                                  measure ATE (trajectory) and map accuracy vs error.
  R1.7  blockage robustness     : drop all detections for a window of frames, measure the drift
                                  during the blackout and whether the PF re-locks afterwards.

Detections are **oracle single-bounce** bistatic returns built from the dataset's true path
table (columns in ``dataset.PATH_COLUMNS``). Oracle detections isolate the variable under study
(AP error / measurement loss) from front-end estimation noise — the clean way to answer a
"what is the tolerance?" question. A realistic-MUSIC variant is provided separately for the
combined effect.
"""
from __future__ import annotations

import numpy as np

from ..dataset import CsiDataset, PATH_COLUMNS
from ..geometry import velocity_from_poses
from ..slam.particle_filter import run_slam
from .metrics import ate, map_accuracy

C = 299792458.0
DEFAULT_DATASET = "data/wifislam_sim_nominal.npz"
DEFAULT_DT = 0.1                       # timestep cancels (velocity_from_poses scales by 1/dt)
_COL = {c: i for i, c in enumerate(PATH_COLUMNS)}


def load_dataset(path: str = DEFAULT_DATASET) -> CsiDataset:
    return CsiDataset.load(path)


def oracle_detections(ds: CsiDataset) -> list[np.ndarray]:
    """Per-frame oracle bistatic detections: one row ``[path_len_m, aoa_rad, ap_index]``.

    Uses **single-bounce** paths only (the single-reflector bistatic model the estimator
    assumes); LOS (n_bounce 0) and multi-bounce are excluded. Returns a list of length
    ``n_frames``; each entry is an ``(k, 3)`` array (possibly empty).
    """
    P = ds.paths
    frame = P[:, _COL["frame"]]
    single = P[:, _COL["n_bounce"]] == 1
    path_len = P[:, _COL["delay_s"]] * C
    aoa = P[:, _COL["phi_r"]]
    ap = P[:, _COL["ap"]]
    n_frames = ds.poses.shape[0]
    out: list[np.ndarray] = []
    for f in range(n_frames):
        m = single & (frame == f)
        out.append(np.column_stack([path_len[m], aoa[m], ap[m]]) if m.any()
                   else np.empty((0, 3)))
    return out


def perturb_ap_positions(ap_positions: np.ndarray, sigma_m: float,
                         rng: np.random.Generator) -> np.ndarray:
    """Return a copy of the AP positions with i.i.d. Gaussian error (std ``sigma_m``) on x,y."""
    ap = np.asarray(ap_positions, dtype=float).copy()
    if sigma_m > 0:
        ap[:, :2] += rng.normal(0.0, sigma_m, size=ap[:, :2].shape)
    return ap


def blackout(detections: list[np.ndarray], start: int, length: int) -> list[np.ndarray]:
    """Return a copy of ``detections`` with frames ``[start, start+length)`` emptied.

    Models a sudden, total loss of measurements (an occlusion/blockage) for ``length`` frames.
    """
    out = [d.copy() for d in detections]
    for f in range(start, min(start + length, len(out))):
        out[f] = np.empty((0, 3))
    return out


def localize(detections: list[np.ndarray], ap_positions: np.ndarray, ds: CsiDataset,
             rng: np.random.Generator, dt: float = DEFAULT_DT, n_particles: int = 200):
    """Run the actual particle-filter SLAM with GT-derived odometry. Returns (est_traj, est_map)."""
    velocity = velocity_from_poses(ds.poses, dt)
    return run_slam(detections, ap_positions, velocity, dt, rng,
                    n_particles=n_particles, init_pose=ds.poses[0])


def evaluate(est_traj: np.ndarray, est_map: np.ndarray, ds: CsiDataset) -> dict:
    """Trajectory ATE and map accuracy against the dataset ground truth."""
    return {"ate_m": ate(est_traj, ds.poses),
            "map_acc_m": map_accuracy(est_map, ds.gt_map) if len(est_map) else float("nan")}


def window_ate(est_traj: np.ndarray, ds: CsiDataset, start: int, length: int) -> float:
    """ATE computed only over frames ``[start, start+length)`` — the drift during a blackout."""
    end = min(start + length, est_traj.shape[0])
    d = est_traj[start:end, :2] - ds.poses[start:end, :2]
    return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1)))) if end > start else float("nan")
