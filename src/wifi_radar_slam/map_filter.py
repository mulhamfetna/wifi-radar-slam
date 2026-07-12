"""Learned map enhancement (paper 2, RQ2): filter MUSIC detections before they enter
the map, so phantom/LOS/floor/multi-bounce returns stop polluting it.

CRITICAL: the features here are ONLY what a commodity 2-D (delay-azimuth) MUSIC
front-end can actually measure. Paper 1's discriminator also used `elevation`, which a
single ULA never estimates -- an oracle feature. Its F1 ~ 0.9 is therefore optimistic;
we retrain on the observable subset and report the corrected number.

Filter contract (shared by every rung):
    filter(dets, pose, ap_positions) -> np.ndarray[bool] of shape (k,)
Feature extraction happens inside the filter, so slam/particle_filter.py never imports
this module (no circular import).
"""
from __future__ import annotations
import numpy as np

from .slam.particle_filter import _triangulate_bistatic

FEATURE_NAMES = ["path_len_m", "excess_m", "abs_azimuth", "aoa_dev_from_ap"]


def music_features(dets, pose, ap_positions) -> np.ndarray:
    """(k,4) features from MUSIC detections [path_len, aoa, ap_index] + the pose.

    NO elevation, NO interaction type, NO bounce count -- nothing a real 2-D CSI
    receiver could not compute.
    """
    dets = np.asarray(dets, dtype=float).reshape(-1, 3)
    if dets.shape[0] == 0:
        return np.empty((0, 4))
    path_len, aoa = dets[:, 0], dets[:, 1]
    ap_idx = dets[:, 2].astype(int)
    pose_xy = np.asarray(pose, dtype=float)[:2]
    ap_xy = np.array([np.asarray(ap_positions[i], dtype=float)[:2] for i in ap_idx])
    dist_ap = np.linalg.norm(ap_xy - pose_xy, axis=1)
    excess = path_len - dist_ap
    bearing = np.arctan2(ap_xy[:, 1] - pose_xy[1], ap_xy[:, 0] - pose_xy[0])
    aoa_dev = np.abs((aoa - bearing + np.pi) % (2 * np.pi) - np.pi)
    return np.column_stack([path_len, excess, np.abs(aoa), aoa_dev])


def label_from_gt(dets, pose, ap_positions, gt_xy, tol: float = 1.0) -> np.ndarray:
    """Operational label: 1 iff this detection triangulates to a reflector within `tol`
    of a true facade. That IS the definition of a mapping-useful detection. Ground truth
    is used ONLY here (training); inference never sees it.
    """
    dets = np.asarray(dets, dtype=float).reshape(-1, 3)
    y = np.zeros(dets.shape[0], dtype=int)
    gt = np.asarray(gt_xy, dtype=float).reshape(-1, 2)
    if dets.shape[0] == 0 or gt.shape[0] == 0:
        return y
    pose_xy = np.asarray(pose, dtype=float)[:2]
    for k in range(dets.shape[0]):
        path_len, aoa, ap_i = dets[k]
        ap_xy = np.asarray(ap_positions[int(ap_i)], dtype=float)[:2]
        refl = _triangulate_bistatic(pose_xy, ap_xy, path_len, aoa)
        if refl is None:                       # degenerate/LOS solve -> not useful
            continue
        if np.min(np.linalg.norm(gt - refl, axis=1)) <= tol:
            y[k] = 1
    return y


class HeuristicFilter:
    """Rung 1 (physics): keep detections whose bistatic excess clears a threshold.

    LOS and floor-bounce paths barely detour past the direct AP distance; a genuine
    facade reflection detours a lot. This is the existing `map_min_excess_m` gate,
    expressed as a filter so it sits on the same ladder as the learned rungs.
    """

    def __init__(self, min_excess_m: float = 1.5):
        self.min_excess_m = float(min_excess_m)

    def __call__(self, dets, pose, ap_positions) -> np.ndarray:
        X = music_features(dets, pose, ap_positions)
        if X.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        return X[:, 1] >= self.min_excess_m       # column 1 == excess_m


class SklearnFilter:
    """Rungs 2-3 (learned): wrap a fitted sklearn classifier (RandomForest or MLP) in
    the same contract as HeuristicFilter, so run_slam treats every rung identically."""

    def __init__(self, model, threshold: float = 0.5):
        self.model = model
        self.threshold = float(threshold)

    def __call__(self, dets, pose, ap_positions) -> np.ndarray:
        X = music_features(dets, pose, ap_positions)
        if X.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        proba = self.model.predict_proba(X)[:, 1]
        return proba >= self.threshold
