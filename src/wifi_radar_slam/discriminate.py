"""Learned path discrimination — feature extraction + labels.

The mapping bottleneck is telling a genuine facade reflection from LOS/floor/multi-
bounce paths, which commodity CSI does not label. Here we test whether that label
is *learnable* from per-path physical features that do not require the ground-truth
interaction type. Given the dataset's oracle path table (with bounce/interaction/
floor labels only used as the *training target*), `path_features` builds a feature
matrix and a binary label: is this a mapping-useful single-scatter facade reflection
(exactly one interaction, not the floor)?
"""
from __future__ import annotations
import numpy as np

C = 299792458.0
FEATURE_NAMES = ["range_m", "excess_m", "abs_azimuth", "elevation", "aoa_dev_from_ap"]


def path_features(paths: np.ndarray, poses: np.ndarray, ap_positions: np.ndarray):
    """Feature matrix X (P, 5), binary label y (P,), and feature names.

    `paths` columns: [frame, ap, delay_s, phi_r, theta_r, n_bounce,
    first_interaction_type, object_id, is_floor]. Features use only quantities a
    real receiver could estimate (path length, its bistatic excess over the direct
    AP distance, arrival azimuth/elevation, and azimuth deviation from the AP
    bearing) — NOT the interaction type. `y = 1` iff the path is a single-scatter,
    non-floor reflection (the mapping-useful class).
    """
    frame = paths[:, 0].astype(int)
    ap = paths[:, 1].astype(int)
    delay, phi, theta = paths[:, 2], paths[:, 3], paths[:, 4]
    n_bounce, is_floor = paths[:, 5], paths[:, 8]

    rng_m = delay * C
    pose_xy = poses[frame, :2]
    ap_xy = ap_positions[ap, :2]
    dist_ap = np.linalg.norm(ap_xy - pose_xy, axis=1)
    excess = rng_m - dist_ap
    bearing = np.arctan2(ap_xy[:, 1] - pose_xy[:, 1], ap_xy[:, 0] - pose_xy[:, 0])
    aoa_dev = np.abs((phi - bearing + np.pi) % (2 * np.pi) - np.pi)

    X = np.column_stack([rng_m, excess, np.abs(phi), theta, aoa_dev])
    y = ((n_bounce == 1) & (is_floor == 0)).astype(int)
    return X, y, list(FEATURE_NAMES)
