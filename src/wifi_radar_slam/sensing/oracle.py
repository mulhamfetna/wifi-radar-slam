"""Oracle sensing: single-specular-bounce detections straight from Sionna paths.

Bypasses CSI + MUSIC and reads Sionna's ground-truth per-path delay/AoA. Keeps only
single-specular-bounce paths — the sole family the single-reflector bistatic model
can invert (their one reflection vertex is a real facade point). This is the
upper-bound ("oracle") map: it isolates whether the mapping *geometry* is correct
from the harder problem of estimating those delays/angles from commodity CSI.
"""
from __future__ import annotations
import numpy as np
from ..config import RFConfig
from ..scene.builder import BuiltScene
from ..geometry import RX_HEIGHT_M

C = 299792458.0
SPECULAR = 1   # sionna.rt InteractionType: NONE=0 SPECULAR=1 DIFFUSE=2 REFRACTION=4 DIFFRACTION=8


def single_specular_mask(interactions, valid=None) -> np.ndarray:
    """Boolean mask (n_tx, n_paths) of valid single-specular-bounce paths.

    `interactions` is (max_depth, n_tx, n_paths) of InteractionType codes. A
    single-specular-bounce path has exactly one non-NONE interaction across depth
    and that interaction is SPECULAR — so it is a clean AP->facade->vehicle bounce
    with a single reflection point. Multi-bounce, refraction, diffuse and LOS paths
    are rejected (the single-reflector ellipse model does not describe them).
    """
    inter = np.asarray(interactions)
    n_nonzero = np.count_nonzero(inter, axis=0)          # (n_tx, n_paths)
    n_specular = np.sum(inter == SPECULAR, axis=0)       # (n_tx, n_paths)
    mask = (n_nonzero == 1) & (n_specular == 1)
    if valid is not None:
        mask = mask & np.asarray(valid, dtype=bool)
    return mask


def extract_oracle_detections(built: BuiltScene, rf: RFConfig, rng,
                              max_depth: int = 3) -> list[np.ndarray]:
    """Per-frame single-specular-bounce detections (k, 3): [range_m, aoa_rad, ap_index].

    range_m is the bistatic path length (tau * c); aoa_rad is Sionna's `phi_r`
    (azimuth of arrival, world frame). Same output contract as
    `sensing.frontend.extract_detections`, so the SLAM back-end is unchanged.
    """
    import os
    import sionna.rt as rt   # lazy: heavy import, only when solving
    import mitsuba as mi

    n_samples = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
    scene = built.scene
    solver = rt.PathSolver()
    rx = scene.receivers["veh"]
    out: list[np.ndarray] = []
    for f in range(built.trajectory.shape[0]):
        x, y, _yaw = built.trajectory[f]
        rx.position = mi.Point3f(float(x), float(y), RX_HEIGHT_M)
        paths = solver(scene, max_depth=max_depth, samples_per_src=n_samples,
                       seed=int(rng.integers(1, 2**31 - 1)))
        inter = np.asarray(paths.interactions.numpy())[:, 0]   # (depth, n_tx, n_paths)
        tau = np.asarray(paths.tau.numpy())[0]                 # (n_tx, n_paths)
        phir = np.asarray(paths.phi_r.numpy())[0]              # (n_tx, n_paths)
        valid = np.asarray(paths.valid.numpy())[0]             # (n_tx, n_paths)
        mask = single_specular_mask(inter, valid)
        rows = []
        for ap in range(tau.shape[0]):
            for p in np.where(mask[ap])[0]:
                rows.append([float(tau[ap, p]) * C, float(phir[ap, p]), float(ap)])
        out.append(np.array(rows) if rows else np.empty((0, 3)))
    return out
