#!/usr/bin/env python3
"""R1.6 — AP-position sensitivity. How does AP coordinate error degrade ATE and the map?

Estimator-side, Sionna-free: perturbs the AP positions fed to the solver by Gaussian error and
runs the real particle-filter SLAM on the cached WiFiSLAM-Sim dataset. Reports both the oracle
(clean-isolation) and realistic-MUSIC (dense) detection regimes.

    .venv/bin/python experiments/run_ap_sensitivity.py            # -> results/ap_sensitivity.json
"""
from __future__ import annotations

import json

import numpy as np

from wifi_radar_slam.eval import sensitivity as S
from wifi_radar_slam.eval.metrics import chamfer, map_completeness

# Extended to extreme error to find the break-point: the trajectory stays flat throughout
# (structurally odometry-anchored), while the map degrades monotonically.
SIGMAS = [0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]
N_SEEDS = 6


def sweep(dets, ds, label):
    rows = []
    for sig in SIGMAS:
        ate, macc, cham, compl = [], [], [], []
        for seed in range(N_SEEDS):
            ap = S.perturb_ap_positions(ds.ap_positions, sig, np.random.default_rng(seed))
            est, emap = S.localize(dets, ap, ds, np.random.default_rng(1000 + seed))
            ate.append(S.ate(est, ds.poses))
            if len(emap):
                macc.append(S.map_accuracy(emap, ds.gt_map))
                cham.append(chamfer(emap, ds.gt_map))
                compl.append(map_completeness(emap, ds.gt_map))
        rows.append({"sigma_m": sig,
                     "ate_mean": float(np.mean(ate)), "ate_std": float(np.std(ate)),
                     "map_acc_mean": float(np.nanmean(macc)), "map_acc_std": float(np.nanstd(macc)),
                     "chamfer_mean": float(np.nanmean(cham)),
                     "completeness_mean": float(np.nanmean(compl))})
    print(f"\n{label}:  (mean +/- std over {N_SEEDS} seeds)")
    print(f"  {'AP err (m)':>10} {'ATE (m)':>16} {'map acc (m)':>13} {'chamfer':>9} {'compl':>7}")
    for r in rows:
        print(f"  {r['sigma_m']:>10.2f} {r['ate_mean']:>8.3f} +/-{r['ate_std']:<5.3f}"
              f" {r['map_acc_mean']:>13.3f} {r['chamfer_mean']:>9.3f} {r['completeness_mean']:>7.3f}")
    return rows


def main():
    ds = S.load_dataset()
    result = {
        "n_seeds": N_SEEDS, "sigmas_m": SIGMAS,
        "oracle": sweep(S.oracle_detections(ds), ds, "ORACLE detections (clean isolation)"),
        "music": sweep(S.music_detections(ds, joint=True), ds, "REALISTIC MUSIC detections (dense)"),
    }
    import os
    os.makedirs("results", exist_ok=True)
    with open("results/ap_sensitivity.json", "w") as f:
        json.dump(result, f, indent=2)
    print("\nsaved -> results/ap_sensitivity.json")


if __name__ == "__main__":
    main()
