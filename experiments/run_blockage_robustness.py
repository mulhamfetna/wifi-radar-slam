#!/usr/bin/env python3
"""R1.7 — blockage robustness. How does the PF handle a sudden, total loss of measurements?

Estimator-side, Sionna-free: drops ALL detections for a window of frames (an occlusion) and
measures the drift DURING the blackout and the recovery AFTER it. Both oracle and realistic-MUSIC
regimes, several blackout positions per length.

    .venv/bin/python experiments/run_blockage_robustness.py      # -> results/blockage_robustness.json
"""
from __future__ import annotations

import json

import numpy as np

from wifi_radar_slam.eval import sensitivity as S

LENGTHS = [0, 2, 5, 10, 20, 40]       # blackout length in frames (dataset is 240 frames)
STARTS = [80, 160]                    # place the blackout mid-trajectory
N_SEEDS = 4
RECOVER = 10                          # frames after the blackout used to gauge re-lock


def sweep(dets, ds, label):
    rows = []
    for L in LENGTHS:
        during, after, full = [], [], []
        combos = [(0, 0)] if L == 0 else [(st, sd) for st in STARTS for sd in range(N_SEEDS)]
        for st, sd in combos:
            bo = S.blackout(dets, st, L) if L > 0 else dets
            est, _ = S.localize(bo, ds.ap_positions, ds, np.random.default_rng(1000 + sd))
            during.append(S.window_ate(est, ds, st, max(L, 1)))
            after.append(S.window_ate(est, ds, min(st + L, 239), RECOVER))
            full.append(S.ate(est, ds.poses))
        rows.append({"length_frames": L,
                     "drift_during_mean": float(np.mean(during)),
                     "recover_after_mean": float(np.mean(after)),
                     "full_ate_mean": float(np.mean(full))})
    print(f"\n{label}:  (mean over positions x {N_SEEDS} seeds)")
    print(f"  {'blackout (frames)':>17} {'drift during (m)':>17} {'ATE after (m)':>15} {'full ATE (m)':>13}")
    for r in rows:
        print(f"  {r['length_frames']:>17d} {r['drift_during_mean']:>17.3f}"
              f" {r['recover_after_mean']:>15.3f} {r['full_ate_mean']:>13.3f}")
    return rows


def main():
    ds = S.load_dataset()
    result = {
        "n_seeds": N_SEEDS, "lengths_frames": LENGTHS, "starts": STARTS, "recover_frames": RECOVER,
        "oracle": sweep(S.oracle_detections(ds), ds, "ORACLE detections"),
        "music": sweep(S.music_detections(ds, joint=True), ds, "REALISTIC MUSIC detections"),
    }
    import os
    os.makedirs("results", exist_ok=True)
    with open("results/blockage_robustness.json", "w") as f:
        json.dump(result, f, indent=2)
    print("\nsaved -> results/blockage_robustness.json")


if __name__ == "__main__":
    main()
