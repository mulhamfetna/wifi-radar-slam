#!/usr/bin/env python3
"""Reviewer R1.6/R1.7 re-run on the corrected pipeline (noisy odometry, excess delays,
virtual-anchor FastSLAM). Replaces results/{ap_sensitivity,blockage_robustness}.json, whose
"structural robustness" came from ground-truth odometry and a measurement update that carried
no position information.

AP error now matters: the excess-delay conversion uses |AP - p|, so a wrong AP position biases
every range. Blackouts now matter: with drifting odometry the filter must re-acquire.

    .venv/bin/python experiments/run_robustness_corrected.py -> results/robustness_corrected.json
"""
from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from make_music_detections import SCENES, MAIN_BW, load_dets  # noqa: E402

C = 299792458.0
AP_SIGMAS = [0.0, 0.5, 1.0, 2.0, 4.0, 8.0]
BLACKOUTS = [0, 10, 20, 40, 80]
SEEDS = range(5)
RUN_SCENES = ("controlled_wall", "street_canyon")


def job(args):
    scene, seed = args
    from wifi_radar_slam.eval.scenes import SceneCache, excess_oracle_detections
    from wifi_radar_slam.geometry import velocity_from_poses
    from wifi_radar_slam.odometry import OdometryNoise, noisy_velocity, dead_reckon
    from wifi_radar_slam.slam.fastslam import run_fastslam, FastSlamParams
    from wifi_radar_slam.eval.metrics import ate, map_accuracy, occupancy_iou
    sc = SceneCache.load(SCENES[scene]); gt = sc.poses
    v = noisy_velocity(velocity_from_poses(gt, sc.dt), sc.dt, OdometryNoise(),
                       np.random.default_rng(1000 + seed))
    dr = ate(dead_reckon(v, sc.dt, gt[0]), gt)
    fronts = {"oracle": (excess_oracle_detections(sc), FastSlamParams(sigma_range=0.3, sigma_aoa_deg=2.0)),
              "music": (load_dets(scene, MAIN_BW, seed),
                        FastSlamParams(sigma_range=0.5 * C / (MAIN_BW * 1e6), sigma_aoa_deg=5.0))}
    out = {"scene": scene, "seed": seed, "ate_dead_reckoning": dr, "ap": {}, "blackout": {}}
    for fname, (dets, par) in fronts.items():
        def run(d, aps):
            e, m, _ = run_fastslam(d, aps, v, sc.dt, np.random.default_rng(2000 + seed),
                                   init_pose=gt[0], params=par, excess_delay=True)
            return {"ate": ate(e, gt),
                    "map_accuracy": map_accuracy(m, sc.gt_map) if len(m) else float("nan"),
                    "iou": occupancy_iou(m, sc.gt_map, cell=1.0) if len(m) else 0.0}
        out["ap"][fname] = []
        for s in AP_SIGMAS:
            aps = sc.ap_positions.copy()
            aps[:, :2] += np.random.default_rng(500 + seed).normal(0, 1, aps[:, :2].shape) * s
            out["ap"][fname].append({"sigma_m": s, **run(dets, aps)})
        out["blackout"][fname] = []
        n = len(dets); start = n // 3
        for L in BLACKOUTS:
            d = [np.empty((0, 3)) if start <= f < start + L else x for f, x in enumerate(dets)]
            out["blackout"][fname].append({"frames": L, "frac": L / n, **run(d, sc.ap_positions)})
    return out


def main():
    js = [(s, k) for s in RUN_SCENES for k in SEEDS]
    with Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 10) as p:
        rows = list(p.imap_unordered(job, js))
    summ = {}
    for s in RUN_SCENES:
        g = [r for r in rows if r["scene"] == s]
        summ[s] = {"ate_dead_reckoning": float(np.mean([r["ate_dead_reckoning"] for r in g]))}
        for kind, key in (("ap", "sigma_m"), ("blackout", "frames")):
            for fr in ("oracle", "music"):
                tab = []
                for i in range(len(g[0][kind][fr])):
                    cells = [r[kind][fr][i] for r in g]
                    tab.append({key: cells[0][key], **{m: [float(np.nanmean([c[m] for c in cells])),
                                                         float(np.nanstd([c[m] for c in cells]))]
                                                     for m in ("ate", "map_accuracy", "iou")}})
                summ[s][f"{kind}_{fr}"] = tab
    json.dump({"summary": summ, "runs": rows}, open("results/robustness_corrected.json", "w"), indent=1)
    print(json.dumps(summ, indent=1))


if __name__ == "__main__":
    main()
