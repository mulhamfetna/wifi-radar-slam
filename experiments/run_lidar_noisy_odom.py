#!/usr/bin/env python3
"""LiDAR baselines re-run with the SAME noisy odometry as the WiFi runs (paired seeds).

The published LiDAR numbers seeded ICP with the ground-truth velocity, like the WiFi filter.
Here each scene's scans are generated once (sensor noise seed 0) and ICP-SLAM is run with
``noisy_velocity(..., default_rng(1000 + seed))`` -- the exact draws used by
``run_corrected_eval.py`` -- plus a ground-truth-odometry row for the record.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/run_lidar_noisy_odom.py [geo|sionna]
      -> results/lidar_noisy_odom_<model>.json
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.odometry import OdometryNoise, noisy_velocity, dead_reckon
from wifi_radar_slam.eval.metrics import ate, chamfer, map_accuracy, map_completeness, occupancy_iou

SCENES = {"controlled_wall": "configs/controlled_music_joint.yaml",
          "street_canyon": "configs/street_metal_music.yaml",
          "street_canyon_scurve": "configs/street_metal_music_scurve.yaml"}
N_SEEDS = 10


def _metrics(est, emap, gt, gt_xy):
    return {"ate": ate(est, gt), "chamfer": chamfer(emap, gt_xy),
            "map_accuracy": map_accuracy(emap, gt_xy),
            "map_completeness": map_completeness(emap, gt_xy),
            "iou": occupancy_iou(emap, gt_xy, cell=1.0)}


def main(model: str):
    if model == "geo":
        from wifi_radar_slam.lidar.sensor_geo import geo_sensor as make
    else:
        from wifi_radar_slam.lidar.sensor_sionna import sionna_lidar_sensor as make
    out = {}
    for name, cfgp in SCENES.items():
        cfg = load_config(cfgp)
        built = build_scene(cfg)
        gt, dt, gt_xy = built.trajectory, cfg.trajectory.timestep_s, built.ground_truth_map[:, :2]
        sensor = make(built, OUSTER_OS1, np.random.default_rng(0))
        scans = [sensor(gt[f]) for f in range(len(gt))]
        vgt = velocity_from_poses(gt, dt)
        e, m = run_lidar_slam(scans, vgt, dt, np.random.default_rng(0), init_pose=gt[0])
        rows = {"gt_odometry": _metrics(e, m, gt, gt_xy), "noisy": []}
        for s in range(N_SEEDS):
            v = noisy_velocity(vgt, dt, OdometryNoise(), np.random.default_rng(1000 + s))
            e, m = run_lidar_slam(scans, v, dt, np.random.default_rng(s), init_pose=gt[0])
            r = _metrics(e, m, gt, gt_xy); r["ate_dead_reckoning"] = ate(dead_reckon(v, dt, gt[0]), gt)
            rows["noisy"].append(r)
        agg = {k: {"mean": float(np.mean([r[k] for r in rows["noisy"]])),
                   "std": float(np.std([r[k] for r in rows["noisy"]]))} for k in rows["noisy"][0]}
        rows["noisy_summary"] = agg
        out[name] = rows
        print(name, "GT-odom ATE", round(rows["gt_odometry"]["ate"], 3), "| noisy:",
              {k: f"{v['mean']:.3f}+-{v['std']:.3f}" for k, v in agg.items()}, flush=True)
    os.makedirs("results", exist_ok=True)
    json.dump(out, open(f"results/lidar_noisy_odom_{model}.json", "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "geo")
