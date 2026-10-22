#!/usr/bin/env python3
"""Stage 2 of the corrected evaluation -- every number the revised manuscript reports.

Fixes relative to the published pipeline (both documented in PROJECT.md):
  1. Odometry is NOISY (wheel-grade, ``odometry.OdometryNoise`` defaults) and dead reckoning
     is always reported; the published runs fed the filter the ground-truth velocity.
  2. CSI delays are EXCESS over the first arrival (Sionna ``normalize_delays``), converted
     with the known AP position; the published runs read them as absolute lengths.
  3. The localization back-end is the virtual-anchor FastSLAM (``slam.fastslam``); the
     published particle filter's update was self-referential.

All filter / noise parameters are fixed a priori (no tuning on these seeds):
  oracle  sigma_r 0.3 m, sigma_az 2 deg;   MUSIC sigma_r = 0.5 c/B, sigma_az 5 deg.

    .venv/bin/python experiments/run_corrected_eval.py   -> results/corrected_eval.json
"""
from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from make_music_detections import SCENES, MAIN_BW, SWEEP_BWS, N_SEEDS, N_SWEEP_SEEDS, load_dets  # noqa: E402

C = 299792458.0
MAP_MIN_SUPPORT = 5            # as the published configs (map_min_support: 5)


def _map_metrics(m, gt):
    from wifi_radar_slam.eval.metrics import chamfer, map_accuracy, map_completeness, occupancy_iou
    if len(m) == 0:
        return {"chamfer": float("nan"), "map_accuracy": float("nan"),
                "map_completeness": float("nan"), "iou": 0.0, "n_map": 0}
    return {"chamfer": chamfer(m, gt), "map_accuracy": map_accuracy(m, gt),
            "map_completeness": map_completeness(m, gt),
            "iou": occupancy_iou(m, gt, cell=1.0), "n_map": int(len(m))}


def gt_pose_map(sc, dets_abs3):
    """Triangulate ABSOLUTE 3-D lengths from the ground-truth poses; cluster as published."""
    from wifi_radar_slam.eval.scenes import horizontal
    from wifi_radar_slam.slam.particle_filter import _triangulate_bistatic, _cluster
    pts = []
    for p, d in zip(sc.poses, horizontal(dets_abs3, sc.ap_positions)):
        for r, a, ap in d:
            q = _triangulate_bistatic(p[:2], sc.ap_positions[int(ap)][:2], r, a)
            if q is not None:
                pts.append(q)
    m = _cluster(np.array(pts), radius=0.5, min_support=MAP_MIN_SUPPORT) if pts else np.empty((0, 2))
    return _map_metrics(m, sc.gt_map)


def slam_runs(sc, front, dets_excess, seed, bw, ambiguous=False):
    from wifi_radar_slam.geometry import velocity_from_poses
    from wifi_radar_slam.odometry import OdometryNoise, noisy_velocity, dead_reckon
    from wifi_radar_slam.slam.fastslam import run_fastslam, FastSlamParams
    from wifi_radar_slam.eval.metrics import ate, rpe
    gt = sc.poses
    v = noisy_velocity(velocity_from_poses(gt, sc.dt), sc.dt, OdometryNoise(),
                       np.random.default_rng(1000 + seed))
    par = (FastSlamParams(sigma_range=0.3, sigma_aoa_deg=2.0) if front == "oracle"
           else FastSlamParams(sigma_range=0.5 * C / (bw * 1e6), sigma_aoa_deg=5.0))
    dr = dead_reckon(v, sc.dt, gt[0])
    est, emap, info = run_fastslam(dets_excess, sc.ap_positions, v, sc.dt,
                                   np.random.default_rng(2000 + seed), init_pose=gt[0],
                                   params=par, excess_delay=True,
                                   ambiguous_azimuth=ambiguous)
    return {"ate_dead_reckoning": ate(dr, gt), "ate": ate(est, gt), "rpe": rpe(est, gt),
            "n_confirmed": info["n_confirmed"], **_map_metrics(emap, sc.gt_map)}, (v, dr)


def legacy_runs(sc, dets_excess, v, seed):
    """The PUBLISHED chain (excess read as absolute, legacy PF), with GT and noisy odometry."""
    from wifi_radar_slam.geometry import velocity_from_poses
    from wifi_radar_slam.slam.particle_filter import run_slam
    from wifi_radar_slam.eval.metrics import ate
    out = {}
    for tag, vel in (("gt_odometry", velocity_from_poses(sc.poses, sc.dt)), ("noisy_odometry", v)):
        est, emap = run_slam(dets_excess, sc.ap_positions, vel, sc.dt,
                             np.random.default_rng(3000 + seed), init_pose=sc.poses[0],
                             map_min_support=MAP_MIN_SUPPORT)
        out[tag] = {"ate": ate(est, sc.poses), **_map_metrics(emap, sc.gt_map)}
    return out


def job(args):
    scene, bw, seed = args
    from wifi_radar_slam.eval.scenes import SceneCache, excess_oracle_detections, excess_to_absolute
    from wifi_radar_slam.eval.phantom_decomp import decompose
    sc = SceneCache.load(SCENES[scene])
    music = load_dets(scene, bw, seed)
    res = {"scene": scene, "bw": bw, "seed": seed}
    music_abs = excess_to_absolute(music, sc.poses, sc.ap_positions)
    res["phantoms_corrected"] = decompose(sc, music_abs)
    res["gt_map_music_corrected"] = gt_pose_map(sc, music_abs)
    res["slam_music"], (v, _) = slam_runs(sc, "music", music, seed, bw)
    if bw == MAIN_BW:
        res["phantoms_as_published"] = decompose(sc, music)
        res["gt_map_music_as_published"] = gt_pose_map(sc, music)
        orc = excess_oracle_detections(sc)
        res["gt_map_oracle"] = gt_pose_map(sc, excess_to_absolute(orc, sc.poses, sc.ap_positions))
        res["slam_oracle"], _ = slam_runs(sc, "oracle", orc, seed, bw)
        res["legacy_music"] = legacy_runs(sc, music, v, seed)
        res["slam_music_two_branch"], _ = slam_runs(sc, "music", music, seed, bw, ambiguous=True)
    return res


def _agg(rows, path):
    vals = []
    for r in rows:
        x = r
        for k in path:
            x = x[k]
        vals.append(x)
    v = np.array(vals, float)
    return {"mean": float(np.nanmean(v)), "std": float(np.nanstd(v)), "n": int(np.isfinite(v).sum())}


def _walk(d, prefix=()):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _walk(v, prefix + (k,))
        elif isinstance(v, (int, float)) and k not in ("bw", "seed"):
            yield prefix + (k,)


def main():
    js = [(s, MAIN_BW, k) for s in SCENES for k in range(N_SEEDS)]
    js += [(s, bw, k) for s in ("controlled_wall", "street_canyon") for bw in SWEEP_BWS
           for k in range(N_SWEEP_SEEDS)]
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    with Pool(n) as p:
        rows = []
        for i, r in enumerate(p.imap_unordered(job, js)):
            rows.append(r); print(f"[{i + 1}/{len(js)}] {r['scene']} bw{r['bw']} s{r['seed']}", flush=True)
    summary = {}
    for scene in SCENES:
        for bw in (MAIN_BW,) + SWEEP_BWS:
            grp = [r for r in rows if r["scene"] == scene and r["bw"] == bw]
            if not grp:
                continue
            summary[f"{scene}/bw{bw}"] = {"/".join(p): _agg(grp, p) for p in _walk(grp[0])
                                          if p[0] != "scene"}
    os.makedirs("results", exist_ok=True)
    json.dump({"summary": summary, "runs": rows}, open("results/corrected_eval.json", "w"), indent=1)
    print("saved -> results/corrected_eval.json")


if __name__ == "__main__":
    main()
