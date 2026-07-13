"""THE ABLATION -- paper 3's experiment. Server-only (needs Sionna).

Five rows, ONE detection chain, scored UNDER GROUND-TRUTH POSES -- no estimator in the loop for
anyone. (Why that is a STRONGER experiment here rather than a weaker one:
docs/results-paper3-anchor.md. Our shared back-end provably cannot estimate rotation from radar,
so a SLAM comparison would have handed radar a crippled trajectory and flattered WiFi.)

    A  WiFi bistatic    5.2 GHz  160 MHz   ambient APs illuminate -> ELLIPSE solve
    B  WiFi monostatic  5.2 GHz  160 MHz   own TX                 -> isolates GEOMETRY (A->B)
    C  Radar narrow     77 GHz   160 MHz   own TX                 -> isolates CARRIER  (B->C)
    D  Radar full       77 GHz   4 GHz     own TX                 -> isolates BANDWIDTH (C->D)
    M  WiFi + joint 2-D MUSIC (papers 1-2's front-end) -- the reference row, kept OUT of the
       chain so the superresolution-vs-FFT axis stays visible instead of confounded

RQ1 (the headline) is the PHANTOM RATE, reported at BOTH a fixed tolerance (comparable to paper
2's ~89 %) and a resolution-scaled one -- because a fixed absolute window would hand radar a
lower phantom rate BY CONSTRUCTION (cell D resolves to 0.0375 m, cell A to 0.94 m).

    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 .venv/bin/python experiments/run_ablation.py
"""
from __future__ import annotations
import json
import logging
import os
import time

import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.slam.particle_filter import _triangulate_bistatic
from wifi_radar_slam.radar.cells import CELLS
from wifi_radar_slam.radar.sensor import SionnaRadarSensor
from wifi_radar_slam.radar.sensor_bistatic import SionnaBistaticSensor
from wifi_radar_slam.radar.truth import true_paths_for_tx
from wifi_radar_slam.eval.phantom import phantom_stats_frames
from wifi_radar_slam.eval.mapping import map_under_gt_poses
from wifi_radar_slam.eval.metrics import (chamfer, map_accuracy, map_completeness,
                                          occupancy_iou)

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}
SEEDS = [0, 1, 2]
MAP_VOXEL = 0.5

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("ablation")


def _voxel_map(points: np.ndarray, voxel: float = MAP_VOXEL) -> np.ndarray:
    cells: dict[tuple[int, int], np.ndarray] = {}
    for p in points:
        cells.setdefault((int(round(p[0] / voxel)), int(round(p[1] / voxel))), p)
    return np.array(list(cells.values())) if cells else np.empty((0, 2))


def _map_metrics(est_map: np.ndarray, gt_xy: np.ndarray) -> dict:
    """The four map metrics.

    ATE/RPE are absent BY DESIGN: poses are ground truth, so there is no trajectory error to
    report -- for any cell. That is the point of scoring this way.
    """
    if est_map.size == 0:
        return {"chamfer": float("inf"), "map_accuracy": float("inf"),
                "map_completeness": float("inf"), "iou": 0.0, "n_map_points": 0}
    return {
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
        "n_map_points": int(est_map.shape[0]),
    }


def run_cell(cell, built, seed: int) -> dict:
    """One cell, one scene, one seed -> phantom stats (BOTH tolerances) + map metrics."""
    rng = np.random.default_rng(seed)
    traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]

    sensor = (SionnaRadarSensor(built, cell.config, rng) if cell.monostatic
              else SionnaBistaticSensor(built, cell.config, rng))

    scans, world_pts = [], []
    det_r, det_a, true_r, true_a = [], [], [], []
    t0 = time.time()
    for f in range(len(traj)):
        pose = traj[f]
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        paths = sensor._solve(pose)                 # ray-trace ONCE per frame

        if cell.monostatic:
            scan = sensor.detect(paths, pose)
            scans.append(scan)
            if len(scan):
                # the chain's detections, in the SAME quantity the true paths use:
                # round-trip range (m) and WORLD azimuth
                r = np.linalg.norm(scan.points, axis=1)
                a = np.arctan2(scan.points[:, 1], scan.points[:, 0]) + yaw
            else:
                r = a = np.empty(0)
            tp = true_paths_for_tx(paths, sensor.tidx, yaw, sensor.floor_ids, monostatic=True)
            true_r.append(tp["range_m"])
            true_a.append(tp["azimuth_world_rad"])
        else:
            world, r, a, _ = sensor.detect(paths, pose)
            world_pts.append(world)
            # pool the true paths of EVERY illuminating AP -- cell A pools their detections too
            tr, ta = [], []
            for t in sensor.ap_idx:
                one = true_paths_for_tx(paths, t, yaw, sensor.floor_ids, monostatic=False)
                tr.append(one["range_m"])
                ta.append(one["azimuth_world_rad"])
            true_r.append(np.concatenate(tr) if tr else np.empty(0))
            true_a.append(np.concatenate(ta) if ta else np.empty(0))

        det_r.append(np.asarray(r).ravel())
        det_a.append(np.angle(np.exp(1j * np.asarray(a).ravel())))

        if f % 100 == 0 or f == len(traj) - 1:
            el = time.time() - t0
            eta = el / (f + 1) * (len(traj) - f - 1)
            log.info("    cell %s frame %4d/%d  elapsed %.0fs  ETA %.0fs",
                     cell.key, f, len(traj), el, eta)

    est_map = (map_under_gt_poses(scans, traj, voxel=MAP_VOXEL) if cell.monostatic
               else _voxel_map(np.concatenate([w for w in world_pts if w.size])
                               if any(w.size for w in world_pts) else np.empty((0, 2))))

    # --- RQ1: the phantom rate, at BOTH tolerances -------------------------------------
    # Matched FRAME BY FRAME. A detection made at frame 5 must be explained by a path that
    # existed at frame 5's vehicle position; pooling every frame's paths into one haystack
    # would let a detection be "explained" by a path from somewhere else entirely on the
    # trajectory, which massively undercounts phantoms.
    res = cell.config.range_resolution_m
    fixed = phantom_stats_frames(det_r, det_a, true_r, true_a, range_scale_m=3.0)
    scaled = phantom_stats_frames(det_r, det_a, true_r, true_a, range_scale_m=3.0 * res)

    return {
        "cell": cell.key, "label": cell.label, "seed": seed,
        "carrier_ghz": cell.config.carrier_hz / 1e9,
        "bandwidth_mhz": cell.config.bandwidth_hz / 1e6,
        "monostatic": cell.monostatic,
        "isolates": cell.isolates,
        "range_resolution_m": res,
        "n_true_paths": int(sum(t.size for t in true_r)),
        "phantom_fixed_3m": fixed,
        "phantom_resolution_scaled": scaled,
        "map": _map_metrics(est_map, gt_xy),
    }


def run_music_reference(built, cfg, seed: int) -> dict:
    """The 5th row: WiFi + joint 2-D MUSIC -- papers 1-2's front-end, same GT poses.

    Kept OUT of the cell chain on purpose, so the superresolution-vs-FFT axis is VISIBLE rather
    than silently confounded with the physics the ablation isolates.
    """
    rng = np.random.default_rng(seed)
    traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
    dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa, joint=True)

    pts = []
    for f in range(len(traj)):
        for (pl, aoa, ap_i) in np.asarray(dets[f]).reshape(-1, 3):
            ap_xy = np.asarray(built.ap_positions[int(ap_i)])[:2]
            R = _triangulate_bistatic(traj[f][:2], ap_xy, float(pl), float(aoa))
            if R is not None:
                pts.append(R)
    est_map = _voxel_map(np.array(pts) if pts else np.empty((0, 2)))
    return {"cell": "M", "label": "WiFi + joint 2-D MUSIC (papers 1-2 front-end)",
            "seed": seed, "map": _map_metrics(est_map, gt_xy)}


def main() -> None:
    import sys
    # Optionally restrict to one scene, so the two scenes can run as parallel processes:
    #     python experiments/run_ablation.py street_canyon
    only = sys.argv[1] if len(sys.argv) > 1 else None
    scenes = {only: SCENES[only]} if only else SCENES
    suffix = f"_{only}" if only else ""

    results = []
    for scene, cfgp in scenes.items():
        cfg = load_config(cfgp)
        log.info("=== scene %s (%s) ===", scene, cfgp)
        for seed in SEEDS:
            for key in ("A", "B", "C", "D"):
                built = build_scene(cfg)            # rebuild: the sensors mutate the scene
                log.info("  cell %s, seed %d ...", key, seed)
                r = run_cell(CELLS[key], built, seed)
                r["scene"] = scene
                results.append(r)
                log.info("  -> cell %s seed %d: phantom(fixed)=%.1f%% "
                         "phantom(res-scaled)=%.1f%% IoU=%.3f  map=%d pts",
                         key, seed,
                         100 * r["phantom_fixed_3m"]["phantom_rate"],
                         100 * r["phantom_resolution_scaled"]["phantom_rate"],
                         r["map"]["iou"], r["map"]["n_map_points"])
            built = build_scene(cfg)
            m = run_music_reference(built, cfg, seed)
            m["scene"] = scene
            results.append(m)
            log.info("  -> MUSIC ref seed %d: IoU=%.3f", seed, m["map"]["iou"])

    os.makedirs("results", exist_ok=True)
    out = f"results/ablation{suffix}.json"
    with open(out, "w") as f:
        json.dump(results, f, indent=2)
    log.info("saved -> %s  (%d rows)", out, len(results))


if __name__ == "__main__":
    main()
