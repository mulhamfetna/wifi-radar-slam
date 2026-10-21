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

TWO SOLVES PER FRAME, NOT FOUR. A Sionna solve returns the paths of EVERY transmitter at once,
and cells C and D differ only in BANDWIDTH -- which changes the signal chain, not the physics. So
one solve at 5.2 GHz serves cells A and B, and one at 77 GHz serves C and D. Ray tracing
dominates the runtime, so this halves it, and every cell still sees exactly the rays it would
have seen on its own.

    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 \\
        .venv/bin/python experiments/run_ablation.py [scene_name]
"""
from __future__ import annotations
import json
import logging
import os
import sys
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

# Take every FRAME_STRIDE-th frame. Consecutive frames are 0.25 m apart (5 m/s at a 50 ms step),
# so they are highly redundant for the MAP and DETECTION statistics -- which is all we measure,
# since with ground-truth poses there is no trajectory to estimate. Stated, not silent.
#
# Set per scene via WRS_FRAME_STRIDE so both scenes contribute the SAME number of frames:
# controlled_wall has a 120-frame trajectory (stride 2 -> 60 frames, 0.5 m apart) and
# street_canyon a 240-frame one (stride 4 -> 60 frames, 1.0 m apart). The street canyon carries
# far more geometry, so it costs ~6.6x more per frame to ray-trace; equalising the frame count
# keeps the map statistics comparable AND the run tractable.
FRAME_STRIDE = int(os.environ.get("WRS_FRAME_STRIDE", "2"))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("ablation")


def _voxel_map(points: np.ndarray, voxel: float = MAP_VOXEL) -> np.ndarray:
    cells: dict[tuple[int, int], np.ndarray] = {}
    for p in points:
        cells.setdefault((int(round(p[0] / voxel)), int(round(p[1] / voxel))), p)
    return np.array(list(cells.values())) if cells else np.empty((0, 2))


def _map_metrics(est_map: np.ndarray, gt_xy: np.ndarray) -> dict:
    """The four map metrics.

    ATE/RPE are absent BY DESIGN: the poses are ground truth, so there is no trajectory error to
    report -- for any cell. That is the entire point of scoring this way.
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


def _summarise(cell, poses, maps, det_r, det_a, true_r, true_a, gt_xy) -> dict:
    """Phantom stats (BOTH tolerances) + map metrics, for one cell."""
    est_map = (map_under_gt_poses(maps, poses, voxel=MAP_VOXEL) if cell.monostatic
               else _voxel_map(np.concatenate([w for w in maps if w.size])
                               if any(w.size for w in maps) else np.empty((0, 2))))

    # Matched FRAME BY FRAME. A detection made at frame f must be explained by a path that
    # existed at frame f's vehicle position; pooling every frame's paths into one haystack would
    # let a detection be "explained" by a path from somewhere else entirely on the trajectory,
    # which massively undercounts phantoms.
    res = cell.config.range_resolution_m
    return {
        "cell": cell.key, "label": cell.label,
        "carrier_ghz": cell.config.carrier_hz / 1e9,
        "bandwidth_mhz": cell.config.bandwidth_hz / 1e6,
        "monostatic": cell.monostatic,
        "isolates": cell.isolates,
        "range_resolution_m": res,
        "n_frames": int(len(poses)),
        "n_true_paths": int(sum(t.size for t in true_r)),
        "phantom_fixed_3m": phantom_stats_frames(det_r, det_a, true_r, true_a,
                                                 range_scale_m=3.0),
        "phantom_resolution_scaled": phantom_stats_frames(det_r, det_a, true_r, true_a,
                                                          range_scale_m=3.0 * res),
        "map": _map_metrics(est_map, gt_xy),
    }


def run_all_cells(cfg, seed: int) -> list[dict]:
    """All four cells, for one scene and one seed, with only TWO ray-trace solves per frame.

    A Sionna solve returns every transmitter's paths at once, and cells C and D differ only in
    bandwidth -- a property of the signal chain, not of the physics. So:

        solve at 5.2 GHz  ->  cell A (bistatic; the AP transmitters) and cell B (monostatic)
        solve at 77 GHz   ->  cells C and D (the same rays; different bandwidth downstream)

    Ray tracing dominates the runtime, so this halves it -- and every cell still sees exactly the
    rays it would have seen had it run alone.
    """
    built52 = build_scene(cfg)          # its sensors retune it to 5.2 GHz
    built77 = build_scene(cfg)          # its sensors retune it to 77 GHz
    traj = built52.trajectory
    poses = traj[list(range(0, len(traj), FRAME_STRIDE))]
    gt_xy = built52.ground_truth_map[:, :2]

    rng = np.random.default_rng(seed)
    sB = SionnaRadarSensor(built52, CELLS["B"].config, rng)      # adds radar_tx; retunes to 5.2
    sA = SionnaBistaticSensor(built52, CELLS["A"].config, rng)   # same scene, same carrier
    sC = SionnaRadarSensor(built77, CELLS["C"].config, rng)      # adds radar_tx; retunes to 77
    sD = SionnaRadarSensor(built77, CELLS["D"].config, rng)      # same scene, same carrier

    maps = {"A": [], "B": [], "C": [], "D": []}
    det = {k: ([], []) for k in "ABCD"}                          # per-frame (ranges, azimuths)
    tru = {k: ([], []) for k in "ABCD"}

    def _polar(scan, yaw):
        if len(scan) == 0:
            return np.empty(0), np.empty(0)
        r = np.linalg.norm(scan.points, axis=1)
        a = np.arctan2(scan.points[:, 1], scan.points[:, 0]) + yaw   # -> WORLD azimuth
        return r, np.angle(np.exp(1j * a))

    t0 = time.time()
    for i, pose in enumerate(poses):
        yaw = float(pose[2]) if len(pose) > 2 else 0.0

        # ---- ONE solve at 5.2 GHz: serves cells A and B --------------------------------
        p52 = sB._solve(pose)

        s = sB.detect(p52, pose)
        maps["B"].append(s)
        r, a = _polar(s, yaw)
        det["B"][0].append(r)
        det["B"][1].append(a)
        tB = true_paths_for_tx(p52, sB.tidx, yaw, sB.floor_ids, monostatic=True)
        tru["B"][0].append(tB["range_m"])
        tru["B"][1].append(tB["azimuth_world_rad"])

        w, rA, aA, _ = sA.detect(p52, pose)
        maps["A"].append(w)
        det["A"][0].append(np.asarray(rA).ravel())
        det["A"][1].append(np.angle(np.exp(1j * np.asarray(aA).ravel())))
        trs, tas = [], []
        for t in sA.ap_idx:                     # pool EVERY illuminating AP's true paths
            one = true_paths_for_tx(p52, t, yaw, sA.floor_ids, monostatic=False)
            trs.append(one["range_m"])
            tas.append(one["azimuth_world_rad"])
        tru["A"][0].append(np.concatenate(trs) if trs else np.empty(0))
        tru["A"][1].append(np.concatenate(tas) if tas else np.empty(0))

        # ---- ONE solve at 77 GHz: serves cells C and D ---------------------------------
        p77 = sC._solve(pose)
        tCD = true_paths_for_tx(p77, sC.tidx, yaw, sC.floor_ids, monostatic=True)
        for key, sensor in (("C", sC), ("D", sD)):
            s = sensor.detect(p77, pose)
            maps[key].append(s)
            r, a = _polar(s, yaw)
            det[key][0].append(r)
            det[key][1].append(a)
            tru[key][0].append(tCD["range_m"])
            tru[key][1].append(tCD["azimuth_world_rad"])

        if i % 10 == 0 or i == len(poses) - 1:
            el = time.time() - t0
            eta = el / (i + 1) * (len(poses) - i - 1)
            log.info("    frame %3d/%d  elapsed %.0fs  ETA %.0fs", i, len(poses), el, eta)

    out = []
    for key in ("A", "B", "C", "D"):
        r = _summarise(CELLS[key], poses, maps[key], det[key][0], det[key][1],
                       tru[key][0], tru[key][1], gt_xy)
        r["seed"] = seed
        out.append(r)
        log.info("  -> cell %s seed %d: phantom(fixed)=%.1f%% (res-scaled %.1f%%)  "
                 "n_det=%d  IoU=%.3f  map=%d pts",
                 key, seed,
                 100 * r["phantom_fixed_3m"]["phantom_rate"],
                 100 * r["phantom_resolution_scaled"]["phantom_rate"],
                 r["phantom_fixed_3m"]["n_detections"],
                 r["map"]["iou"], r["map"]["n_map_points"])
    return out


def run_music_reference(cfg, seed: int) -> dict:
    """The 5th row: WiFi + joint 2-D MUSIC -- papers 1-2's front-end, on the SAME GT poses.

    Kept OUT of the cell chain on purpose. This row is what makes the superresolution-vs-FFT axis
    VISIBLE: cells A and M share the same physics (bistatic 5.2 GHz WiFi) and the same
    ground-truth poses, and differ ONLY in the front-end. Any gap between them is therefore the
    front-end, with nothing else left to blame -- which is exactly the confound the spec insisted
    we must not bury.
    """
    built = build_scene(cfg)
    rng = np.random.default_rng(seed)
    traj = built.trajectory
    idx = list(range(0, len(traj), FRAME_STRIDE))
    gt_xy = built.ground_truth_map[:, :2]

    csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
    dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa, joint=True)

    pts = []
    n_det = 0
    for f in idx:
        D = np.asarray(dets[f]).reshape(-1, 3)
        n_det += int(D.shape[0])
        for (pl, aoa, ap_i) in D:
            ap_xy = np.asarray(built.ap_positions[int(ap_i)])[:2]
            R = _triangulate_bistatic(traj[f][:2], ap_xy, float(pl), float(aoa))
            if R is not None:
                pts.append(R)
    est_map = _voxel_map(np.array(pts) if pts else np.empty((0, 2)))
    return {"cell": "M", "label": "WiFi + joint 2-D MUSIC (papers 1-2 front-end)",
            "seed": seed, "n_frames": len(idx), "n_detections": n_det,
            "map": _map_metrics(est_map, gt_xy)}


def main() -> None:
    only = sys.argv[1] if len(sys.argv) > 1 else None
    scenes = {only: SCENES[only]} if only else SCENES
    suffix = f"_{only}" if only else ""

    results = []
    for scene, cfgp in scenes.items():
        cfg = load_config(cfgp)
        log.info("=== scene %s (%s) ===", scene, cfgp)
        for seed in SEEDS:
            log.info("  seed %d: four cells, TWO solves per frame ...", seed)
            for r in run_all_cells(cfg, seed):
                r["scene"] = scene
                results.append(r)
            m = run_music_reference(cfg, seed)
            m["scene"] = scene
            results.append(m)
            log.info("  -> MUSIC ref seed %d: n_det=%d  IoU=%.3f  map=%d pts",
                     seed, m["n_detections"], m["map"]["iou"], m["map"]["n_map_points"])

            os.makedirs("results", exist_ok=True)
            with open(f"results/ablation{suffix}.json", "w") as f:   # checkpoint every seed
                json.dump(results, f, indent=2)
            log.info("  [checkpointed %d rows -> results/ablation%s.json]",
                     len(results), suffix)

    log.info("DONE -> results/ablation%s.json  (%d rows)", suffix, len(results))


if __name__ == "__main__":
    main()
