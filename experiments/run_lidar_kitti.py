"""Model C: run our ICP SLAM on real KITTI seq-04 velodyne and report aligned ATE
vs GT poses — an external-validity anchor for the idealized models A/B.

Server-only (needs the fetched data). Throttled, multi-core (KD-tree ICP uses all
cores, scans load in parallel):
    nice -n 19 ionice -c3 python experiments/run_lidar_kitti.py
"""
import glob
import json
import logging
import os
import time
from multiprocessing import Pool

import numpy as np

from wifi_radar_slam.lidar.kitti import (load_velodyne_scan, load_gt_trajectory,
                                        align_2d_ate)
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam, _rigid_2d, _apply
from wifi_radar_slam.eval.metrics import rpe

SEQ = "04"
ROOT = "data/kitti"
DT = 0.1                      # KITTI velodyne is 10 Hz
MAX_FRAMES = 271
SCAN_VOXEL = 0.4             # downsample dense KITTI scans so ICP is tractable

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("kitti")


def _load(path):
    return load_velodyne_scan(path, voxel=SCAN_VOXEL)


def main() -> None:
    files = sorted(glob.glob(f"{ROOT}/sequences/{SEQ}/velodyne/*.bin"))[:MAX_FRAMES]
    log.info("loading %d scans on %d cores ...", len(files), os.cpu_count())
    t0 = time.time()
    with Pool(os.cpu_count()) as pool:
        scans = pool.map(_load, files)
    n = len(scans)
    log.info("loaded %d scans in %.1fs (mean %d pts/scan)", n, time.time() - t0,
             int(np.mean([len(s) for s in scans])))

    poses = open(f"{ROOT}/poses/{SEQ}.txt").read()
    calib = open(f"{ROOT}/sequences/{SEQ}/calib.txt").read()
    gt = load_gt_trajectory(poses, calib)[:n]
    # Adaptive constant-velocity motion model (velocity=None): frame-agnostic, correct
    # in the SLAM's own velodyne frame. (A GT-differenced prior would be in the camera
    # (x,z) frame, mismatched to the velodyne frame the SLAM runs in.)

    t_slam = time.time()

    def progress(f, nn, ns, nm):
        if f == 1 or f % 10 == 0 or f == nn - 1:
            el = time.time() - t_slam
            eta = el / f * (nn - f)
            log.info("frame %d/%d  scan=%d map=%d  elapsed=%.0fs  ETA=%.0fs",
                     f, nn, ns, nm, el, eta)

    est, _ = run_lidar_slam(scans, None, DT, np.random.default_rng(0),
                            init_pose=(0.0, 0.0, 0.0), progress=progress)
    log.info("SLAM done in %.1fs", time.time() - t_slam)

    # Global aligned ATE accumulates drift over the whole drive; RPE (per-frame,
    # drift-robust) is the scan-matching-accuracy metric. Align once, report both.
    ate = align_2d_ate(est, gt)
    ax, ay, ayaw = _rigid_2d(est[:, :2], gt)
    aligned = _apply(est[:, :2], ax, ay, ayaw)
    rpe_m = rpe(aligned, gt)
    path_len = float(np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1)))
    out = {"sequence": SEQ, "frames": n, "path_length_m": round(path_len, 1),
           "aligned_ate_m": ate, "rpe_m": rpe_m,
           "slam_seconds": round(time.time() - t_slam, 1)}
    log.info("[model C] KITTI seq %s: RPE = %.3f m/frame, aligned ATE = %.2f m "
             "over %d frames (%.0f m path)", SEQ, rpe_m, ate, n, path_len)
    with open("data/kitti_results.json", "w") as fh:
        json.dump(out, fh, indent=2)
    log.info("saved -> data/kitti_results.json")


if __name__ == "__main__":
    main()
