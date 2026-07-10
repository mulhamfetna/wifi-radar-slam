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
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam

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
    vel = np.zeros((n, 2))
    vel[1:] = (gt[1:] - gt[:-1]) / DT      # GT-differenced velocity prior (as in sim)

    t_slam = time.time()

    def progress(f, nn, ns, nm):
        if f == 1 or f % 10 == 0 or f == nn - 1:
            el = time.time() - t_slam
            eta = el / f * (nn - f)
            log.info("frame %d/%d  scan=%d map=%d  elapsed=%.0fs  ETA=%.0fs",
                     f, nn, ns, nm, el, eta)

    est, _ = run_lidar_slam(scans, vel, DT, np.random.default_rng(0),
                            init_pose=(gt[0, 0], gt[0, 1], 0.0), progress=progress)
    ate = align_2d_ate(est, gt)
    log.info("SLAM done in %.1fs", time.time() - t_slam)
    out = {"sequence": SEQ, "frames": n, "aligned_ate_m": ate,
           "slam_seconds": round(time.time() - t_slam, 1)}
    log.info("[model C] KITTI seq %s: aligned ATE = %.3f m over %d frames", SEQ, ate, n)
    with open("data/kitti_results.json", "w") as fh:
        json.dump(out, fh, indent=2)
    log.info("saved -> data/kitti_results.json")


if __name__ == "__main__":
    main()
