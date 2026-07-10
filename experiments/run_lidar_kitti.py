"""Model C: run our ICP SLAM on real KITTI seq-04 velodyne and report aligned ATE
vs GT poses — an external-validity anchor for the idealized models A/B.

Server-only (needs the fetched data). Throttled:
    nice -n 19 ionice -c3 python experiments/run_lidar_kitti.py
"""
import glob
import json
import numpy as np
from wifi_radar_slam.lidar.kitti import (load_velodyne_scan, load_gt_trajectory,
                                        align_2d_ate)
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam

SEQ = "04"
ROOT = "data/kitti"
DT = 0.1                      # KITTI velodyne is 10 Hz
MAX_FRAMES = 271


def main() -> None:
    files = sorted(glob.glob(f"{ROOT}/sequences/{SEQ}/velodyne/*.bin"))[:MAX_FRAMES]
    scans = [load_velodyne_scan(f) for f in files]
    n = len(scans)
    poses = open(f"{ROOT}/poses/{SEQ}.txt").read()
    calib = open(f"{ROOT}/sequences/{SEQ}/calib.txt").read()
    gt = load_gt_trajectory(poses, calib)[:n]
    # velocity prior from GT (same recipe as the sim runs); final pose is ICP's output
    vel = np.zeros((n, 2))
    vel[1:] = (gt[1:] - gt[:-1]) / DT
    est, _ = run_lidar_slam(scans, vel, DT, np.random.default_rng(0),
                            init_pose=(gt[0, 0], gt[0, 1], 0.0))
    ate = align_2d_ate(est, gt)
    out = {"sequence": SEQ, "frames": n, "aligned_ate_m": ate}
    print(f"[model C] KITTI seq {SEQ}: aligned ATE = {ate:.3f} m over {n} frames")
    with open("data/kitti_results.json", "w") as fh:
        json.dump(out, fh, indent=2)
    print("saved -> data/kitti_results.json")


if __name__ == "__main__":
    main()
