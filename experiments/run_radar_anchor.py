"""THE CREDIBILITY GATE. Run our SHARED scan-to-map ICP back-end on REAL radar (Boreas) and
report KITTI-protocol drift beside the cited SOTA.

Why this exists: radar odometry is a mature field. If our back-end produces a laughable drift
number on real radar, then in paper 3's ablation radar would be an artificially weak baseline and
WiFi would look artificially good -- and the whole paper would be built on sand. The verdict
thresholds are fixed IN ADVANCE (see VERDICT below) precisely so they cannot be tuned after
seeing the number.

The back-end (lidar/slam_icp.run_lidar_slam) is used COMPLETELY UNCHANGED. That is the entire
argument: any difference between sensors must be attributable to the sensor, not the estimator.

    nice -n 19 ionice -c3 .venv/bin/python experiments/run_radar_anchor.py
"""
from __future__ import annotations
import concurrent.futures as cf
import glob
import json
import logging
import os
import time

import numpy as np

from wifi_radar_slam.radar import boreas
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam, _rigid_2d, _apply
from wifi_radar_slam.eval.drift import drift, path_lengths
from wifi_radar_slam.eval.metrics import rpe

SEQ = "boreas-2020-11-26-13-58"
ROOT = f"data/boreas/{SEQ}"
# Front-end, configured to radar's PHYSICS -- see docs/results-paper3-anchor.md. Radar noise is
# ANISOTROPIC: range is accurate (0.06 m) but cross-range grows with range (a 0.9 deg beam gives
# +/-1.6 m of tangential error at 100 m). Yaw is estimated FROM tangential displacement -- exactly
# the noisy direction -- and point-to-point ICP weights every direction equally. So we (a) take
# many more returns per azimuth, which averages the noise down, and (b) crop range, where the
# cross-range error is worst. Measured effect on per-frame yaw error: k=12/100 m gives 5.35 deg
# std; k=40/50 m gives 0.46 deg -- an 11x reduction, and the difference between a radar baseline
# that tracks and one that does not.
K = 40                    # k-strongest returns per azimuth (CFEAR-class front-end)
MAX_RANGE_M = 50.0
MAP_VOXEL = 0.5           # accumulated-map voxel
DT = 0.25                 # Navtech is 4 Hz

# Cited SOTA -- NOT reimplemented. The caveats are part of the citation, not footnotes.
SOTA = {
    "CFEAR (T-RO 2023, Oxford, radar-only, TUNED)": 1.09,
    "CFEAR (T-RO 2023, Oxford, radar-only, untuned)": 1.16,
    "DRO (arXiv 2504.20339, Boreas) -- GYRO-AIDED + direct-intensity, NOT comparable": 0.26,
}

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("anchor")


def check_yaw_convention(poses: np.ndarray) -> float:
    """Verify Boreas's `heading` against the actual direction of travel. Returns mean |error|.

    NEVER ASSUME A CONVENTION -- sub-project 1 taught this three times over. Applanix `heading`
    could plausibly be counter-clockwise from east (the maths convention) or clockwise from
    north (the survey convention), and picking wrong would rotate every scan into the map at the
    wrong angle, producing a drift number that says nothing whatsoever about our back-end.

    So: compare the reported heading against atan2(dy, dx) over the GT positions, while the
    vehicle is actually moving.
    """
    d = np.diff(poses[:, :2], axis=0)
    speed = np.linalg.norm(d, axis=1)
    if not (speed > 0).any():
        return float("nan")
    moving = speed > 0.5 * np.median(speed[speed > 0])
    if moving.sum() < 10:
        log.warning("too few moving frames to check the yaw convention")
        return float("nan")
    course = np.arctan2(d[moving, 1], d[moving, 0])
    reported = poses[:-1, 2][moving]
    return float(np.mean(np.abs(np.angle(np.exp(1j * (course - reported))))))


def main() -> None:
    files = sorted(glob.glob(f"{ROOT}/radar/*.png"))
    if not files:
        raise SystemExit(f"no scans under {ROOT}/radar -- run experiments/fetch_boreas.py first")
    log.info("found %d radar scans", len(files))

    ts_gt, poses_gt = boreas.load_gt_poses(open(f"{ROOT}/applanix/radar_poses.csv").read())
    by_ts = {int(t): p for t, p in zip(ts_gt, poses_gt)}

    # Join scans to poses by FILENAME -- GPSTime is exactly the PNG's name.
    keep, gt = [], []
    for f in files:
        t = int(os.path.splitext(os.path.basename(f))[0])
        if t in by_ts:
            keep.append(f)
            gt.append(by_ts[t])
    gt = np.array(gt)
    log.info("matched %d/%d scans to GT poses", len(keep), len(files))
    if len(keep) < 200:
        raise SystemExit("too few matched frames to measure 100 m sub-sequences")

    # --- the yaw convention, verified rather than assumed -----------------------------
    yaw_err = check_yaw_convention(gt)
    log.info("yaw convention: mean |heading - course of travel| = %.1f deg",
             np.rad2deg(yaw_err))
    if not (yaw_err < np.deg2rad(20)):
        log.warning("!! `heading` does NOT match the direction of travel -- the convention is "
                    "not what we assumed. Fix boreas.load_gt_poses before trusting any drift "
                    "number below.")

    total_m = float(path_lengths(gt)[-1])
    log.info("trajectory: %.0f m over %d frames (%.0f s)", total_m, len(gt), len(gt) * DT)
    if total_m < 200.0:
        raise SystemExit(f"trajectory is only {total_m:.0f} m -- too short for KITTI drift")

    # --- load scans in parallel, motion-compensated using the GT speed -----------------
    # Motion compensation needs a velocity. We use the GT velocity: this experiment measures
    # the BACK-END, not a velocity estimator, and CFEAR-class methods likewise rely on a motion
    # estimate. Stated, not hidden.
    vel = np.zeros((len(gt), 2))
    vel[1:] = np.diff(gt[:, :2], axis=0) / DT
    vel[0] = vel[1]
    c, s = np.cos(-gt[:, 2]), np.sin(-gt[:, 2])          # world velocity -> sensor-local
    vel_local = np.stack([c * vel[:, 0] - s * vel[:, 1],
                          s * vel[:, 0] + c * vel[:, 1]], axis=1)

    def load(i):
        return boreas.load_radar_scan(keep[i], k=K, max_range_m=MAX_RANGE_M,
                                      velocity_xy=tuple(vel_local[i]))

    log.info("loading %d scans on %d cores ...", len(keep), os.cpu_count())
    t0 = time.time()
    with cf.ThreadPoolExecutor(max_workers=max(os.cpu_count() - 2, 1)) as pool:
        scans = list(pool.map(load, range(len(keep))))
    log.info("loaded in %.1f s; mean %.0f points/scan",
             time.time() - t0, np.mean([len(s) for s in scans]))

    # --- the SHARED back-end, UNCHANGED -----------------------------------------------
    t0 = time.time()

    def progress(f, n, npts, ncells):
        if f % 50 == 0 or f == n - 1:
            el = time.time() - t0
            eta = el / max(f, 1) * (n - f)
            log.info("  ICP %5d/%d  scan=%5d pts  map=%7d cells  elapsed %.0fs  ETA %.0fs",
                     f, n, npts, ncells, el, eta)

    # velocity=None -> the frame-agnostic adaptive constant-velocity motion model, correct in
    # the SLAM's own frame even though GT lives in a different one (the choice that fixed the
    # KITTI run in paper 2).
    est, est_map = run_lidar_slam(scans, None, DT, np.random.default_rng(0),
                                  voxel=MAP_VOXEL, progress=progress)
    log.info("SLAM done in %.0f s", time.time() - t0)

    # --- score: KITTI drift at the STANDARD lengths (valid here -- km-scale) ------------
    d = drift(est, gt)
    r = rpe(est, gt)
    R = _rigid_2d(est[:, :2], gt[:, :2])              # aligned ATE, for context only
    ate = float(np.sqrt(np.mean(np.sum(
        (_apply(est[:, :2], *R) - gt[:, :2]) ** 2, axis=1))))

    print("\n" + "=" * 76)
    print("RADAR CREDIBILITY ANCHOR -- Boreas, our SHARED scan-to-map ICP back-end (unchanged)")
    print("=" * 76)
    print(f"sequence            : {SEQ}")
    print(f"frames              : {len(gt)}    trajectory: {total_m:.0f} m")
    print(f"front-end           : k-strongest, k={K}, range <= {MAX_RANGE_M:.0f} m")
    print(f"motion compensation : ON (the sweep is ~249 ms)")
    print(f"yaw convention      : mean err {np.rad2deg(yaw_err):.1f} deg vs course of travel")
    print()
    print(f"OUR drift           : {d['trans_pct']:.2f} % trans, "
          f"{d['rot_deg_per_100m']:.2f} deg/100m   ({d['n_segments']} segments)")
    print(f"OUR RPE             : {r:.3f} m/frame")
    print(f"OUR aligned ATE     : {ate:.1f} m")
    print()
    print("cited SOTA (NOT reimplemented):")
    for name, v in SOTA.items():
        print(f"  {v:5.2f} %  {name}")
    if d["per_length"]:
        print("\nper sub-sequence length:")
        for L, (t, rr) in sorted(d["per_length"].items()):
            print(f"  {L:3d} m : {t:6.2f} % trans, {rr:5.2f} deg/100m")

    # --- THE VERDICT, thresholds fixed in advance --------------------------------------
    t = d["trans_pct"]
    if not np.isfinite(t):
        verdict = "FAIL (no drift computed -- trajectory too short, or SLAM diverged)"
    elif t < 5.0:
        verdict = "PASS -- the baseline is credible. Proceed to sub-project 3."
    elif t < 10.0:
        verdict = ("MARGINAL -- report it and BOUND every claim: our back-end is not "
                   "CFEAR-class, so radar's true capability is UNDERSTATED here and the "
                   "WiFi-vs-radar gap we measure is a LOWER BOUND.")
    else:
        verdict = ("FAIL -- the radar baseline is a strawman. STOP: the ablation would be built "
                   "on sand. Fix the back-end or reconsider the paper.")
    print("\n" + "=" * 76)
    print(f"VERDICT: {verdict}")
    print("=" * 76)

    os.makedirs("results", exist_ok=True)
    out = {
        "sequence": SEQ, "n_frames": len(gt), "trajectory_m": total_m,
        "k": K, "max_range_m": MAX_RANGE_M, "motion_compensated": True,
        "map_voxel_m": MAP_VOXEL,
        "mean_points_per_scan": float(np.mean([len(s) for s in scans])),
        "yaw_convention_mean_err_deg": float(np.rad2deg(yaw_err)),
        "drift": d, "rpe_m": r, "aligned_ate_m": ate,
        "sota_cited": SOTA, "verdict": verdict,
    }
    with open("results/radar_anchor.json", "w") as f:
        json.dump(out, f, indent=2)
    print("saved -> results/radar_anchor.json")


if __name__ == "__main__":
    main()
