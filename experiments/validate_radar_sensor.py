"""Validate the Sionna radar sensor against a KNOWN geometry. Server-only (needs Sionna).

Three things cannot be checked without the simulator, and each would silently corrupt the
paper if it were wrong:

  1. Sionna's angle convention when TX and RX are CO-LOCATED, as they are in a monostatic
     radar (NVlabs/sionna-rt#5). If the bearing comes back mirrored, every reflector lands
     on the wrong side of the road and the A->B geometry ablation becomes uninterpretable.
  2. The array layouts of paths.tau / paths.a / paths.phi_r in Sionna RT 2.0.1.
  3. That diffuse scattering is genuinely required at 77 GHz monostatic. A specular-only
     wall is a mirror: it reflects away from the sensor, not back to it. Paper 2 measured
     1 return specular vs 8,417 diffuse for LiDAR -- confirm it reproduces for radar.

This is the GATE for sub-project 1. A radar that points the wrong way would produce a
beautifully self-consistent, completely wrong paper.

Run:
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/validate_radar_sensor.py
"""
from __future__ import annotations
import json
import os
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.geometry import RX_HEIGHT_M
from wifi_radar_slam.radar.config import RADAR_77G_4G
from wifi_radar_slam.radar.sensor import SionnaRadarSensor, paths_to_rays
from wifi_radar_slam.radar.processing import radar_scan

CFG_PATH = "configs/street_metal_oracle.yaml"


def main() -> None:
    cfg_run = load_config(CFG_PATH)
    built = build_scene(cfg_run)
    rng = np.random.default_rng(0)
    pose = built.trajectory[len(built.trajectory) // 2]      # mid-trajectory
    px, py = float(pose[0]), float(pose[1])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    print(f"scene   : {CFG_PATH}")
    print(f"pose    : x={px:.2f} y={py:.2f} yaw={np.rad2deg(yaw):+.1f} deg")

    sensor = SionnaRadarSensor(built, RADAR_77G_4G, rng)

    # --- (3) + (2): specular vs diffuse, and the raw array layouts --------------------
    import mitsuba as mi
    sensor.scene.transmitters["radar_tx"].position = mi.Point3f(px, py, RX_HEIGHT_M)
    sensor.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
    ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))

    counts = {}
    for diffuse in (False, True):
        paths = sensor.solver(sensor.scene, max_depth=3, samples_per_src=ns,
                              diffuse_reflection=diffuse, seed=1)
        tau_raw = np.asarray(paths.tau.numpy())
        phi_raw = np.asarray(paths.phi_r.numpy())
        # paths.a is a TUPLE (real, imag), not a tensor -- confirmed here, not assumed
        re_t, im_t = paths.a
        a_raw = np.asarray(re_t.numpy())
        valid_raw = np.asarray(paths.valid.numpy())
        # count only OUR transmitter's paths: the scene also carries the WiFi APs
        n_valid = int(np.count_nonzero(valid_raw[0, sensor.tidx]))
        counts[f"diffuse_{diffuse}"] = n_valid
        print(f"\n--- diffuse_reflection={diffuse} ---")
        print(f"  tau   shape {tau_raw.shape}   (n_rx, n_tx, n_paths)")
        print(f"  a     shape {a_raw.shape} x2  (tuple of real, imag)")
        print(f"  phi_r shape {phi_raw.shape}")
        print(f"  n_tx={tau_raw.shape[1]} (our radar_tx is index {sensor.tidx}; "
              f"the rest are the scene's WiFi APs)")
        print(f"  valid paths for radar_tx: {n_valid}")

        if diffuse:                       # what our _extract actually pulls out
            tau, a, phi = sensor._extract(paths)
            print(f"  _extract -> tau {tau.shape}, a {a.shape} ({a.dtype}), "
                  f"phi {phi.shape}")
            taus, amps, az = paths_to_rays(tau, a, phi, yaw)
            print(f"  after paths_to_rays: {len(taus)} rays")
            if len(taus):
                rr = taus * 299792458.0 / 2.0        # monostatic round-trip -> range
                print(f"  ray ranges: min {rr.min():.2f} m, median "
                      f"{np.median(rr):.2f} m, max {rr.max():.2f} m")

    if counts["diffuse_False"] >= counts["diffuse_True"]:
        print("\n!! WARNING: diffuse scattering did NOT increase the path count. "
              "Pitfall #2 does not reproduce -- investigate before trusting the sensor.")

    # --- (1): the geometry / angle-convention check -----------------------------------
    # The nearest ground-truth surface tells us where the strongest return SHOULD be.
    gt = built.ground_truth_map[:, :2]
    rel = gt - np.array([px, py])
    d = np.linalg.norm(rel, axis=1)
    near = int(np.argmin(d))
    true_range = float(d[near])
    tb = float(np.arctan2(rel[near, 1], rel[near, 0]) - yaw)
    true_bearing = float(np.arctan2(np.sin(tb), np.cos(tb)))
    print(f"\nnearest GT surface: range {true_range:.2f} m, "
          f"bearing {np.rad2deg(true_bearing):+.1f} deg")

    scan = sensor(pose)
    print(f"radar scan: {len(scan)} detections")
    if len(scan) == 0:
        raise SystemExit("FAIL: no detections at all. Check diffuse scattering (pitfall #2).")

    truth_xy = np.array([true_range * np.cos(true_bearing),
                         true_range * np.sin(true_bearing)])
    mirror_xy = np.array([true_range * np.cos(-true_bearing),
                          true_range * np.sin(-true_bearing)])
    err = np.linalg.norm(scan.points - truth_xy, axis=1)
    err_mirror = np.linalg.norm(scan.points - mirror_xy, axis=1)
    best = int(np.argmin(err))
    r = float(np.linalg.norm(scan.points[best]))
    b = float(np.arctan2(scan.points[best, 1], scan.points[best, 0]))
    print(f"closest detection : range {r:.2f} m, bearing {np.rad2deg(b):+.1f} deg")
    print(f"  error, assumed convention : {err.min():.2f} m")
    print(f"  error, MIRRORED convention: {err_mirror.min():.2f} m")

    mirrored = bool(err_mirror.min() < 0.5 * err.min())
    if mirrored:
        print("  *** ANGLE CONVENTION IS MIRRORED -- negate the azimuth in "
              "paths_to_rays. Do NOT compensate downstream. ***")

    out = {
        "scene": CFG_PATH,
        "pose": [px, py, yaw],
        "valid_paths_specular": counts["diffuse_False"],
        "valid_paths_diffuse": counts["diffuse_True"],
        "true_range_m": true_range,
        "true_bearing_deg": float(np.rad2deg(true_bearing)),
        "n_detections": int(len(scan)),
        "best_range_m": r,
        "best_bearing_deg": float(np.rad2deg(b)),
        "best_error_m": float(err.min()),
        "best_error_mirrored_m": float(err_mirror.min()),
        "angle_convention_mirrored": mirrored,
    }
    print("\n" + json.dumps(out, indent=2))
    os.makedirs("results", exist_ok=True)
    with open("results/radar_substrate_validation.json", "w") as f:
        json.dump(out, f, indent=2)
    print("saved -> results/radar_substrate_validation.json")

    # THE GATE
    if err.min() > 1.0 and not mirrored:
        raise SystemExit(
            f"FAIL (gate): no detection within 1 m of the true nearest surface "
            f"(best {err.min():.2f} m), and the mirror hypothesis does not explain it. "
            f"Diagnose before building anything on this sensor.")
    print("\nGATE PASSED" if not mirrored else "\nGATE: fix the mirrored convention, re-run.")


if __name__ == "__main__":
    main()
