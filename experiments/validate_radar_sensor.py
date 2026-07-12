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
    # Compare each detection against ITS OWN nearest ground-truth surface. (An earlier
    # version compared everything to the single nearest GT point overall -- which sat at
    # -92 deg, i.e. OUTSIDE the radar's +/-90 deg field of view, so it was scoring the
    # sensor against a wall it physically cannot see.)
    gt_world = built.ground_truth_map[:, :2]
    scan = sensor(pose)
    print(f"\nradar scan: {len(scan)} detections")
    if len(scan) == 0:
        raise SystemExit("FAIL: no detections at all. Check diffuse scattering (pitfall #2).")

    def errors(points_local: np.ndarray) -> np.ndarray:
        """Distance from each detection (sensor-local) to the nearest GT surface."""
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        world = points_local @ R.T + np.array([px, py])       # local -> world
        return np.linalg.norm(world[:, None, :] - gt_world[None, :, :], axis=2).min(axis=1)

    err = errors(scan.points)
    # The mirror hypothesis: if Sionna's co-located-TX/RX convention flips azimuth, then
    # NEGATING every detection's bearing should fit the ground truth markedly BETTER.
    mirrored_pts = scan.points * np.array([1.0, -1.0])        # negate the bearing
    err_mirror = errors(mirrored_pts)

    print(f"  detection -> nearest real surface (assumed convention):")
    print(f"     min {err.min():.2f} m | median {np.median(err):.2f} m | "
          f"max {err.max():.2f} m")
    print(f"  same, with the bearing MIRRORED:")
    print(f"     min {err_mirror.min():.2f} m | median {np.median(err_mirror):.2f} m | "
          f"max {err_mirror.max():.2f} m")

    mirrored = bool(np.median(err_mirror) < 0.5 * np.median(err))
    if mirrored:
        print("  *** ANGLE CONVENTION IS MIRRORED -- negate the azimuth in paths_to_rays. "
              "Do NOT compensate downstream. ***")

    for i, (p, e) in enumerate(zip(scan.points, err)):
        r = float(np.hypot(*p))
        b = float(np.rad2deg(np.arctan2(p[1], p[0])))
        print(f"     det {i}: range {r:6.2f} m  bearing {b:+7.1f} deg  "
              f"-> nearest surface {e:5.2f} m")

    out = {
        "scene": CFG_PATH,
        "pose": [px, py, yaw],
        "valid_paths_specular": counts["diffuse_False"],
        "valid_paths_diffuse": counts["diffuse_True"],
        "n_detections": int(len(scan)),
        "err_to_nearest_surface_m": {
            "min": float(err.min()), "median": float(np.median(err)),
            "max": float(err.max()),
        },
        "err_mirrored_m": {
            "min": float(err_mirror.min()), "median": float(np.median(err_mirror)),
        },
        "angle_convention_mirrored": mirrored,
    }
    print("\n" + json.dumps(out, indent=2))
    os.makedirs("results", exist_ok=True)
    with open("results/radar_substrate_validation.json", "w") as f:
        json.dump(out, f, indent=2)
    print("saved -> results/radar_substrate_validation.json")

    # THE GATE: at least one detection must land on a real surface, and the convention
    # must not be mirrored. (A high MEDIAN error is not a failure here -- ghosts are
    # expected and are the paper's subject. What must be true is that the sensor CAN see
    # a real wall, in the right place.)
    if mirrored:
        raise SystemExit("GATE: angle convention is mirrored. Fix paths_to_rays, re-run.")
    if err.min() > 1.0:
        raise SystemExit(
            f"FAIL (gate): not one detection lands within 1 m of any real surface "
            f"(best {err.min():.2f} m). Diagnose before building on this sensor.")
    print(f"\nGATE PASSED: best detection is {err.min():.2f} m from a real surface; "
          f"angle convention confirmed (not mirrored).")


if __name__ == "__main__":
    main()
