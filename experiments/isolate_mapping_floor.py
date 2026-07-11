"""Isolate DISCRIMINATION vs ESTIMATION as the WiFi mapping floor.

For every MUSIC detection, match it to the nearest TRUE Sionna path (same AP) in
(range, azimuth) space, then classify that true path (facade single-bounce vs not).
For MUSIC detections that DID find a true facade path, triangulate TWICE:
  - with the TRUE delay/AoA of that path   -> discrimination held perfect
  - with the MUSIC-ESTIMATED delay/AoA     -> adds estimation error only
The gap between the two is PURE estimation error.
"""
import os
import numpy as np
import sionna.rt as rt
import mitsuba as mi
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses, RX_HEIGHT_M
from wifi_radar_slam.slam.particle_filter import run_slam, _triangulate_bistatic
C = 299792458.0


def true_paths(built, cfg, rng, max_depth=3):
    """Per-frame list of (n_ap, n_path) arrays: range, phi, is_facade, valid."""
    ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
    scene = built.scene
    solver = rt.PathSolver()
    rx = scene.receivers["veh"]
    floor_ids = {o.object_id for n, o in scene.objects.items() if "floor" in n.lower()}
    out = []
    for f in range(built.trajectory.shape[0]):
        x, y, _ = built.trajectory[f]
        rx.position = mi.Point3f(float(x), float(y), RX_HEIGHT_M)
        p = solver(scene, max_depth=max_depth, samples_per_src=ns,
                   seed=int(rng.integers(1, 2**31 - 1)))
        inter = np.asarray(p.interactions.numpy())[:, 0]      # (depth, n_tx, n_paths)
        tau = np.asarray(p.tau.numpy())[0]
        phi = np.asarray(p.phi_r.numpy())[0]
        valid = np.asarray(p.valid.numpy())[0]
        objs = np.asarray(p.objects.numpy())[0, 0]
        nb = np.count_nonzero(inter, axis=0)                  # (n_tx, n_paths)
        isfloor = np.vectorize(lambda o: int(o) in floor_ids)(objs)
        facade = (nb == 1) & (~isfloor) & valid
        out.append({"rng": tau * C, "phi": phi, "facade": facade, "valid": valid})
    return out


for scene, cfgp in [("controlled_wall", "configs/controlled_music_joint.yaml"),
                    ("street_canyon", "configs/street_metal_music.yaml")]:
    cfg = load_config(cfgp); built = build_scene(cfg)
    gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
    vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(cfg.seed))
    dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                              joint=cfg.joint_estimation)
    est, _ = run_slam(dets, built.ap_positions, vel, cfg.trajectory.timestep_s,
                      np.random.default_rng(0), init_pose=gt[0],
                      map_min_support=cfg.map_min_support,
                      map_min_excess_m=cfg.map_min_excess_m)
    tp = true_paths(built, cfg, np.random.default_rng(0))

    n_match_facade = n_match_other = n_nomatch = 0
    err_true, err_music, d_rng, d_aoa = [], [], [], []
    for f in range(len(dets)):
        D = np.asarray(dets[f]).reshape(-1, 3)
        T = tp[f]
        for (pl, aoa, ap_i) in D:
            a = int(ap_i)
            v = T["valid"][a]
            if not v.any():
                n_nomatch += 1; continue
            # nearest true path of the same AP in normalised (range, azimuth) space
            dr = (T["rng"][a] - pl) / 3.0                      # 3 m range scale
            da = np.arctan2(np.sin(T["phi"][a] - aoa), np.cos(T["phi"][a] - aoa)) / np.deg2rad(10)
            cost = np.where(v, np.hypot(dr, da), np.inf)
            j = int(np.argmin(cost))
            if not np.isfinite(cost[j]) or cost[j] > 3.0:     # no plausible match
                n_nomatch += 1; continue
            ap_xy = np.asarray(built.ap_positions[a])[:2]
            if not T["facade"][a][j]:
                n_match_other += 1; continue                  # DISCRIMINATION failure
            n_match_facade += 1
            d_rng.append(abs(T["rng"][a][j] - pl))
            d_aoa.append(abs(np.rad2deg(np.arctan2(np.sin(T["phi"][a][j] - aoa),
                                                   np.cos(T["phi"][a][j] - aoa)))))
            # SAME true facade path, triangulated with TRUE vs MUSIC params
            rt_ = _triangulate_bistatic(est[f][:2], ap_xy, T["rng"][a][j], T["phi"][a][j])
            rm_ = _triangulate_bistatic(est[f][:2], ap_xy, pl, aoa)
            if rt_ is not None:
                err_true.append(np.min(np.linalg.norm(gt_xy - rt_, axis=1)))
            if rm_ is not None:
                err_music.append(np.min(np.linalg.norm(gt_xy - rm_, axis=1)))

    tot = n_match_facade + n_match_other + n_nomatch
    print(f"\n=== {scene}: {tot} MUSIC detections ===")
    print(f"  matched a TRUE FACADE path : {n_match_facade} ({100*n_match_facade/tot:.1f}%)")
    print(f"  matched a NON-facade path  : {n_match_other} ({100*n_match_other/tot:.1f}%)  <- DISCRIMINATION failures")
    print(f"  no plausible match         : {n_nomatch} ({100*n_nomatch/tot:.1f}%)")
    if d_rng:
        print(f"  MUSIC estimation error on those facade paths: "
              f"range med={np.median(d_rng):.2f} m | azimuth med={np.median(d_aoa):.2f} deg")
    for tag, e in (("TRUE params (discrimination perfect)", err_true),
                   ("MUSIC params (adds estimation error)", err_music)):
        if not e:
            print(f"  {tag}: none"); continue
        e = np.array(e)
        print(f"  {tag}: n={len(e)} med={np.median(e):.2f} m | "
              f"within 1m={100*(e<=1).mean():.1f}% | within 2m={100*(e<=2).mean():.1f}%")
