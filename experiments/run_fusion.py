"""RQ4: WiFi+LiDAR fusion on both scenes, for both LiDAR models (A/B).

For each (scene, lidar_model) emits the six metrics for:
  wifi_only, lidar_only, fused_tight, fused_loose

ORDERING HAZARD: SionnaLidarSensor (model B) MUTATES the scene -- it adds a lidar_tx
transmitter and sets scattering_coefficient on every material. The WiFi CSI must be
simulated BEFORE any model-B sensor is constructed, or the WiFi channel is corrupted.
Below, the WiFi detections are computed before the LiDAR-model loop. Do not reorder.

Run on a host with sionna-rt (amd server), throttled:
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_fusion.py
"""
import json
import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_geo import geo_sensor
from wifi_radar_slam.lidar.sensor_sionna import sionna_lidar_sensor
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam
from wifi_radar_slam.fusion import run_fused_slam, fuse_loose
from wifi_radar_slam.eval.metrics import (ate, rpe, chamfer, occupancy_iou,
                                          map_accuracy, map_completeness)

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}
LIDAR_MODELS = {"A_geometric": geo_sensor, "B_sionna": sionna_lidar_sensor}


def _metrics(est_traj, est_map, gt_traj, gt_xy):
    return {
        "ate": ate(est_traj, gt_traj), "rpe": rpe(est_traj, gt_traj),
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }


def main() -> None:
    results = {}
    for scene, cfgpath in SCENES.items():
        cfg = load_config(cfgpath)
        built = build_scene(cfg)
        gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
        vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)

        # --- realistic WiFi detections (commodity CSI -> joint 2-D MUSIC) ---
        # MUST run before any model-B sensor (see ORDERING HAZARD above).
        rng = np.random.default_rng(cfg.seed)
        csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
        dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                                  joint=cfg.joint_estimation)
        w_traj, w_map = run_slam(dets, built.ap_positions, vel,
                                 cfg.trajectory.timestep_s, np.random.default_rng(0),
                                 init_pose=gt[0], map_min_support=cfg.map_min_support,
                                 map_min_excess_m=cfg.map_min_excess_m)
        results.setdefault(scene, {})["wifi_only"] = _metrics(w_traj, w_map, gt, gt_xy)
        print(f"[{scene}] wifi_only: {results[scene]['wifi_only']}")

        for mname, make_sensor in LIDAR_MODELS.items():
            sensor = make_sensor(built, OUSTER_OS1, np.random.default_rng(0))
            scans = [sensor(gt[f]) for f in range(len(gt))]

            l_traj, l_map = run_lidar_slam(scans, vel, cfg.trajectory.timestep_s,
                                           np.random.default_rng(0), init_pose=gt[0])
            f_traj, f_map = run_fused_slam(dets, scans, built.ap_positions, vel,
                                           cfg.trajectory.timestep_s,
                                           np.random.default_rng(0), init_pose=gt[0],
                                           map_min_support=cfg.map_min_support,
                                           map_min_excess_m=cfg.map_min_excess_m)
            lo_traj, lo_map = fuse_loose(w_traj, w_map, l_traj, l_map)

            results[scene][f"lidar_only_{mname}"] = _metrics(l_traj, l_map, gt, gt_xy)
            results[scene][f"fused_tight_{mname}"] = _metrics(f_traj, f_map, gt, gt_xy)
            results[scene][f"fused_loose_{mname}"] = _metrics(lo_traj, lo_map, gt, gt_xy)
            print(f"[{scene}/{mname}] lidar_only: {results[scene][f'lidar_only_{mname}']}")
            print(f"[{scene}/{mname}] fused_tight: {results[scene][f'fused_tight_{mname}']}")
            print(f"[{scene}/{mname}] fused_loose: {results[scene][f'fused_loose_{mname}']}")

    with open("data/fusion_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/fusion_results.json")


if __name__ == "__main__":
    main()
