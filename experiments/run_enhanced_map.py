"""RQ2: rebuild the WiFi map under each ladder rung and score it.

rung 0 = none (pure WiFi)   rung 1 = physics heuristic (min-excess gate)
rung 2 = RandomForest       rung 3 = MLP

LEAKAGE-FREE BY CONSTRUCTION: each scene's map is rebuilt using the model trained on the
*other* scene. So no frame the filter trained on contributes to the map it is scored on,
the full trajectory is kept (numbers stay comparable to the LiDAR/fusion rows), and this
is the honest deployment case -- a filter trained offline, deployed somewhere new.

Server (needs sionna-rt):
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_enhanced_map.py
"""
import json
import joblib
import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam
from wifi_radar_slam.map_filter import HeuristicFilter, SklearnFilter
from wifi_radar_slam.eval.metrics import (ate, rpe, chamfer, occupancy_iou,
                                          map_accuracy, map_completeness)

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}


def _metrics(est, m, gt, gt_xy):
    return {"ate": ate(est, gt), "rpe": rpe(est, gt), "chamfer": chamfer(m, gt_xy),
            "map_accuracy": map_accuracy(m, gt_xy),
            "map_completeness": map_completeness(m, gt_xy),
            "iou": occupancy_iou(m, gt_xy, cell=1.0),
            "n_map_points": int(m.shape[0])}


def main() -> None:
    results = {}
    names = list(SCENES)
    other = {names[0]: names[1], names[1]: names[0]}    # cross-scene: no leakage
    for scene, cfgpath in SCENES.items():
        cfg = load_config(cfgpath)
        built = build_scene(cfg)
        gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
        vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)
        csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(cfg.seed))
        dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                                  joint=cfg.joint_estimation)

        src = other[scene]          # model trained on the OTHER scene
        print(f"[{scene}] applying filters trained on '{src}' (leakage-free)")
        rungs = {
            "0_none": None,
            "1_heuristic": HeuristicFilter(min_excess_m=1.5),
            "2_random_forest": SklearnFilter(
                joblib.load(f"data/map_filter_rf_{src}.joblib")),
            "3_mlp": SklearnFilter(
                joblib.load(f"data/map_filter_mlp_{src}.joblib")),
        }
        results[scene] = {}
        for name, filt in rungs.items():
            est, m = run_slam(dets, built.ap_positions, vel, cfg.trajectory.timestep_s,
                              np.random.default_rng(0), init_pose=gt[0],
                              map_min_support=cfg.map_min_support,
                              map_min_excess_m=cfg.map_min_excess_m, map_filter=filt)
            results[scene][name] = _metrics(est, m, gt, gt_xy)
            print(f"[{scene}/{name}] {results[scene][name]}")

    with open("data/enhanced_map_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/enhanced_map_results.json")


if __name__ == "__main__":
    main()
