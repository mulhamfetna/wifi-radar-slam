"""RQ2: train the learned map filter on MUSIC-OBSERVABLE features only.

Paper 1's discriminator also used `elevation`, which a single-ULA 2-D MUSIC front-end
cannot measure -- an oracle feature. We retrain on the observable 4 and report the
corrected F1. Two leakage-free splits: held-out frames, and cross-scene.

Server (needs sionna-rt):
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/train_map_filter.py
"""
import json
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import f1_score, classification_report

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam
from wifi_radar_slam.map_filter import music_features, label_from_gt

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}


def collect(scene, cfgpath):
    """(X, y, frame_idx) at the ESTIMATED poses of an unfiltered run."""
    cfg = load_config(cfgpath)
    built = build_scene(cfg)
    gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
    vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(cfg.seed))
    dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                              joint=cfg.joint_estimation)
    est, _ = run_slam(dets, built.ap_positions, vel, cfg.trajectory.timestep_s,
                      np.random.default_rng(0), init_pose=gt[0],
                      map_min_support=cfg.map_min_support,
                      map_min_excess_m=cfg.map_min_excess_m)
    Xs, ys, fs = [], [], []
    for f in range(len(dets)):
        if dets[f].shape[0] == 0:
            continue
        Xs.append(music_features(dets[f], est[f], built.ap_positions))
        ys.append(label_from_gt(dets[f], est[f], built.ap_positions, gt_xy, tol=1.0))
        fs.append(np.full(dets[f].shape[0], f))
    return np.vstack(Xs), np.concatenate(ys), np.concatenate(fs)


def _fit_and_score(Xtr, ytr, Xte, yte, tag):
    out = {}
    for name, model in (("rf", RandomForestClassifier(n_estimators=300,
                                                      class_weight="balanced",
                                                      random_state=0, n_jobs=-1)),
                        ("mlp", MLPClassifier(hidden_layer_sizes=(32, 16), max_iter=2000,
                                              random_state=0))):
        if ytr.sum() == 0 or ytr.sum() == len(ytr):
            print(f"[{tag}/{name}] degenerate labels; skipping")
            continue
        model.fit(Xtr, ytr)
        f1 = f1_score(yte, model.predict(Xte), zero_division=0)
        out[name] = f1
        print(f"[{tag}/{name}] F1 = {f1:.3f}")
        print(classification_report(yte, model.predict(Xte), zero_division=0, digits=3))
    return out


def main() -> None:
    data = {s: collect(s, p) for s, p in SCENES.items()}
    report = {}

    for scene, (X, y, f) in data.items():
        print(f"\n=== {scene}: {len(y)} detections, useful = {int(y.sum())} "
              f"({100 * y.mean():.1f}%) ===")
        cut = np.quantile(f, 0.6)                      # held-out FRAMES (temporal split)
        tr, te = f <= cut, f > cut
        report[f"{scene}_heldout_frames"] = _fit_and_score(X[tr], y[tr], X[te], y[te],
                                                           f"{scene}/frames")

    names = list(SCENES)                               # cross-scene generalization
    for a, b in ((names[0], names[1]), (names[1], names[0])):
        Xa, ya, _ = data[a]
        Xb, yb, _ = data[b]
        report[f"train_{a}_test_{b}"] = _fit_and_score(Xa, ya, Xb, yb, f"{a}->{b}")

    # Ship models trained on the FULL data of each scene. The map evaluation applies
    # each scene's model to the *other* scene (cross-scene), so training on all frames
    # here introduces NO leakage -- and it is the honest deployment case (a filter
    # trained offline, deployed somewhere new).
    for scene, (X, y, _f) in data.items():
        rf = RandomForestClassifier(n_estimators=300, class_weight="balanced",
                                    random_state=0, n_jobs=-1).fit(X, y)
        mlp = MLPClassifier(hidden_layer_sizes=(32, 16), max_iter=2000,
                            random_state=0).fit(X, y)
        joblib.dump(rf, f"data/map_filter_rf_{scene}.joblib")
        joblib.dump(mlp, f"data/map_filter_mlp_{scene}.joblib")

    with open("data/map_filter_f1.json", "w") as fh:
        json.dump(report, fh, indent=2)
    print("\nsaved -> data/map_filter_f1.json + per-scene rf/mlp models")


if __name__ == "__main__":
    main()
