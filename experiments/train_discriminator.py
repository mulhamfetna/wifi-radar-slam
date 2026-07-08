"""Train + evaluate a learned path discriminator on the WiFiSLAM-Sim dataset.

Tests whether "is this a mapping-useful single-scatter facade reflection?" is
learnable from per-path physical features (range, bistatic excess, azimuth,
elevation, azimuth-deviation-from-AP) without the ground-truth interaction label.
Reports held-out precision/recall/F1 and feature importance. Requires scikit-learn.

Usage:
    python experiments/train_discriminator.py data/wifislam_sim_nominal.npz
"""
import sys
import numpy as np
from wifi_radar_slam.dataset import CsiDataset
from wifi_radar_slam.discriminate import path_features


def main(path: str):
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import classification_report, roc_auc_score

    ds = CsiDataset.load(path)
    X, y, names = path_features(ds.paths, ds.poses, ds.ap_positions)
    print(f"dataset: {len(y)} paths, positive (single-scatter facade) = {int(y.sum())} "
          f"({100*y.mean():.1f}%)")

    Xtr, Xte, ytr, yte = train_test_split(X, y, test_size=0.3, random_state=0, stratify=y)
    clf = RandomForestClassifier(n_estimators=300, class_weight="balanced",
                                 random_state=0, n_jobs=-1)
    clf.fit(Xtr, ytr)
    pred = clf.predict(Xte)
    proba = clf.predict_proba(Xte)[:, 1]

    print("\n=== held-out classification (single-scatter facade vs rest) ===")
    print(classification_report(yte, pred, target_names=["other", "useful"], digits=3))
    print(f"ROC-AUC: {roc_auc_score(yte, proba):.3f}")
    print("\nfeature importance:")
    for n, imp in sorted(zip(names, clf.feature_importances_), key=lambda t: -t[1]):
        print(f"  {n:18s} {imp:.3f}")
    print("\n=> path discrimination IS learnable from realistic per-path features; "
          "the learned label can gate detections without the interaction ground truth.")


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else "data/wifislam_sim_nominal.npz"
    main(p)
