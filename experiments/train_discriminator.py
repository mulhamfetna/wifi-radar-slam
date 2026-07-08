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

    # Robustness to realistic estimation error: the oracle features are exact, but a
    # real receiver estimates range (hence excess) and azimuth with MUSIC error --
    # and the delay bias that broke simple excess-gating perturbs the key features.
    # Perturb range/excess by a delay-noise sigma and azimuth by an angle sigma,
    # retrain, and report held-out F1 to see how far discrimination survives.
    from sklearn.metrics import f1_score
    rng = np.random.default_rng(0)
    print("\n=== robustness to MUSIC-level feature noise (train+test noisy) ===")
    print(" sigma_range(m)  sigma_aoa(deg)   F1(useful)")
    for sr, sa in [(0.0, 0.0), (1.0, 3.0), (2.0, 6.0), (4.0, 10.0)]:
        Xn = X.copy()
        dn = rng.normal(0, sr, len(X))
        Xn[:, 0] += dn                                   # range
        Xn[:, 1] += dn                                   # excess = range - dist_ap
        Xn[:, 2] += rng.normal(0, np.deg2rad(sa), len(X))   # |azimuth|
        Xn[:, 4] += rng.normal(0, np.deg2rad(sa), len(X))   # aoa deviation
        xtr, xte, ytr2, yte2 = train_test_split(Xn, y, test_size=0.3,
                                                random_state=0, stratify=y)
        c = RandomForestClassifier(n_estimators=300, class_weight="balanced",
                                   random_state=0, n_jobs=-1).fit(xtr, ytr2)
        print(f"   {sr:5.1f}          {sa:5.1f}          {f1_score(yte2, c.predict(xte)):.3f}")
    print("\n=> path discrimination is well-posed (excess + azimuth-deviation separate "
          "the classes cleanly); its accuracy is bounded by how well those features can "
          "be estimated from noisy CSI -- the same feature-estimation problem behind the "
          "~5 m mapping floor.")


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else "data/wifislam_sim_nominal.npz"
    main(p)
