#!/usr/bin/env python3
"""Decompose the realistic-CSI "phantom" rate: how much is single-ULA front/back folding?

The manuscript's ~89 % phantom figure matched each joint-MUSIC detection against the TRUE
full-circle azimuth. A single uniform linear array only reports the forward branch
(``sensing.superres.azimuth_from_electrical``), so a real path arriving from behind the
array is reported at its mirror angle and fails the 10-degree match. This script re-scores
the same detections with the same tolerances (3 m / 10 deg / cost 3, per frame, same AP)
and splits them into:

  matched           -- explained by a real path at its true azimuth
  folded            -- explained by a real path only after mirroring its azimuth onto the
                       forward branch (an ambiguity, not an estimator artefact)
  unexplained       -- no real path either way (a genuine estimator phantom)

Each of the first two is further split into facade single-bounce vs other real paths.

    .venv/bin/python experiments/decompose_phantoms.py   -> results/phantom_decomposition.json
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

from wifi_radar_slam.dataset import PATH_COLUMNS
from wifi_radar_slam.eval.scenes import SceneCache, music_detections, fold_forward

C = 299792458.0
COL = {c: i for i, c in enumerate(PATH_COLUMNS)}
SCENES = {"controlled_wall": "data/cache_controlled_wall.npz",
          "street_canyon": "data/cache_street_canyon_metal.npz",
          "street_canyon_scurve": "data/cache_street_canyon_metal_scurve.npz"}
SEEDS = range(5)
BW, SNR = 160, 20.0


def _cost(rng_t, phi_t, r, a):
    dr = (rng_t - r) / 3.0
    da = np.angle(np.exp(1j * (phi_t - a))) / np.deg2rad(10)
    return np.hypot(dr, da)


def decompose(sc: SceneCache, dets) -> dict:
    n = {"matched_facade": 0, "matched_other": 0, "folded_facade": 0, "folded_other": 0,
         "unexplained": 0}
    bias_facade = []
    for f, D in enumerate(dets):
        P = sc.frame_paths(f)
        for r, a, ap in np.asarray(D).reshape(-1, 3):
            T = P[P[:, COL["ap"]] == ap]
            if not len(T):
                n["unexplained"] += 1; continue
            rt, pt = T[:, COL["delay_s"]] * C, T[:, COL["phi_r"]]
            facade = (T[:, COL["n_bounce"]] == 1) & (T[:, COL["is_floor"]] == 0)
            c0 = _cost(rt, pt, r, a); j0 = int(np.argmin(c0))
            if c0[j0] <= 3.0:
                k = "matched_facade" if facade[j0] else "matched_other"
                n[k] += 1
                if facade[j0]:
                    bias_facade.append(r - rt[j0])
                continue
            c1 = _cost(rt, fold_forward(pt), r, a); j1 = int(np.argmin(c1))
            if c1[j1] <= 3.0:
                n["folded_facade" if facade[j1] else "folded_other"] += 1
            else:
                n["unexplained"] += 1
    tot = sum(n.values())
    out = {k: v / tot for k, v in n.items()}
    out["n_detections"] = tot
    out["legacy_phantom_rate"] = (n["folded_facade"] + n["folded_other"]
                                  + n["unexplained"]) / tot
    out["median_signed_range_err_facade_m"] = (float(np.median(bias_facade))
                                               if bias_facade else float("nan"))
    return out


def main():
    res = {"bandwidth_mhz": BW, "snr_db": SNR, "seeds": list(SEEDS),
           "tolerance": "3 m / 10 deg / cost 3, per frame, same AP", "scenes": {}}
    for name, path in SCENES.items():
        if not os.path.exists(path):
            print(f"skip {name}: {path} missing"); continue
        sc = SceneCache.load(path)
        runs = [decompose(sc, music_detections(sc, BW, SNR, np.random.default_rng(s)))
                for s in SEEDS]
        keys = [k for k in runs[0] if k != "n_detections"]
        res["scenes"][name] = {k: {"mean": float(np.nanmean([r[k] for r in runs])),
                                   "std": float(np.nanstd([r[k] for r in runs]))}
                               for k in keys}
        res["scenes"][name]["n_detections_per_seed"] = runs[0]["n_detections"]
        print(f"\n{name}")
        for k in keys:
            v = res["scenes"][name][k]
            print(f"  {k:36s} {v['mean']:8.3f} +/- {v['std']:.3f}")
    os.makedirs("results", exist_ok=True)
    out = sys.argv[1] if len(sys.argv) > 1 else "results/phantom_decomposition.json"
    json.dump(res, open(out, "w"), indent=2)
    print(f"\nsaved -> {out}")


if __name__ == "__main__":
    main()
