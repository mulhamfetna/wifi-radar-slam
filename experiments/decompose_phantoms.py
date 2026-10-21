#!/usr/bin/env python3
"""AS-PUBLISHED interpretation (excess length read as absolute) -- kept to reproduce the
manuscript's ~89 % and show its folding share. The corrected numbers are produced by
``experiments/run_corrected_eval.py``.

Decompose the realistic-CSI "phantom" rate: how much is single-ULA front/back folding?

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
from wifi_radar_slam.eval.scenes import SceneCache, music_detections
from wifi_radar_slam.eval.phantom_decomp import decompose

C = 299792458.0
COL = {c: i for i, c in enumerate(PATH_COLUMNS)}
SCENES = {"controlled_wall": "data/cache_controlled_wall.npz",
          "street_canyon": "data/cache_street_canyon_metal.npz",
          "street_canyon_scurve": "data/cache_street_canyon_metal_scurve.npz"}
SEEDS = range(5)
BW, SNR = 160, 20.0


def main():
    res = {"bandwidth_mhz": BW, "snr_db": SNR, "seeds": list(SEEDS),
           "tolerance": "3 m / 10 deg / cost 3, per frame, same AP", "scenes": {}}
    for name, path in SCENES.items():
        if not os.path.exists(path):
            print(f"skip {name}: {path} missing"); continue
        sc = SceneCache.load(path)
        runs = [decompose(sc, music_detections(sc, BW, SNR, np.random.default_rng(s)))
                for s in SEEDS]
        keys = [k for k in runs[0] if not k.startswith("n_")]
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
