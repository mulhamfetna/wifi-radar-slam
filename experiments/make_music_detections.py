#!/usr/bin/env python3
"""Stage 1 of the corrected evaluation: joint 2-D MUSIC detections from the cached channels.

One file per (scene, bandwidth, seed); the seed draws the receiver AWGN. Detections keep the
commodity-CSI convention: column 0 is the EXCESS path length over the first arrival.

    .venv/bin/python experiments/make_music_detections.py   -> results/dets/*.npz (gitignored)
"""
from __future__ import annotations

import os
import sys
from multiprocessing import Pool

import numpy as np

SCENES = {"controlled_wall": "data/cache_controlled_wall.npz",
          "street_canyon": "data/cache_street_canyon_metal.npz",
          "street_canyon_scurve": "data/cache_street_canyon_metal_scurve.npz"}
MAIN_BW, SWEEP_BWS = 160, (20, 40, 80)
SNR_DB = 20.0
N_SEEDS, N_SWEEP_SEEDS = 10, 5


def det_path(scene, bw, seed):
    return f"results/dets/{scene}_bw{bw}_s{seed}.npz"


def _job(args):
    scene, bw, seed = args
    out = det_path(scene, bw, seed)
    if os.path.exists(out):
        return out
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from wifi_radar_slam.eval.scenes import SceneCache, music_detections
    sc = SceneCache.load(SCENES[scene])
    dets = music_detections(sc, bw, SNR_DB, np.random.default_rng(seed))
    np.savez_compressed(out, **{f"f{i}": d for i, d in enumerate(dets)})
    return out


def load_dets(scene, bw, seed):
    z = np.load(det_path(scene, bw, seed))
    return [z[f"f{i}"] for i in range(len(z.files))]


def jobs():
    js = [(s, MAIN_BW, k) for s in SCENES for k in range(N_SEEDS)]
    js += [(s, bw, k) for s in ("controlled_wall", "street_canyon") for bw in SWEEP_BWS
           for k in range(N_SWEEP_SEEDS)]
    return js


if __name__ == "__main__":
    os.makedirs("results/dets", exist_ok=True)
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    with Pool(n) as p:
        for i, o in enumerate(p.imap_unordered(_job, jobs())):
            print(f"[{i + 1}/{len(jobs())}] {o}", flush=True)
