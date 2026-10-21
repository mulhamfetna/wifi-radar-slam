#!/usr/bin/env python3
"""Ray-trace a scene ONCE and cache everything the back-ends need (Sionna-free afterwards).

Per frame, one PathSolver call yields (a) the noise-free channel frequency response at each
requested bandwidth and (b) the flat oracle path table (``dataset.PATH_COLUMNS``). AWGN is
added later, per seed, by ``eval.scenes.noisy_csi`` -- so seeds vary the receiver noise and
the odometry, not the ray tracer's sampling.

    .venv/bin/python experiments/cache_channels.py configs/controlled_music_joint.yaml
      -> data/cache_controlled_wall.npz   (gitignored; regenerate with this script)
"""
from __future__ import annotations

import json
import os
import sys
import time

import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.geometry import RX_HEIGHT_M
from wifi_radar_slam.scene.builder import build_scene

BANDWIDTHS = (20e6, 40e6, 80e6, 160e6)


def main(config_path: str, out_path: str | None = None, max_frames: int | None = None):
    import sionna.rt as rt
    import mitsuba as mi
    cfg = load_config(config_path)
    built = build_scene(cfg)
    scene, rx = built.scene, built.scene.receivers["veh"]
    n_samples = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
    solver = rt.PathSolver()
    floor_ids = {o.object_id for n, o in scene.objects.items() if "floor" in n.lower()}
    n_sub = cfg.rf.n_subcarriers
    freqs = {bw: rt.subcarrier_frequencies(n_sub, bw / n_sub) for bw in BANDWIDTHS}
    n = built.trajectory.shape[0] if max_frames is None else min(max_frames,
                                                                 built.trajectory.shape[0])
    n_ap = len(built.ap_positions)
    cfr = {bw: np.zeros((n, n_ap, cfg.rf.n_rx_antennas, n_sub), np.complex64)
           for bw in BANDWIDTHS}
    rows = []
    rng = np.random.default_rng(cfg.seed)
    t0 = time.time()
    for f in range(n):
        x, y, _ = built.trajectory[f]
        rx.position = mi.Point3f(float(x), float(y), RX_HEIGHT_M)
        p = solver(scene, max_depth=3, samples_per_src=n_samples,
                   seed=int(rng.integers(1, 2**31 - 1)))
        for bw in BANDWIDTHS:
            h = np.asarray(p.cfr(frequencies=freqs[bw], normalize=False, out_type="numpy"))
            cfr[bw][f] = np.transpose(h[0, :, :, 0, 0, :], (1, 0, 2))
        inter = np.asarray(p.interactions.numpy())[:, 0]
        tau = np.asarray(p.tau.numpy())[0]; phir = np.asarray(p.phi_r.numpy())[0]
        thetar = np.asarray(p.theta_r.numpy())[0]; valid = np.asarray(p.valid.numpy())[0]
        objs = np.asarray(p.objects.numpy())[0, 0]
        for ap in range(tau.shape[0]):
            for q in range(tau.shape[1]):
                if valid[ap, q]:
                    oid = int(objs[ap, q]); types = inter[:, ap, q]
                    rows.append([f, ap, float(tau[ap, q]), float(phir[ap, q]),
                                 float(thetar[ap, q]), int(np.count_nonzero(types)),
                                 int(types[0]), oid, int(oid in floor_ids)])
        if f % 20 == 0:
            print(f"  frame {f}/{n}  {time.time() - t0:.0f}s", flush=True)
    out_path = out_path or f"data/cache_{cfg.scene.name}.npz"
    meta = {"config": config_path, "scene": cfg.scene.name, "carrier_hz": cfg.rf.carrier_hz,
            "n_subcarriers": n_sub, "n_rx_antennas": cfg.rf.n_rx_antennas,
            "antenna_spacing_frac": cfg.rf.antenna_spacing_frac,
            "timestep_s": cfg.trajectory.timestep_s, "samples_per_src": n_samples,
            "bandwidths_hz": list(BANDWIDTHS), "simulator": "Sionna RT 2.0.1"}
    np.savez_compressed(out_path, poses=built.trajectory[:n].astype(float),
                        ap_positions=np.asarray(built.ap_positions, float),
                        gt_map=built.ground_truth_map[:, :2].astype(float),
                        paths=np.array(rows, float), meta=json.dumps(meta),
                        **{f"cfr_{int(bw / 1e6)}": c for bw, c in cfr.items()})
    print(f"wrote {out_path} ({n} frames, {len(rows)} paths, {time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main(sys.argv[1], *(sys.argv[2:3] or [None]),
         max_frames=int(sys.argv[3]) if len(sys.argv) > 3 else None)
