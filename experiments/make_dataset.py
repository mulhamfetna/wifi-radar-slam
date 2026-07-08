"""Generate the ray-traced outdoor/vehicular WiFi-CSI dataset from a config.

Produces a CsiDataset .npz: noisy CSI + ground-truth poses/APs/map + a flat oracle
path table (with per-path bounce count / interaction type / object id) that labels
each ray-traced path for learned path-discrimination research. Requires sionna-rt.

Usage:
    python experiments/make_dataset.py configs/nominal.yaml data/wifislam_sim.npz
"""
import sys
import json
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.dataset import CsiDataset
from wifi_radar_slam.geometry import RX_HEIGHT_M


def _oracle_paths(built, rng):
    """Flat oracle path table over the trajectory (one row per ray-traced path)."""
    import os
    import sionna.rt as rt
    import mitsuba as mi
    n_samples = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
    scene = built.scene
    solver = rt.PathSolver()
    rx = scene.receivers["veh"]
    floor_ids = {o.object_id for n, o in scene.objects.items() if "floor" in n.lower()}
    rows = []
    for f in range(built.trajectory.shape[0]):
        x, y, _ = built.trajectory[f]
        rx.position = mi.Point3f(float(x), float(y), RX_HEIGHT_M)
        p = solver(scene, max_depth=3, samples_per_src=n_samples,
                   seed=int(rng.integers(1, 2**31 - 1)))
        inter = np.asarray(p.interactions.numpy())[:, 0]     # (depth, n_tx, n_paths)
        tau = np.asarray(p.tau.numpy())[0]
        phir = np.asarray(p.phi_r.numpy())[0]
        thetar = np.asarray(p.theta_r.numpy())[0]
        valid = np.asarray(p.valid.numpy())[0]
        objs = np.asarray(p.objects.numpy())[0, 0]
        for ap in range(tau.shape[0]):
            for q in range(tau.shape[1]):
                if not valid[ap, q]:
                    continue
                types = inter[:, ap, q]
                oid = int(objs[ap, q])
                rows.append([f, ap, float(tau[ap, q]), float(phir[ap, q]),
                             float(thetar[ap, q]), int(np.count_nonzero(types)),
                             int(types[0]), oid, int(oid in floor_ids)])
    return np.array(rows, dtype=float)


def main(config_path: str, out_path: str):
    cfg = load_config(config_path)
    rng = np.random.default_rng(cfg.seed)
    built = build_scene(cfg)
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng).astype(np.complex64)
    paths = _oracle_paths(built, np.random.default_rng(cfg.seed + 1))
    meta = {
        "name": "WiFiSLAM-Sim: ray-traced outdoor/vehicular WiFi-CSI",
        "carrier_hz": cfg.rf.carrier_hz, "bandwidth_hz": cfg.rf.bandwidth_hz,
        "n_subcarriers": cfg.rf.n_subcarriers, "n_rx_antennas": cfg.rf.n_rx_antennas,
        "antenna_spacing_frac": cfg.rf.antenna_spacing_frac, "snr_db": cfg.snr_db,
        "scene": cfg.scene.name, "n_frames": int(built.trajectory.shape[0]),
        "units": {"positions": "m", "delay": "s", "angles": "rad"},
        "simulator": "Sionna RT 2.0 (ray-traced, not real hardware)",
        "license": "AGPL-3.0-or-later", "config": config_path,
    }
    ds = CsiDataset(csi=csi, poses=built.trajectory.astype(float),
                    ap_positions=np.asarray(built.ap_positions, dtype=float),
                    gt_map=built.ground_truth_map[:, :2].astype(float),
                    paths=paths, meta=meta)
    ds.save(out_path)
    print(f"wrote {out_path}")
    print(f"  csi {ds.csi.shape}, poses {ds.poses.shape}, aps {ds.ap_positions.shape}, "
          f"gt_map {ds.gt_map.shape}, paths {ds.paths.shape}")
    pf = ds.path_frame()
    nb = pf["n_bounce"]
    print(f"  path bounce-count histogram: "
          f"{dict(zip(*[a.tolist() for a in np.unique(nb, return_counts=True)]))}")


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "configs/nominal.yaml"
    out = sys.argv[2] if len(sys.argv) > 2 else "data/wifislam_sim.npz"
    import os
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    main(cfg, out)
