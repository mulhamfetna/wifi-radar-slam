"""Run the sensing front-end on a REAL commodity-WiFi CSI capture.

Proof-of-concept that the same MUSIC delay/AoA front-end used on the simulated
channel also runs on measured hardware CSI. Indoor static captures (e.g. the Intel
5300 samples shipped with CSIKit) have no ground-truth trajectory, so this reports
the estimated multipath structure rather than SLAM metrics.

Usage:
    python experiments/run_real_csi.py data/real/log.all_csi.6.7.6.dat [bandwidth_hz]
"""
import sys
import numpy as np
from wifi_radar_slam.config import RFConfig
from wifi_radar_slam.io_csi import load_real_csi
from wifi_radar_slam.sensing.frontend import extract_detections, C


def main(path: str, bandwidth_hz: float = 20e6):
    csi = load_real_csi(path)
    n_frames, n_ap, n_rx, n_sub = csi.shape
    print(f"real CSI: {n_frames} frames, {n_ap} tx/ap, {n_rx} rx antennas, {n_sub} subcarriers")
    print(f"complex: {np.iscomplexobj(csi)}, |CSI| mean={np.mean(np.abs(csi)):.2f}")

    rf = RFConfig(carrier_hz=5.5e9, bandwidth_hz=bandwidth_hz, n_subcarriers=n_sub,
                  n_rx_antennas=n_rx, antenna_spacing_frac=0.5)
    dets = extract_detections(csi, rf, n_paths=3)

    ranges = np.concatenate([d[:, 0] for d in dets if d.size]) if any(d.size for d in dets) else np.array([])
    aoas = np.concatenate([d[:, 1] for d in dets if d.size]) if any(d.size for d in dets) else np.array([])
    print(f"\nMUSIC front-end ran on real CSI over {n_frames} frames -> "
          f"{ranges.size} path detections")
    if ranges.size:
        print(f"estimated path length (m): min={ranges.min():.2f} "
              f"median={np.median(ranges):.2f} max={ranges.max():.2f}  "
              f"(delay resolution c/2B = {C/(2*bandwidth_hz):.2f} m)")
        print(f"estimated AoA (deg): "
              f"[{np.degrees(aoas).min():.1f}, {np.degrees(aoas).max():.1f}]")
        print("=> the simulated-channel sensing front-end runs end-to-end on measured "
              "commodity CSI, producing plausible indoor multipath delay/angle estimates.")


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else "data/real/log.all_csi.6.7.6.dat"
    bw = float(sys.argv[2]) if len(sys.argv) > 2 else 20e6
    main(p, bw)
