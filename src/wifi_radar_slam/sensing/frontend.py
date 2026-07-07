from __future__ import annotations
import numpy as np
from ..config import RFConfig
from .superres import estimate_delays, estimate_aoa

C = 299792458.0


def extract_detections(csi_timeseries: np.ndarray, rf: RFConfig, n_paths: int = 3) -> list[np.ndarray]:
    """CSI (n_frames, n_ap, n_rx_antennas, n_subcarriers) -> per-frame detections.

    Each per-frame array has shape (k, 3): columns [range_m, aoa_rad, ap_index].
    v1 pairs delays with angles by sorted index (adequate for the low-target
    nominal case); joint delay-AoA estimation is a documented future refinement.
    """
    n_frames, n_ap = csi_timeseries.shape[0], csi_timeseries.shape[1]
    out = []
    for f in range(n_frames):
        rows = []
        for ap in range(n_ap):
            block = csi_timeseries[f, ap]                    # (n_ant, n_sub)
            csi_freq = block.mean(axis=0)                    # collapse antennas -> delays
            delays = np.sort(estimate_delays(csi_freq, rf.bandwidth_hz, n_paths))
            csi_ant = block.mean(axis=1)                     # collapse subcarriers -> AoA
            angles = np.sort(estimate_aoa(csi_ant, rf.antenna_spacing_frac, n_paths))
            k = min(len(delays), len(angles))
            for i in range(k):
                rows.append([delays[i] * C, angles[i], float(ap)])
        out.append(np.array(rows) if rows else np.empty((0, 3)))
    return out
