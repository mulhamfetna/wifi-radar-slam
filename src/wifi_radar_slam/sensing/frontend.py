from __future__ import annotations
import numpy as np
from ..config import RFConfig
from .superres import estimate_delays, estimate_aoa, azimuth_from_electrical

C = 299792458.0

# reflectors farther than this are non-physical; also bounds the MUSIC delay grid
MAX_RANGE_M = 150.0


def extract_detections(csi_timeseries: np.ndarray, rf: RFConfig, n_paths: int = 3) -> list[np.ndarray]:
    """CSI (n_frames, n_ap, n_rx_antennas, n_subcarriers) -> per-frame detections.

    Each per-frame array has shape (k, 3): columns [range_m, aoa_rad, ap_index],
    where aoa_rad is a world-frame azimuth. Delays use antennas as snapshots (and a
    physically-bounded grid); AoA uses subcarriers as snapshots and is converted
    from the array-relative electrical angle to world azimuth. Delays and angles
    are paired by sorted index (adequate for the low-target case; joint delay-AoA
    estimation is a documented future refinement).
    """
    n_frames, n_ap = csi_timeseries.shape[0], csi_timeseries.shape[1]
    out = []
    for f in range(n_frames):
        rows = []
        for ap in range(n_ap):
            block = csi_timeseries[f, ap]                    # (n_ant, n_sub)
            delays = np.sort(estimate_delays(block, rf.bandwidth_hz, n_paths,
                                             max_range_m=MAX_RANGE_M))
            electrical = estimate_aoa(block.T, rf.antenna_spacing_frac, n_paths)
            angles = np.sort(azimuth_from_electrical(electrical))
            k = min(len(delays), len(angles))
            for i in range(k):
                rows.append([delays[i] * C, angles[i], float(ap)])
        out.append(np.array(rows) if rows else np.empty((0, 3)))
    return out
