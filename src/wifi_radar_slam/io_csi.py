"""Ingest real commodity-WiFi CSI captures into the pipeline CSI format.

Bridges real hardware CSI (Intel 5300, nexmon/Broadcom, ESP32, PicoScenes,
FeitCSI) to the same complex array the simulator produces, so the sensing
front-end and SLAM back-end can be exercised on measured data. Parsing is done
with CSIKit (https://github.com/Gi-z/CSIKit); install with the `realcsi` extra.

Pipeline CSI shape: (n_frames, n_ap, n_rx_antennas, n_subcarriers), complex.
A single transmitter maps to n_ap = 1.
"""
from __future__ import annotations
import numpy as np


def csi_frames_to_pipeline(frames: list[np.ndarray]) -> np.ndarray:
    """Stack per-frame CSI matrices (n_sub, n_rx[, n_tx]) into the pipeline array.

    Real captures can have per-frame shape variation (e.g. Intel 5300 antenna
    selection), so only frames sharing the modal shape are kept. Returns complex
    (n_frames, n_ap, n_rx_antennas, n_subcarriers).
    """
    norm = []
    for m in frames:
        m = np.asarray(m)
        if m.ndim == 2:                       # (n_sub, n_rx) -> add tx axis
            m = m[:, :, None]
        norm.append(m)
    if not norm:
        return np.zeros((0, 1, 0, 0), dtype=complex)
    shapes = [m.shape for m in norm]
    modal = max(set(shapes), key=shapes.count)      # most common (n_sub, n_rx, n_tx)
    kept = np.stack([m for m in norm if m.shape == modal])   # (n_frames, n_sub, n_rx, n_tx)
    # -> (n_frames, n_ap=n_tx, n_rx_antennas=n_rx, n_subcarriers=n_sub)
    return np.transpose(kept, (0, 3, 2, 1)).astype(complex)


def load_real_csi(path: str) -> np.ndarray:
    """Parse a real CSI capture file into the pipeline CSI format via CSIKit.

    Supports the formats CSIKit supports (.dat Intel 5300, .pcap nexmon, ESP32,
    PicoScenes, .csi FeitCSI). Returns complex
    (n_frames, n_ap, n_rx_antennas, n_subcarriers).
    """
    from CSIKit.reader import get_reader   # lazy: optional dependency
    reader = get_reader(path)
    data = reader.read_file(path)
    frames = [fr.csi_matrix for fr in data.frames]
    return csi_frames_to_pipeline(frames)
