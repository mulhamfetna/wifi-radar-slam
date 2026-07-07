from __future__ import annotations
import numpy as np
from ..config import RFConfig
from ..scene.builder import BuiltScene
from ..geometry import RX_HEIGHT_M


def _subcarrier_freqs(rf: RFConfig) -> np.ndarray:
    return np.linspace(-rf.bandwidth_hz / 2, rf.bandwidth_hz / 2, rf.n_subcarriers)


def simulate_csi(built: BuiltScene, rf: RFConfig, snr_db: float, rng) -> np.ndarray:
    """Ray-trace the AP->vehicle channel per frame -> CSI timeseries.

    Returns complex array (n_frames, n_ap, n_rx_antennas, n_subcarriers).

    NOTE (Sionna API): compute_paths(...) / paths.cfr(...) target Sionna RT
    0.19.x. Newer Sionna exposes PathSolver + paths.cir(); if so, adapt within
    this file and keep the returned shape identical. On first bring-up, print
    `h_freq.shape` once to confirm the squeeze/transpose, then remove the print.
    """
    import os
    import sionna.rt as rt   # lazy: GPU stage, only imported when actually simulating

    # Ray-tracing sample count dominates VRAM. Override on low-memory GPUs
    # (e.g. 4 GB) via WRS_NUM_SAMPLES to avoid OOM during local bring-up.
    num_samples = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
    scene = built.scene
    n_frames = built.trajectory.shape[0]
    n_ap = len(built.ap_positions)
    freqs = _subcarrier_freqs(rf)
    csi = np.zeros((n_frames, n_ap, rf.n_rx_antennas, rf.n_subcarriers), dtype=complex)

    scene.add(rt.Receiver(name="veh", position=[0.0, 0.0, RX_HEIGHT_M]))

    for f in range(n_frames):
        x, y, _yaw = built.trajectory[f]
        scene.receivers["veh"].position = [float(x), float(y), RX_HEIGHT_M]
        paths = scene.compute_paths(max_depth=3, num_samples=num_samples)
        # channel frequency response; collapse the singleton tx/rx indices,
        # keep [tx=ap, rx_antenna, freq].
        h_freq = paths.cfr(frequencies=freqs, normalize=False).numpy()
        h = np.squeeze(h_freq)
        if h.ndim == 2:               # single-AP edge case -> add ap axis
            h = h[None, ...]
        csi[f] = h[:n_ap]

    # additive white Gaussian noise at the configured SNR
    sig_p = np.mean(np.abs(csi) ** 2) + 1e-30
    noise_p = sig_p / (10 ** (snr_db / 10))
    noise = (rng.normal(size=csi.shape) + 1j * rng.normal(size=csi.shape)) * np.sqrt(noise_p / 2)
    return csi + noise
