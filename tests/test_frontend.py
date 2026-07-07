import numpy as np
from wifi_radar_slam.config import RFConfig
from wifi_radar_slam.sensing.frontend import extract_detections

C = 299792458.0


def _make_csi(n_frames, rf, delay_s, aoa_rad):
    freqs = np.linspace(-rf.bandwidth_hz / 2, rf.bandwidth_hz / 2, rf.n_subcarriers)
    idx = np.arange(rf.n_rx_antennas)
    csi = np.zeros((n_frames, 1, rf.n_rx_antennas, rf.n_subcarriers), dtype=complex)
    for f in range(n_frames):
        delay_phase = np.exp(-1j * 2 * np.pi * freqs * delay_s)             # (sub,)
        ant_phase = np.exp(-1j * 2 * np.pi * rf.antenna_spacing_frac * idx * np.sin(aoa_rad))
        csi[f, 0] = ant_phase[:, None] * delay_phase[None, :]
    return csi


def test_single_target_range_and_angle():
    rf = RFConfig(5.2e9, 160e6, 128, 8, 0.5)
    delay = 40e-9
    aoa = 0.3
    csi = _make_csi(5, rf, delay, aoa)
    dets = extract_detections(csi, rf, n_paths=1)
    assert len(dets) == 5
    r, a, ap = dets[0][0]
    assert np.isclose(r, delay * C, atol=1.0)      # within 1 m
    assert np.isclose(a, aoa, atol=0.05)
    assert ap == 0
