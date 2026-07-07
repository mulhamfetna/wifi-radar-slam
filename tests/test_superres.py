import numpy as np
from wifi_radar_slam.sensing.superres import estimate_delays, estimate_aoa


def _synth_freq(delays, subcarrier_freqs):
    csi = np.zeros(subcarrier_freqs.shape, dtype=complex)
    for tau in delays:
        csi += np.exp(-1j * 2 * np.pi * subcarrier_freqs * tau)
    return csi


def test_recover_two_delays():
    bw = 160e6
    n = 128
    freqs = np.linspace(-bw / 2, bw / 2, n)
    true = np.array([20e-9, 60e-9])         # 20 ns, 60 ns  (~6 m, ~18 m paths)
    csi = _synth_freq(true, freqs)
    est = np.sort(estimate_delays(csi, bandwidth_hz=bw, n_paths=2))
    assert np.allclose(est, true, atol=3e-9)


def _synth_ant(angles, n_ant, spacing_frac):
    csi = np.zeros(n_ant, dtype=complex)
    idx = np.arange(n_ant)
    for a in angles:
        csi += np.exp(-1j * 2 * np.pi * spacing_frac * idx * np.sin(a))
    return csi


def test_recover_two_angles():
    true = np.array([-0.3, 0.5])            # radians
    csi = _synth_ant(true, n_ant=8, spacing_frac=0.5)
    est = np.sort(estimate_aoa(csi, spacing_frac=0.5, n_paths=2))
    assert np.allclose(est, np.sort(true), atol=0.05)
