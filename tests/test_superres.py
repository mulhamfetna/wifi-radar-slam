import numpy as np
from wifi_radar_slam.sensing.superres import (
    estimate_delays, estimate_aoa, azimuth_from_electrical,
)


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


def test_recover_delays_multisnapshot_bounded_grid():
    """Delays from a multi-antenna block (antennas as snapshots) with a bounded
    grid — mirrors the real frontend path, where each antenna carries the same
    delays under a different AoA phase."""
    bw, n_sub, n_ant = 160e6, 128, 4
    freqs = np.linspace(-bw / 2, bw / 2, n_sub)
    true = np.array([20e-9, 60e-9])         # ~6 m, ~18 m
    idx = np.arange(n_ant)
    block = np.zeros((n_ant, n_sub), dtype=complex)
    for ai in idx:                          # per-antenna AoA phase + shared delays
        col = np.zeros(n_sub, dtype=complex)
        for j, tau in enumerate(true):
            ant_phase = np.exp(-1j * 2 * np.pi * 0.5 * ai * np.sin(0.2 * (j + 1)))
            col += ant_phase * np.exp(-1j * 2 * np.pi * freqs * tau)
        block[ai] = col
    est = np.sort(estimate_delays(block, bw, n_paths=2, max_range_m=150.0))
    assert np.allclose(est, true, atol=3e-9)


def test_azimuth_from_electrical():
    # forward branch: beta = arcsin(-sin(theta)) = -theta for |theta| < pi/2
    for th in (-0.7, -0.2, 0.0, 0.4, 0.9):
        assert np.isclose(azimuth_from_electrical(th), -th, atol=1e-9)
    # vectorized + clipped, stays in [-pi/2, pi/2]
    out = azimuth_from_electrical(np.array([-1.5, 1.5]))
    assert np.all(np.abs(out) <= np.pi / 2 + 1e-9)
