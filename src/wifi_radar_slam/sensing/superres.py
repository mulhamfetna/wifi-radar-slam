from __future__ import annotations
import numpy as np


def _covariance(samples: np.ndarray, subarray_len: int) -> np.ndarray:
    """Spatially-smoothed covariance from a single snapshot via forward subarrays."""
    n = samples.shape[0]
    L = subarray_len
    subs = np.stack([samples[i:i + L] for i in range(n - L + 1)], axis=1)  # (L, K)
    return subs @ subs.conj().T / subs.shape[1]


def _music_1d(samples: np.ndarray, steering, grid: np.ndarray, n_sources: int) -> np.ndarray:
    """Generic 1-D MUSIC. `steering(param, L) -> (L,) complex` array."""
    n = samples.shape[0]
    L = max(n_sources + 1, (2 * n) // 3)          # subarray length
    R = _covariance(samples, L)
    _, evecs = np.linalg.eigh(R)                   # ascending eigenvalues
    noise = evecs[:, : L - n_sources]              # smallest eigenvectors span noise
    spectrum = np.empty(grid.shape[0])
    for i, g in enumerate(grid):
        a = steering(g, L)
        spectrum[i] = 1.0 / (np.linalg.norm(noise.conj().T @ a) ** 2 + 1e-12)
    return grid[_pick_peaks(spectrum, n_sources)]


def _pick_peaks(spectrum: np.ndarray, k: int) -> np.ndarray:
    interior = np.where((spectrum[1:-1] > spectrum[:-2]) &
                        (spectrum[1:-1] > spectrum[2:]))[0] + 1
    if interior.size < k:
        return np.argsort(spectrum)[-k:]
    order = interior[np.argsort(spectrum[interior])[::-1]]
    return order[:k]


def estimate_delays(csi_freq: np.ndarray, bandwidth_hz: float, n_paths: int) -> np.ndarray:
    n = csi_freq.shape[0]
    df = bandwidth_hz / n
    grid = np.linspace(0.0, (n - 1) / bandwidth_hz, 4000)   # delay grid up to ~1/df

    def steering(tau, L):
        k = np.arange(L)
        return np.exp(-1j * 2 * np.pi * (k * df) * tau)

    return _music_1d(csi_freq, steering, grid, n_paths)


def estimate_aoa(csi_ant: np.ndarray, spacing_frac: float, n_paths: int) -> np.ndarray:
    grid = np.linspace(-np.pi / 2, np.pi / 2, 4000)

    def steering(theta, L):
        idx = np.arange(L)
        return np.exp(-1j * 2 * np.pi * spacing_frac * idx * np.sin(theta))

    return _music_1d(csi_ant, steering, grid, n_paths)
