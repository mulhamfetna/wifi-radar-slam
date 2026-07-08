from __future__ import annotations
import numpy as np


def _music(block: np.ndarray, steering, grid: np.ndarray, n_sources: int) -> np.ndarray:
    """Multi-snapshot 1-D MUSIC with forward spatial smoothing.

    `block` is (n_snapshots, n_elements); each row is one snapshot over the array
    dimension being scanned (subcarriers for delays, antennas for AoA). Using real
    snapshots (the other CSI dimension) instead of collapsing it gives a far
    better-conditioned covariance than a single averaged snapshot. `steering(param,
    L) -> (L,)` builds the subarray steering vector.
    """
    block = np.atleast_2d(block)
    n = block.shape[1]
    L = max(n_sources + 1, (2 * n) // 3)          # subarray length
    R = np.zeros((L, L), dtype=complex)
    n_sub = n - L + 1
    for snap in block:                             # accumulate smoothed covariance
        subs = np.stack([snap[i:i + L] for i in range(n_sub)], axis=1)  # (L, n_sub)
        R += subs @ subs.conj().T
    R /= (block.shape[0] * n_sub)

    _, evecs = np.linalg.eigh(R)                    # ascending eigenvalues
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


C = 299792458.0


def estimate_delays(block: np.ndarray, bandwidth_hz: float, n_paths: int,
                    max_range_m: float | None = None) -> np.ndarray:
    """Delays from the frequency-domain CSI via MUSIC.

    `block` is (n_antennas, n_subcarriers) — antennas are used as snapshots — or a
    single (n_subcarriers,) vector. `max_range_m` bounds the delay grid to a
    physical range; leaving it None keeps the full unambiguous span (used by the
    convention unit tests). Bounding it is essential on real CSI, where an
    unbounded grid places spurious peaks at the aliasing edge.
    """
    block = np.atleast_2d(block)
    n = block.shape[1]
    df = bandwidth_hz / n
    hi = (n - 1) / bandwidth_hz if max_range_m is None else max_range_m / C
    grid = np.linspace(0.0, hi, 3000)

    def steering(tau, L):
        k = np.arange(L)
        return np.exp(-1j * 2 * np.pi * (k * df) * tau)

    return _music(block, steering, grid, n_paths)


def estimate_aoa(block: np.ndarray, spacing_frac: float, n_paths: int) -> np.ndarray:
    """Electrical (array-relative) angles from the spatial CSI via MUSIC.

    `block` is (n_subcarriers, n_antennas) — subcarriers are used as snapshots — or
    a single (n_antennas,) vector. Returns the electrical angle theta of the
    steering `exp(-j 2 pi spacing_frac k sin(theta))`; convert to a world-frame
    azimuth with `azimuth_from_electrical`.
    """
    grid = np.linspace(-np.pi / 2, np.pi / 2, 4000)

    def steering(theta, L):
        idx = np.arange(L)
        return np.exp(-1j * 2 * np.pi * spacing_frac * idx * np.sin(theta))

    return _music(block, steering, grid, n_paths)


def _peaks_2d(spectrum: np.ndarray, k: int) -> list[tuple[int, int]]:
    """Top-k 2-D local maxima (row, col) of a spectrum, strongest first."""
    s = spectrum
    interior = np.ones_like(s, dtype=bool)
    interior[0, :] = interior[-1, :] = interior[:, 0] = interior[:, -1] = False
    is_max = interior.copy()
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            is_max &= s >= np.roll(np.roll(s, dr, 0), dc, 1)
    rc = np.argwhere(is_max)
    if len(rc) < k:                                    # fall back to global top-k
        flat = np.argsort(s, axis=None)[::-1][:k]
        return [tuple(np.unravel_index(i, s.shape)) for i in flat]
    order = rc[np.argsort(s[rc[:, 0], rc[:, 1]])[::-1]]
    return [tuple(p) for p in order[:k]]


def estimate_joint(block: np.ndarray, bandwidth_hz: float, spacing_frac: float,
                   n_paths: int, max_range_m: float = 150.0) -> np.ndarray:
    """Joint (delay, angle) estimation via 2-D MUSIC with 2-D spatial smoothing.

    `block` is (n_antennas, n_subcarriers). Each path contributes a separable
    delay-over-subcarriers x angle-over-antennas signature, so a single 2-D MUSIC
    scan recovers each path's delay AND electrical angle *together* — the
    association is intrinsic, unlike separate 1-D delay/AoA estimates paired by
    sorted index. Returns (n_paths, 2): columns [delay_s, electrical_angle_rad].
    """
    block = np.asarray(block)
    n_ant, n_sub = block.shape
    df = bandwidth_hz / n_sub
    La = max(2, n_ant - 1)                              # antenna subarray length
    Lf = max(n_paths + 1, min(n_sub // 4, (2 * n_sub) // 3))  # subcarrier subarray length
    dim = La * Lf

    # 2-D forward spatial smoothing -> many vec'd subarray snapshots. Spatial
    # smoothing decorrelates the (coherent, single-time-snapshot) paths, so the
    # snapshot matrix has rank = n_paths and its right singular vectors give the
    # signal/noise split directly (numerically cleaner than eigendecomposing SᴴS).
    snaps = []
    for a0 in range(n_ant - La + 1):
        for f0 in range(n_sub - Lf + 1):
            snaps.append(block[a0:a0 + La, f0:f0 + Lf].ravel())   # row-major: i*Lf+j
    S = np.array(snaps)                                  # (n_snap, dim)
    _, _, Vh = np.linalg.svd(S, full_matrices=True)     # rows of Vh = right sing. vectors
    noise = Vh[n_paths:].conj().T                       # (dim, n_noise)

    # bound the delay grid to the unambiguous range 1/df (delays beyond it alias
    # back to near zero and would be picked as spurious peaks)
    hi = min(max_range_m / C, 0.95 / df)
    tau_grid = np.linspace(0.0, hi, 300)
    th_grid = np.linspace(-np.pi / 2, np.pi / 2, 120)
    a_delay = np.exp(-1j * 2 * np.pi * np.outer(tau_grid, np.arange(Lf) * df))   # (nt, Lf)
    a_ant = np.exp(-1j * 2 * np.pi * spacing_frac * np.outer(np.sin(th_grid), np.arange(La)))  # (na, La)
    # joint steering for every (theta, tau): kron(a_ant, a_delay) -> (na, nt, dim)
    A = (a_ant[:, None, :, None] * a_delay[None, :, None, :]).reshape(
        th_grid.size, tau_grid.size, dim)
    proj = A.reshape(-1, dim) @ noise                   # (na*nt, n_noise)
    power = 1.0 / (np.sum(np.abs(proj) ** 2, axis=1) + 1e-12)
    spectrum = power.reshape(th_grid.size, tau_grid.size)

    out = []
    for ti, tj in _peaks_2d(spectrum, n_paths):
        out.append([tau_grid[tj], th_grid[ti]])
    return np.array(out)


def azimuth_from_electrical(theta: np.ndarray) -> np.ndarray:
    """Map the array-relative electrical angle to a world-frame azimuth.

    The vehicle carries a horizontal ULA whose axis is world +y (the receiver is
    not rotated along the straight trajectory). Empirically, Sionna's antenna phase
    gives `sin(theta) = -sin(beta)` where beta = atan2(dy, dx) is the world bearing
    to the source. Inverting, `beta = arcsin(-sin(theta))` — the forward (dx>0)
    branch. A single ULA cannot resolve the dx sign (front/back), so this returns
    the forward branch; the SLAM triangulation guards reject the back-branch cases
    (direct/behind paths) via the s<=0 range test.
    """
    return np.arcsin(np.clip(-np.sin(np.asarray(theta)), -1.0, 1.0))
