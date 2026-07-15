"""1-D CA-CFAR on the delay profile — the SAME detector family as radar.processing.cfar_2d.

Comparability is the whole point of the hardware experiment: the phantom rate is only
comparable to paper 2's ~89% and paper 3's 0.1%/18.2% if the detector is the same. So this
mirrors ``cfar_2d`` exactly — the identical threshold multiplier

    alpha = N * (Pfa**(-1/N) - 1)

which holds the false-alarm rate constant regardless of noise level. A tuned threshold
would make the phantom rate meaningless.

The guard band must span the main lobe, or a tap masks itself in its own training cells
(the exact bug we hit in the 2-D chain). With a zero-padded profile one native cell spans
``zero_pad`` bins and the Hann main lobe is a couple of cells wide, so guard/train are
specified in NATIVE CELLS and scaled by ``zero_pad`` internally.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from .config import CSIConfig


def cfar_1d(
    power: np.ndarray,
    *,
    pfa: float = 1e-6,
    guard: int = 32,
    train: int = 48,
) -> np.ndarray:
    """Boolean detection mask over a 1-D POWER profile. ``guard``/``train`` are in BINS.

    The delay axis is NOT periodic (bin 0 and bin n are unrelated), so the boundary mode is
    'nearest' — matching the RANGE axis of ``cfar_2d`` (only its azimuth axis wraps).
    """
    full = 2 * (guard + train) + 1
    guard_win = 2 * guard + 1
    n_train = full - guard_win
    if n_train <= 0:
        raise ValueError("CFAR training region is empty; increase train")

    sum_full = ndimage.uniform_filter1d(power, size=full, mode="nearest") * full
    sum_guard = ndimage.uniform_filter1d(power, size=guard_win, mode="nearest") * guard_win
    noise = (sum_full - sum_guard) / n_train

    alpha = n_train * (pfa ** (-1.0 / n_train) - 1.0)
    return power > alpha * noise


def detect_delays(
    profile: np.ndarray,
    cfg: CSIConfig,
    *,
    zero_pad: int = 16,
    pfa: float = 1e-6,
    guard_cells: float = 2.0,
    train_cells: float = 3.0,
    max_path_m: float = 120.0,
) -> np.ndarray:
    """CFAR the profile, collapse each blob to its centroid, return EXCESS delays (seconds).

    Delays are measured from bin 0, which after LOS alignment IS the LOS — so a returned
    value is the tap's excess delay over LOS. Detections beyond ``max_path_m`` of path
    length are dropped: they are IFFT edge wraparound (the negative-delay half of the
    circular CIR), not physical echoes.

    ``guard_cells``/``train_cells`` are in native delay cells and scaled by ``zero_pad``.
    One physical tap lights up several adjacent bins, so blobs are merged before counting —
    exactly as ``cluster_detections`` does in the 2-D chain, and for the same reason.
    """
    power = np.abs(profile) ** 2
    guard = max(1, int(round(guard_cells * zero_pad)))
    train = max(1, int(round(train_cells * zero_pad)))
    mask = cfar_1d(power, pfa=pfa, guard=guard, train=train)

    grid = cfg.delay_grid_s(zero_pad)
    path = grid * cfg.path_cell_m / cfg.delay_resolution_s  # = C * grid, path length (m)
    mask &= path <= max_path_m                              # gate out the aliased half

    if not mask.any():
        return np.empty(0)
    labels, n = ndimage.label(mask)
    cent = ndimage.center_of_mass(np.where(mask, power, 0.0), labels, np.arange(1, n + 1))
    bin_idx = np.array([c[0] for c in cent])
    return np.interp(bin_idx, np.arange(grid.size), grid)


def resolved(profile: np.ndarray, bin1: int, bin2: int, dip_ratio: float = 0.7) -> bool:
    """Are two peaks at ``bin1``/``bin2`` resolved — i.e. is there a real dip between them?

    Two taps are 'resolved' when the minimum of the profile between the peaks falls below
    ``dip_ratio`` times the smaller peak. This measures RESOLUTION directly (Part 4.4),
    without CFAR — which cannot be used on a noiseless profile because it then fires on the
    Hann sidelobes. A pure resolution test wants no noise, so this dip metric is the right
    tool.
    """
    lo, hi = sorted((int(bin1), int(bin2)))
    if hi - lo < 2:
        return False
    valley = profile[lo:hi + 1].min()
    peak = min(profile[lo], profile[hi])
    return valley < dip_ratio * peak
