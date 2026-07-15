"""ESP32 HT40 CSI physical constants — every number verified, every derivation shown.

Sources (see ``docs/paper4-restart-static-bench.md`` Part 4 and Part 7 for the [V] tags):
  - HT40 reports 128 HT-LTF subcarriers on a uniform 312.5 kHz grid (ESP-IDF wifi.rst).
  - 114 non-null of 128; the DC notch is 3 subcarriers wide (ESPARGOS HT40_GAP_SUBCARRIERS=3).
  - 11 edge guard bins (|k| >= 59) set the sidelobe skirt — window before the IFFT.
  - Delay resolution 1/B = 25 ns -> one cell = 7.5 m of PATH = 3.75 m of monostatic RANGE.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

C = 299792458.0  # m/s — the same constant radar/config.py uses


@dataclass(frozen=True)
class CSIConfig:
    """Physical layout of one ESP32 HT40 CSI vector.

    Indexing convention: subcarrier index ``k`` runs over ``arange(-64, 64)`` (an
    ``fftshift`` of the raw 0..63,-64..-1 order — see Part 7.2). The active band is
    ``|k| <= 57``; ``k`` in ``{-1, 0, +1}`` is the dead DC notch; ``|k| >= 59`` are the
    guard bins. Frequency of bin k = carrier + k * subcarrier_spacing_hz.
    """

    n_subcarriers: int = 128           # HT-LTF bins reported by ESP32 in HT40
    subcarrier_spacing_hz: float = 312_500.0
    carrier_hz: float = 2.412e9        # 2.4 GHz channel 1 centre (carrier does not matter — paper 3)

    # The dead / untrusted bins, in fftshifted index space (k in [-64, 63]).
    dc_notch_k: tuple[int, ...] = (-1, 0, 1)          # 3-bin centre notch (2.6% of band)
    guard_abs_k_min: int = 59                          # |k| >= 59 are edge guard bins

    def __post_init__(self) -> None:
        if self.n_subcarriers % 2 != 0:
            raise ValueError("n_subcarriers must be even (fftshift symmetry)")

    # ---- the frequency grid -------------------------------------------------
    @property
    def k_grid(self) -> np.ndarray:
        """Subcarrier indices, fftshifted: arange(-n/2, n/2)."""
        n = self.n_subcarriers
        return np.arange(-n // 2, n // 2)

    @property
    def bandwidth_hz(self) -> float:
        """Occupied bandwidth = n_subcarriers * spacing (128 * 312.5 kHz = 40 MHz)."""
        return self.n_subcarriers * self.subcarrier_spacing_hz

    @property
    def active_mask(self) -> np.ndarray:
        """True where the subcarrier carries usable signal (not notch, not guard)."""
        k = self.k_grid
        notch = np.isin(k, np.array(self.dc_notch_k))
        guard = np.abs(k) >= self.guard_abs_k_min
        return ~(notch | guard)

    @property
    def dead_mask(self) -> np.ndarray:
        """True where the subcarrier is dead/untrusted (notch or guard)."""
        return ~self.active_mask

    # ---- the delay grid -----------------------------------------------------
    @property
    def delay_resolution_s(self) -> float:
        """1/B — the width of one native delay cell. 25 ns at 40 MHz."""
        return 1.0 / self.bandwidth_hz

    @property
    def path_cell_m(self) -> float:
        """One delay cell in PATH length: c/B. 7.5 m at 40 MHz."""
        return C * self.delay_resolution_s

    @property
    def range_cell_m(self) -> float:
        """One delay cell in monostatic RANGE: c/2B. 3.75 m at 40 MHz."""
        return 0.5 * self.path_cell_m

    def delay_grid_s(self, zero_pad: int = 1) -> np.ndarray:
        """Delay axis of the IFFT output, in seconds.

        With ``zero_pad`` the IFFT length is ``n_subcarriers * zero_pad`` and the delay
        step shrinks to ``1 / (B * zero_pad)`` — finer sampling of the SAME resolution.
        Sampling is not resolution (Part 4.4): zero-padding interpolates, it does not
        separate targets any better.
        """
        n = self.n_subcarriers * zero_pad
        step = 1.0 / (self.bandwidth_hz * zero_pad)
        return np.arange(n) * step

    def path_grid_m(self, zero_pad: int = 1) -> np.ndarray:
        """Delay axis expressed as PATH length (c * tau)."""
        return C * self.delay_grid_s(zero_pad)

    # ---- geometry helpers for the bench -------------------------------------
    def excess_delay_s(self, plate_distance_m: float, baseline_m: float) -> float:
        """Excess delay of a plate echo over the direct LOS path.

        Path = |TX->plate| + |plate->RX| ~= 2d; direct = baseline b. Excess = (2d - b)/c.
        At d=12 m, b=0.5 m -> 78 ns -> 3.13 cells (the recommended Rung-1 start point).
        """
        excess_path = 2.0 * plate_distance_m - baseline_m
        return excess_path / C


# The one preset: a bench ESP32-S3 on 2.4 GHz channel 1, HT40.
ESP32_HT40 = CSIConfig()
