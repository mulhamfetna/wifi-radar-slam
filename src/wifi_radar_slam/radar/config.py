from __future__ import annotations
from dataclasses import dataclass
import numpy as np

C = 299792458.0


@dataclass(frozen=True)
class RadarConfig:
    """FMCW radar parameters for the range-azimuth detection chain (BEV plane).

    One config type serves every ablation cell: a cell is just a (carrier, bandwidth)
    pair on a fixed detection chain, which is exactly what makes the ablation clean.

    `n_chirps` is the **coherent-integration factor**, not a processed axis. The scenes
    are static, so all chirps in the CPI carry the same signal and differ only in noise;
    integrating them coherently is analytically identical to scaling the noise sigma by
    1/sqrt(n_chirps), which is what `processing.beat_matrix` does.
    """
    carrier_hz: float          # f_c
    bandwidth_hz: float        # B, the sweep bandwidth -- sets range resolution
    chirp_time_s: float        # T_c, sweep duration
    n_samples: int             # ADC samples per chirp (fast time) -> range bins
    n_chirps: int              # chirps per CPI == coherent-integration factor
    n_rx: int                  # (virtual) ULA elements
    rx_spacing_frac: float     # element spacing in wavelengths (0.5 = half-wavelength)
    n_azimuth: int             # beamforming grid size
    fov_deg: float             # total azimuth field of view
    max_range_m: float         # detections beyond this are discarded
    min_range_m: float         # blind zone: TX/RX are co-located, so near returns are self-clutter
    cfar_guard_range: int      # CA-CFAR guard cells, range axis
    cfar_train_range: int      # CA-CFAR training cells, range axis
    cfar_guard_azimuth: int    # CA-CFAR guard cells, azimuth axis
    cfar_train_azimuth: int    # CA-CFAR training cells, azimuth axis
    pfa: float                 # CFAR design probability of false alarm (per cell)
    noise_sigma: float         # per-sample complex-noise std BEFORE coherent integration

    def __post_init__(self) -> None:
        """Reject a radar that cannot sample its own maximum range.

        A target at max_range_m beats at f_b = 2*R*S/c. If that exceeds the kept FFT
        span, the target ALIASES: it folds back and appears at a short range where
        nothing exists. That is a phantom detection manufactured by our own ADC, and
        since the phantom rate is this paper's headline measurement, a config that does
        it is not a tuning choice -- it is a bug. Fail loudly at construction.

        The invariant reduces to  n_samples >= 4 * max_range_m * bandwidth_hz / c
        (chirp_time_s cancels), so the ADC sample COUNT is forced by range x bandwidth
        alone. This is the real hardware trade-off behind cell D: 4 GHz of sweep and
        100 m of range together demand ~5300 samples, which is why the presets carry
        8192 rather than a token 256.
        """
        if self.max_beat_range_m < self.max_range_m:
            need = int(np.ceil(4.0 * self.max_range_m * self.bandwidth_hz / C))
            raise ValueError(
                f"radar aliases its own max range: max_beat_range_m="
                f"{self.max_beat_range_m:.1f} m < max_range_m={self.max_range_m:.1f} m. "
                f"With B={self.bandwidth_hz/1e9:g} GHz need n_samples >= {need}, "
                f"have {self.n_samples}.")

    # --- derived RF quantities -------------------------------------------------
    @property
    def wavelength_m(self) -> float:
        return C / self.carrier_hz

    @property
    def sweep_slope_hz_per_s(self) -> float:
        """S = B / T_c. A target at delay tau beats at f_b = S * tau."""
        return self.bandwidth_hz / self.chirp_time_s

    @property
    def sample_rate_hz(self) -> float:
        return self.n_samples / self.chirp_time_s

    @property
    def range_resolution_m(self) -> float:
        """c / 2B -- set by bandwidth alone. This is the ablation's C->D axis."""
        return C / (2.0 * self.bandwidth_hz)

    @property
    def n_range(self) -> int:
        """Range bins kept: the positive half of the FFT."""
        return self.n_samples // 2

    @property
    def max_beat_range_m(self) -> float:
        """Range whose beat frequency sits at the top kept FFT bin.

        f_b = 2*R*S/c, and the highest kept bin is (n_range-1)*fs/n_samples, so
        R_max = (n_range-1) * fs * c / (n_samples * 2 * S). Anything past this aliases.
        """
        f_b_max = (self.n_range - 1) * self.sample_rate_hz / self.n_samples
        return f_b_max * C / (2.0 * self.sweep_slope_hz_per_s)

    # --- grids -----------------------------------------------------------------
    def range_bins(self) -> np.ndarray:
        """Range (m) of each kept FFT bin. Spacing == range_resolution_m exactly."""
        k = np.arange(self.n_range)
        f_b = k * self.sample_rate_hz / self.n_samples
        return f_b * C / (2.0 * self.sweep_slope_hz_per_s)

    def azimuth_grid(self) -> np.ndarray:
        """Beamforming steering angles (rad), spanning the FOV symmetrically about 0
        (local +x forward, positive toward +y)."""
        half = np.deg2rad(self.fov_deg) / 2.0
        return np.linspace(-half, half, self.n_azimuth)


# --- the ablation cells ---------------------------------------------------------
# EVERY field below is shared by every cell. Only carrier_hz and bandwidth_hz change --
# that is the ablation, and holding the rest fixed is what makes B->C attributable to
# carrier and C->D to bandwidth rather than to some incidental difference in the chain.
#
# The ADC numbers are forced, not chosen (see __post_init__): 100 m of range at cell D's
# 4 GHz sweep needs n_samples >= 5337, so 8192. A 200 us chirp then puts the sample rate
# at 41 MHz, which is what a high-end automotive ADC actually delivers -- a 40 us chirp
# would demand 205 MHz, which no such part does. 64 chirps give a 12.8 ms CPI, comfortably
# inside one 50 ms simulation frame.
#
# Cells B and C use only the bottom of that same spectrum (their beat frequencies are 25x
# lower), which is exactly right: it is one ADC, one chirp duration, one detection chain --
# only the sweep differs. That is the physical situation the ablation is meant to model.
_COMMON = dict(chirp_time_s=200e-6, n_samples=8192, n_chirps=64, n_rx=16,
               rx_spacing_frac=0.5, n_azimuth=181, fov_deg=180.0,
               max_range_m=100.0, min_range_m=1.0,
               cfar_guard_range=2, cfar_train_range=8,
               cfar_guard_azimuth=2, cfar_train_azimuth=4,
               pfa=1e-4, noise_sigma=1e-3)

# Cell D: full-bandwidth 77 GHz automotive radar. 4 GHz -> 3.75 cm range resolution.
RADAR_77G_4G = RadarConfig(carrier_hz=77e9, bandwidth_hz=4e9, **_COMMON)

# Cell C: the SAME radar crippled to WiFi's bandwidth. Isolates what the carrier buys.
RADAR_77G_160M = RadarConfig(carrier_hz=77e9, bandwidth_hz=160e6, **_COMMON)

# Cell B: an active monostatic WiFi radar (5.2 GHz, 160 MHz). Isolates bistatic geometry.
WIFI_5G2_160M = RadarConfig(carrier_hz=5.2e9, bandwidth_hz=160e6, **_COMMON)
