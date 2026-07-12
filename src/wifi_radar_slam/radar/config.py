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

        # The CFAR guard band must span the main lobe. If it does not, a target's own
        # energy lands in its own TRAINING cells, inflating its own noise estimate and
        # punching holes in its own detection -- one reflector then fragments into several,
        # every detection count is wrong, and the phantom rate is wrong with it. We hit
        # exactly this in development: a guard band narrower than the beam split each
        # target into three blobs.
        guard_u = self.cfar_guard_azimuth * self.u_step
        if guard_u < self.beamwidth_u / 2.0:
            need = int(np.ceil((self.beamwidth_u / 2.0) / self.u_step))
            raise ValueError(
                f"CFAR guard band ({guard_u:.3f} in u) is narrower than half the "
                f"{self.beamwidth_u:.3f} beam: targets would mask themselves. Need "
                f"cfar_guard_azimuth >= {need}, have {self.cfar_guard_azimuth}.")

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
    def u_step(self) -> float:
        """Spacing between adjacent beamforming bins in u = sin(azimuth)."""
        return 2.0 * np.sin(np.deg2rad(self.fov_deg) / 2.0) / (self.n_azimuth - 1)

    @property
    def beamwidth_u(self) -> float:
        """Approximate 3 dB beamwidth of the TAPERED array, in u = sin(azimuth).

        A ULA's beam is invariant in u, NOT in angle: the array factor depends on the
        elements only through u = sin(theta). Its width in u is ~1.6/(N * rx_spacing_frac)
        for a tapered aperture, everywhere across the field of view. In *angle* the same
        beam widens by 1/cos(theta) off boresight and diverges at endfire -- which is why
        the beamforming grid is uniform in u and the CFAR guard band is specified in u.
        (Checked against the real array factor: predicts 0.20 in u for the 16-element
        preset, i.e. 11.5 deg at boresight; measured 11.8 deg.)
        """
        return 1.6 / (self.n_rx * self.rx_spacing_frac)

    @property
    def beamwidth_boresight_rad(self) -> float:
        """The 3 dB beamwidth at boresight, in radians (for reporting only)."""
        return 2.0 * float(np.arcsin(min(1.0, self.beamwidth_u / 2.0)))

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

    def u_grid(self) -> np.ndarray:
        """Beamforming grid in u = sin(azimuth), uniformly spaced.

        Uniform in u, not in angle, because that is the coordinate the array actually
        works in: the beam has constant width in u across the whole field of view. A grid
        uniform in *angle* would under-sample the beam near boresight and over-sample it
        near endfire, and -- worse -- no fixed CFAR guard band could cover a beam whose
        angular width diverges as 1/cos(theta).
        """
        half = np.sin(np.deg2rad(self.fov_deg) / 2.0)
        return np.linspace(-half, half, self.n_azimuth)

    def azimuth_grid(self) -> np.ndarray:
        """Beamforming steering angles (rad), local +x forward, positive toward +y.

        These are arcsin of the uniform u grid, so they are DENSER near boresight and
        sparser toward endfire -- which is exactly right: a ULA's angular resolution
        genuinely degrades off boresight, and the grid should reflect that rather than
        pretend otherwise.
        """
        return np.arcsin(self.u_grid())


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
#
# Pfa is sized to the MAP, not picked by habit. The range-azimuth map is
# 181 x 4096 = 741,376 cells, so the expected number of noise-only false alarms per frame
# is Pfa x 741,376. At a textbook-looking Pfa of 1e-4 that is 74 phantom detections every
# single frame, from thermal noise alone -- they would swamp the map and, worse, they would
# be counted in the phantom rate that RQ1 reports. At 1e-6 it is 0.74 per frame, which is
# the right order for an automotive detector. Real radars use low Pfa for exactly this
# reason: the cell count is enormous.
#
# CFAR guard/training cells are sized to the beam, not guessed. The tapered 16-element
# array has a 0.20-wide beam in u = sin(azimuth) and the grid is 0.011/bin, so the guard
# band needs >= 9 bins to cover half the main lobe; 10 gives margin. (__post_init__ enforces
# this -- too narrow a guard band makes a target mask itself and fragment into several
# detections, which we hit in development.)
_COMMON = dict(chirp_time_s=200e-6, n_samples=8192, n_chirps=64, n_rx=16,
               rx_spacing_frac=0.5, n_azimuth=181, fov_deg=180.0,
               max_range_m=100.0, min_range_m=1.0,
               cfar_guard_range=4, cfar_train_range=12,
               cfar_guard_azimuth=10, cfar_train_azimuth=10,
               pfa=1e-6, noise_sigma=1e-3)

# Cell D: full-bandwidth 77 GHz automotive radar. 4 GHz -> 3.75 cm range resolution.
RADAR_77G_4G = RadarConfig(carrier_hz=77e9, bandwidth_hz=4e9, **_COMMON)

# Cell C: the SAME radar crippled to WiFi's bandwidth. Isolates what the carrier buys.
RADAR_77G_160M = RadarConfig(carrier_hz=77e9, bandwidth_hz=160e6, **_COMMON)

# Cell B: an active monostatic WiFi radar (5.2 GHz, 160 MHz). Isolates bistatic geometry.
WIFI_5G2_160M = RadarConfig(carrier_hz=5.2e9, bandwidth_hz=160e6, **_COMMON)
