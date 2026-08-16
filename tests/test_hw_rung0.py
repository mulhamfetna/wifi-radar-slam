"""Rung 0 — the pipeline recovers a tap we injected, through every documented corruption.

These tests ARE the rung-0 acceptance criteria from docs/paper4-restart-static-bench.md
Part 9. If they pass, the Python side is sound and the failure mode of the real experiment
is physics, not our code. No hardware is involved.

After LOS-peak alignment the LOS sits at delay 0, so a detected delay IS the tap's excess
delay over LOS — exactly the quantity the excess-delay method needs (Part 5).
"""
import numpy as np
import pytest

from wifi_radar_slam.hw import ESP32_HT40 as CFG
from wifi_radar_slam.hw.config import C
from wifi_radar_slam.hw.delay import (
    coherent_average,
    delay_profile,
    differential,
    prepare_packet,
    raw_cir,
)
from wifi_radar_slam.hw.detect import cfar_1d, detect_delays, resolved
from wifi_radar_slam.hw.synth import (
    Tap,
    ideal_csi,
    receiver_response,
    synth_recording,
)

ZP = 16  # zero-pad factor used across the pipeline


# --------------------------------------------------------------------------- config
def test_config_derived_numbers():
    """The load-bearing physics constants match the design doc exactly."""
    assert CFG.bandwidth_hz == pytest.approx(40e6)
    assert CFG.delay_resolution_s == pytest.approx(25e-9)
    # exact c/B = 7.4948 m; the doc rounds to 7.5 with c ~ 3e8.
    assert CFG.path_cell_m == pytest.approx(7.4948, abs=1e-3)
    assert CFG.range_cell_m == pytest.approx(3.7474, abs=1e-3)
    # 128 bins, minus 3 notch, minus 11 guard (|k|>=59: -64..-59 = 6, 59..63 = 5) = 114.
    assert CFG.active_mask.sum() == 114


def test_excess_delay_geometry():
    """Plate at 12 m, baseline 0.5 m -> 78 ns -> 3.13 cells (the Rung-1 start point)."""
    tau = CFG.excess_delay_s(12.0, 0.5)
    assert tau == pytest.approx(78e-9, abs=1e-9)
    assert tau / CFG.delay_resolution_s == pytest.approx(3.13, abs=0.02)


# --------------------------------------------------------------------------- ordering check (Rung 0.5)
def test_empty_corridor_shows_one_tap():
    """Rung 0.5: an empty corridor (LOS only) must show exactly ONE dominant tap.

    Two peaks or a negative-delay peak would mean the subcarrier ordering is wrong. Here the
    ordering is correct by construction, so we assert a single dominant tap at delay 0."""
    h = ideal_csi([Tap(1.0, 0.0)], CFG)
    prof = np.abs(raw_cir(h, CFG, ZP))
    peak = prof.argmax()
    # dominant peak is at (circular) delay 0. Exclude +/-3 path cells around 0 and the end
    # (the Hann main lobe is ~2 cells wide); nothing outside it may rise above 30% of peak.
    g = CFG.path_grid_m(ZP)
    assert g[peak] < CFG.path_cell_m or g[peak] > g[-1] - CFG.path_cell_m
    excl = 3.0 * CFG.path_cell_m
    far = prof[(g > excl) & (g < g[-1] - excl)]
    assert far.max() < 0.3 * prof.max()


# --------------------------------------------------------------------------- referencing
def test_prepare_packet_kills_common_phase_and_sto():
    """Two packets of the same channel, differing only in random phase + STO, prepare to the
    same aligned CIR."""
    rng = np.random.default_rng(3)
    taps = [Tap(1.0, 0.0), Tap(0.5, 90e-9)]
    rec = synth_recording(taps, CFG, n_packets=2, rng=rng, sto_std_s=5e-9,
                          noise_std=0.0, quantise=False)
    a = np.abs(prepare_packet(rec[0], CFG, ZP))
    b = np.abs(prepare_packet(rec[1], CFG, ZP))
    # Integer-bin LOS alignment leaves a sub-bin STO residual per packet (washed out by
    # averaging many packets); two single packets still agree to > 0.95.
    assert np.corrcoef(a, b)[0, 1] > 0.95


def test_coherent_averaging_suppresses_random_noise():
    """Coherent averaging suppresses the RANDOM noise as ~1/sqrt(N).

    The absolute floor of a profile is dominated by the LOS's own (coherent) sidelobes,
    which do NOT average down — so the honest measure of processing gain is the deviation
    from the noiseless reference, which is pure noise. That deviation must shrink with N.
    """
    rng = np.random.default_rng(0)
    taps = [Tap(1.0, 0.0)]
    ref = np.abs(coherent_average(
        synth_recording(taps, CFG, n_packets=4, rng=np.random.default_rng(0),
                        sto_std_s=0.0, noise_std=0.0, quantise=False), CFG, ZP))

    kw = dict(rng=rng, sto_std_s=0.0, noise_std=0.05, quantise=False)
    rec = synth_recording(taps, CFG, n_packets=200, **kw)
    g = CFG.path_grid_m(ZP)
    quiet = (g > 60) & (g < 120)

    def noise_rms(n):
        prof = np.abs(coherent_average(rec[:n], CFG, ZP))
        return np.sqrt(np.mean((prof[quiet] - ref[quiet]) ** 2))

    # 100 packets cut the noise RMS by clearly more than 2x versus a single packet.
    assert noise_rms(100) < 0.5 * noise_rms(1)


# --------------------------------------------------------------------------- THE atomic test
def test_rung0_recovers_injected_echo_through_full_stack():
    """THE rung-0 kill criterion: a plate echo we injected is recovered at its excess delay,
    through the notch, guard bins, S-shaped distortion, per-packet phase+STO, quantisation,
    and the coherent-averaging + differential pipeline."""
    rng = np.random.default_rng(42)
    d, b = 12.0, 0.5
    tau = CFG.excess_delay_s(d, b)

    los = Tap(1.0, 0.0)
    echo = Tap(0.15, tau)                      # weak echo on top of the LOS
    rx = receiver_response(CFG)                # ONE static response for both recordings

    kw = dict(rng=rng, rx_response=rx, sto_std_s=4e-9, noise_std=0.02, quantise=True)
    plate_in = synth_recording([los, echo], CFG, n_packets=500, **kw)
    plate_out = synth_recording([los], CFG, n_packets=500, **kw)

    prof = differential(plate_in, plate_out, CFG, ZP)
    delays = detect_delays(prof, CFG, zero_pad=ZP, pfa=1e-6)

    assert delays.size >= 1
    err = np.min(np.abs(delays - tau))
    assert err < 0.5 * CFG.delay_resolution_s        # within half a native cell

    # And the matched detection maps back to the true excess PATH (2d - b).
    best = delays[np.argmin(np.abs(delays - tau))]
    assert C * best == pytest.approx(2 * d - b, abs=CFG.path_cell_m)


def test_ranging_tracks_true_distance():
    """Rung-2 physics: a single isolated echo is located to well under one cell across the
    corridor range. Precision (not resolution) is SNR-limited, so one target interpolates
    tightly even at 40 MHz — this is exactly what makes the range-bias measurement possible.
    """
    from wifi_radar_slam.hw.delay import raw_cir
    g = CFG.path_grid_m(ZP)
    for d in (8.0, 10.0, 12.0, 14.0):
        tau = CFG.excess_delay_s(d, 0.5)
        prof = np.abs(raw_cir(ideal_csi([Tap(1.0, tau)], CFG), CFG, ZP))
        peak_path = g[prof.argmax()]
        assert peak_path == pytest.approx(2 * d - 0.5, abs=0.5)   # < 0.5 m, i.e. << one cell


def test_receiver_response_cancels_in_differential():
    """The static receiver distortion (the phantom generator) cancels in plate-in - plate-out,
    so the echo survives even with a strong, phantom-making response."""
    rng = np.random.default_rng(7)
    rx = receiver_response(CFG, strength=0.6)
    los = Tap(1.0, 0.0)
    tau = CFG.excess_delay_s(12.0, 0.5)
    echo = Tap(0.15, tau)

    kw = dict(rng=rng, rx_response=rx, sto_std_s=4e-9, noise_std=0.01, quantise=False)
    plate_in = synth_recording([los, echo], CFG, n_packets=200, **kw)
    plate_out = synth_recording([los], CFG, n_packets=200, **kw)

    prof = differential(plate_in, plate_out, CFG, ZP)
    g = CFG.path_grid_m(ZP)
    # The echo dominates the differential and lands at the true excess path (2d - b) --
    # the static receiver distortion has cancelled. (Prominence, not CFAR count: the echo's
    # own wide Hann lobe inflates its CFAR training cells, so we test cancellation directly.)
    peak_path = g[prof.argmax()]
    assert peak_path == pytest.approx(2 * 12.0 - 0.5, abs=CFG.path_cell_m)
    floor = np.median(prof[(g > 60) & (g < 120)])
    assert prof.max() > 4.0 * floor


# --------------------------------------------------------------------------- resolution
def test_two_taps_resolve_when_far_merge_when_close():
    """RESOLUTION (Part 4.4): two taps a few cells apart show a dip between them; taps well
    under a cell apart merge into one lobe. Tested with the dip metric (no CFAR) because a
    noiseless profile has no noise for CFAR to estimate."""
    def dip_present(sep_path_m):
        tau1 = 40e-9
        tau2 = tau1 + sep_path_m / C
        h = ideal_csi([Tap(1.0, tau1), Tap(1.0, tau2)], CFG)
        prof = np.abs(raw_cir(h, CFG, ZP))
        g = CFG.path_grid_m(ZP)
        b1 = int(np.argmin(np.abs(g - C * tau1)))
        b2 = int(np.argmin(np.abs(g - C * tau2)))
        return resolved(prof, b1, b2)

    assert dip_present(3.0 * CFG.path_cell_m)     # 3 cells apart -> resolved
    assert not dip_present(0.2 * CFG.path_cell_m)  # 0.2 cell apart -> merged


# --------------------------------------------------------------------------- CFAR sanity
def test_cfar_1d_constant_false_alarm_on_pure_noise():
    """On pure noise the CFAR fires at roughly its design Pfa, not a tuned flood."""
    rng = np.random.default_rng(1)
    power = rng.exponential(1.0, size=4096)   # |complex gaussian|**2 is exponential
    mask = cfar_1d(power, pfa=1e-3, guard=8, train=32)
    assert mask.mean() < 0.02
