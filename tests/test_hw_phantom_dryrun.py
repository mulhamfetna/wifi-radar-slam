"""The pre-hardware dry run: synthesize captures in the wire format, analyse, recover the echo.

This exercises the full laptop path that the real capture will use -- pack_record (the exact
firmware wire format) -> parse_stream -> differential -> detect -> distance. It is the CI
guard that run_hw_phantom.py stays wired to the pipeline.
"""
import numpy as np

from wifi_radar_slam.hw import ESP32_HT40 as CFG
from wifi_radar_slam.hw.config import C
from wifi_radar_slam.hw.csi import pack_record
from wifi_radar_slam.hw.delay import differential
from wifi_radar_slam.hw.detect import detect_delays
from wifi_radar_slam.hw.synth import Tap, receiver_response, synth_recording

import experiments.run_hw_phantom as rhp

ZP = 16


def _write_capture(path, taps, seed, rx):
    rng = np.random.default_rng(seed)
    packets = synth_recording(taps, CFG, n_packets=300, rng=rng, rx_response=rx,
                              sto_std_s=4e-9, noise_std=0.02, quantise=False)
    with open(path, "wb") as f:
        for i, h in enumerate(packets):
            f.write(pack_record(h / np.abs(h).max() * 100.0, seq=i))


def test_dryrun_recovers_plate_through_wire_format(tmp_path):
    d = 12.0
    tau = CFG.excess_delay_s(d, 0.5)
    rx = receiver_response(CFG)
    pin = tmp_path / "in.bin"
    pout = tmp_path / "out.bin"
    _write_capture(pin, [Tap(1.0, 0.0), Tap(0.15, tau)], 1, rx)
    _write_capture(pout, [Tap(1.0, 0.0)], 2, rx)

    stack_in = rhp.load_capture(str(pin))
    stack_out = rhp.load_capture(str(pout))
    assert stack_in.shape == (300, 128)

    prof = differential(stack_in, stack_out, CFG, ZP)
    delays = detect_delays(prof, CFG, zero_pad=ZP, pfa=1e-6)
    assert delays.size >= 1
    best = delays[np.argmin(np.abs(delays - tau))]
    assert abs(C * best - (2 * d - 0.5)) < CFG.path_cell_m   # within one path cell


def test_dryrun_negative_control_finds_nothing(tmp_path):
    """Two identical (no-plate) recordings must yield NO echo -- the pipeline does not
    hallucinate a tap when there is no difference to detect."""
    rx = receiver_response(CFG)
    a = tmp_path / "a.bin"
    b = tmp_path / "b.bin"
    _write_capture(a, [Tap(1.0, 0.0)], 1, rx)
    _write_capture(b, [Tap(1.0, 0.0)], 2, rx)
    prof = differential(rhp.load_capture(str(a)), rhp.load_capture(str(b)), CFG, ZP)
    delays = detect_delays(prof, CFG, zero_pad=ZP, pfa=1e-6)
    assert delays.size == 0
