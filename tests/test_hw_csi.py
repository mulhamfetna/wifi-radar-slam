"""Tests for the ESP32 CSI wire parser (hw/csi.py).

Round-tripped against pack_record, so no hardware is needed. The final test wires the parser
to the delay pipeline end to end: bytes -> parse -> recover an injected tap.
"""
import numpy as np
import pytest

from wifi_radar_slam.hw import ESP32_HT40 as CFG
from wifi_radar_slam.hw.config import C
from wifi_radar_slam.hw.csi import (
    HEADER_SIZE,
    WIRE_MAGIC,
    parse_record,
    parse_stream,
    pack_record,
)
from wifi_radar_slam.hw.delay import raw_cir
from wifi_radar_slam.hw.synth import Tap, ideal_csi


def test_header_size_matches_firmware():
    """The Python header must be exactly the 18-byte C struct."""
    assert HEADER_SIZE == 18


def test_roundtrip_preserves_ht_ltf_ordering():
    """pack(order A) then ht_ltf(order A) returns the original vector, bin for bin.

    Uses a distinct integer per bin so any misordering is caught exactly (not just on norm).
    """
    k = CFG.k_grid                       # -64..63
    original = (k.astype(float) + 1j * (2 * k))   # distinct real & imag per bin, within int8
    buf = pack_record(original, order="A")
    rec, nxt = parse_record(buf)
    assert nxt == len(buf)
    got = rec.ht_ltf(CFG, order="A")
    np.testing.assert_array_equal(got.real, original.real)
    np.testing.assert_array_equal(got.imag, original.imag)


def test_order_B_is_the_swapped_halves():
    """order 'B' returns the two 64-bin halves swapped versus 'A' — the disputed alternative."""
    original = (CFG.k_grid.astype(float) + 0j)
    rec, _ = parse_record(pack_record(original, order="A"))
    a = rec.ht_ltf(CFG, order="A")
    b = rec.ht_ltf(CFG, order="B")
    np.testing.assert_array_equal(b[:64], a[64:])
    np.testing.assert_array_equal(b[64:], a[:64])


def test_metadata_survives_roundtrip():
    """Header fields are carried faithfully."""
    v = np.zeros(128, dtype=complex)
    buf = pack_record(v, seq=1234, timestamp_us=987654, rssi=-55,
                      agc_gain=20, fft_gain=8, valid=True)
    rec, _ = parse_record(buf)
    assert rec.seq == 1234
    assert rec.timestamp_us == 987654
    assert rec.rssi == -55
    assert rec.agc_gain == 20
    assert rec.fft_gain == 8
    assert rec.valid is True
    assert rec.cwb == 1 and rec.sig_mode == 1 and rec.n_sub == 192


def test_parse_stream_resyncs_after_garbage():
    """Leading and inter-record garbage bytes are skipped; all real records recovered."""
    r0 = pack_record(np.zeros(128, dtype=complex), seq=0)
    r1 = pack_record(np.zeros(128, dtype=complex), seq=1)
    stream = b"\x00\xff garbage " + r0 + b"\x13\x37" + r1
    recs = parse_stream(stream)
    assert [r.seq for r in recs] == [0, 1]


def test_incomplete_trailing_record_is_ignored():
    """A truncated final record does not crash the parser or yield a partial record."""
    full = pack_record(np.zeros(128, dtype=complex), seq=7)
    truncated = full[:HEADER_SIZE + 10]        # header + a few payload bytes only
    recs = parse_stream(full + truncated)
    assert [r.seq for r in recs] == [7]


def test_gain_linear_interprets_signed_bytes():
    """agc/fft gain bytes >= 128 are read as signed (pyespargos convention)."""
    rec, _ = parse_record(pack_record(np.zeros(128, dtype=complex),
                                      agc_gain=0, fft_gain=0))
    assert rec.gain_linear() == pytest.approx(1.0)
    # a positive gain attenuates the linear scale (< 1)
    rec2, _ = parse_record(pack_record(np.zeros(128, dtype=complex),
                                       agc_gain=20, fft_gain=0))
    assert rec2.gain_linear() < 1.0


def test_end_to_end_bytes_to_recovered_tap():
    """Parser -> delay pipeline: an injected echo survives the wire round-trip.

    Build CSI with a tap, scale it into the int8 range, encode as a wire record, parse it
    back, and confirm the delay profile peaks at the injected excess path. This closes the
    loop from the byte format to the physics.
    """
    d, b = 12.0, 0.5
    tau = CFG.excess_delay_s(d, b)
    h = ideal_csi([Tap(1.0, 0.0), Tap(0.4, tau)], CFG)
    h = h / np.abs(h).max() * 100.0                    # scale into int8 range

    rec, _ = parse_record(pack_record(h))
    parsed = rec.ht_ltf(CFG, order="A")

    prof = np.abs(raw_cir(parsed, CFG, 16))
    g = CFG.path_grid_m(16)
    # Gate to the physical range: skip the LOS lobe near delay 0 AND its circular wrap near
    # the top of the CIR (negative delays), exactly as detect_delays gates to max_path_m.
    window = (g > 2 * CFG.path_cell_m) & (g < 120.0)
    peak_path = g[window][np.argmax(prof[window])]
    assert peak_path == pytest.approx(2 * d - b, abs=CFG.path_cell_m)
