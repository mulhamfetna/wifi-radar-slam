"""Parse the ESP32 binary CSI wire stream (firmware/common/csi_wire.h) into CSI vectors.

The firmware ships the RAW ESP32 CSI buffer (LLTF + HT-LTF, imag-first int8 pairs). This
module extracts the HT-LTF and applies the fftshift, deriving its OWN index order — because
Espressif's own parser ordering is disputed and still open (esp-csi #224). We expose BOTH
half-orderings; the Rung-0.5 empirical check (an empty corridor must show ONE tap) picks the
correct one. Do NOT trust a single ordering blindly.

Wire header (little-endian, 18 bytes), then n_sub complex pairs (imag int8, real int8):
    magic[4] seq(u16) timestamp_us(u32) rssi(i8) agc_gain(u8) fft_gain(u8)
    sig_mode(u8) cwb(u8) n_sub(u8) valid(u8) reserved(u8)
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

from .config import CSIConfig

WIRE_MAGIC = b"CSI1"
HEADER_FORMAT = "<4sHIbBBBBBBB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)     # 18 bytes
assert HEADER_SIZE == 18

HT40_N_SUB = 192                                  # 64 LLTF + 128 HT-LTF
HT40_PAYLOAD_BYTES = HT40_N_SUB * 2               # 384

# Raw HT40 buffer layout (esp-idf wifi.rst): items 0..63 = LLTF (discarded); items 64..127 =
# HT-LTF subcarriers 0..+63; items 128..191 = HT-LTF subcarriers -64..-1. fftshift order
# (k = -64..63) therefore concatenates the negative half then the positive half.
_HTLTF_LO = 64        # first HT-LTF raw index
_HTLTF_HI = 192       # one past the last


@dataclass
class CSIRecord:
    """One parsed CSI record: metadata plus the raw 192-complex HT40 buffer."""

    seq: int
    timestamp_us: int
    rssi: int
    agc_gain: int
    fft_gain: int
    sig_mode: int
    cwb: int
    n_sub: int
    valid: bool
    raw: np.ndarray            # complex, length n_sub (imag+1j*real, chip order)

    def ht_ltf(self, cfg: CSIConfig, order: str = "A") -> np.ndarray:
        """Extract the 128 HT-LTF subcarriers in fftshift order (k = -64..63).

        ``order='A'`` is the esp-idf-documented mapping (negative half = raw 128..191,
        positive half = raw 64..127). ``order='B'`` swaps the two halves — the disputed
        alternative (esp-csi #224). Rung 0.5 decides which gives a single clean tap.
        """
        htltf = self.raw[_HTLTF_LO:_HTLTF_HI]      # 128 values, raw order
        pos = htltf[:64]                            # raw 64..127  -> subcarriers 0..+63
        neg = htltf[64:]                            # raw 128..191 -> subcarriers -64..-1
        if order == "A":
            out = np.concatenate([neg, pos])        # [-64..-1, 0..63] = fftshift
        elif order == "B":
            out = np.concatenate([pos, neg])        # swapped halves
        else:
            raise ValueError("order must be 'A' or 'B'")
        if out.size != cfg.n_subcarriers:
            raise ValueError(f"expected {cfg.n_subcarriers} HT-LTF bins, got {out.size}")
        return out

    def gain_linear(self) -> float:
        """AGC/FFT gain as a linear amplitude scale (design doc Part 7.3).

        gain_db = 1.0*agc_gain + 0.25*fft_gain; scale = 10**(-gain_db/20). Gain bytes are
        signed on the ESP32 (pyespargos: value >= 128 -> value-256), so interpret them so.
        """
        agc = self.agc_gain - 256 if self.agc_gain >= 128 else self.agc_gain
        fft = self.fft_gain - 256 if self.fft_gain >= 128 else self.fft_gain
        gain_db = 1.0 * agc + 0.25 * fft
        return 10.0 ** (-gain_db / 20.0)


def _payload_to_complex(payload: bytes, n_sub: int) -> np.ndarray:
    """int8 pairs (imag, real) -> complex vector of length n_sub."""
    a = np.frombuffer(payload, dtype=np.int8).astype(float)
    imag = a[0::2]
    real = a[1::2]
    return real + 1j * imag


def parse_record(buf: bytes, offset: int = 0):
    """Parse one record at ``offset``. Returns ``(CSIRecord, next_offset)`` or ``None``.

    Returns ``None`` if there are not enough bytes or the magic does not match at ``offset``.
    """
    if offset + HEADER_SIZE > len(buf):
        return None
    fields = struct.unpack_from(HEADER_FORMAT, buf, offset)
    (magic, seq, ts, rssi, agc, fft, sig_mode, cwb, n_sub, valid, _reserved) = fields
    if magic != WIRE_MAGIC:
        return None
    payload_bytes = n_sub * 2
    start = offset + HEADER_SIZE
    if start + payload_bytes > len(buf):
        return None
    raw = _payload_to_complex(buf[start:start + payload_bytes], n_sub)
    rec = CSIRecord(seq=seq, timestamp_us=ts, rssi=rssi, agc_gain=agc, fft_gain=fft,
                    sig_mode=sig_mode, cwb=cwb, n_sub=n_sub, valid=bool(valid), raw=raw)
    return rec, start + payload_bytes


def parse_stream(buf: bytes) -> list[CSIRecord]:
    """Parse every record in a byte stream, resynchronising on the magic after any garbage.

    A serial capture can begin mid-record or carry the occasional corrupt byte; we scan for
    the next ``CSI1`` magic rather than trusting a fixed stride.
    """
    records: list[CSIRecord] = []
    offset = 0
    n = len(buf)
    while offset + HEADER_SIZE <= n:
        result = parse_record(buf, offset)
        if result is None:
            nxt = buf.find(WIRE_MAGIC, offset + 1)
            if nxt < 0:
                break
            offset = nxt
            continue
        rec, offset = result
        records.append(rec)
    return records


# --------------------------------------------------------------------------------------
# pack_record: the inverse, used ONLY for testing/simulation — encode a known HT-LTF vector
# as a wire record, so the parser can be round-tripped with no hardware.
# --------------------------------------------------------------------------------------
def pack_record(
    ht_ltf_fftshift: np.ndarray,
    *,
    order: str = "A",
    seq: int = 0,
    timestamp_us: int = 0,
    rssi: int = -40,
    agc_gain: int = 0,
    fft_gain: int = 0,
    valid: bool = True,
) -> bytes:
    """Encode a 128-bin fftshift-order HT-LTF vector into an HT40 wire record.

    Inverts ``CSIRecord.ht_ltf`` for ``order``: places the vector into raw items 64..191
    (LLTF items 0..63 are zeroed), quantises to int8 (imag, real), and prepends the header.
    Values are rounded and clipped to the int8 range; keep |CSI| <= 127 for exact round-trips.
    """
    ht = np.asarray(ht_ltf_fftshift)
    if ht.size != 128:
        raise ValueError("ht_ltf_fftshift must have 128 bins")
    neg = ht[:64]      # subcarriers -64..-1
    pos = ht[64:]      # subcarriers 0..63
    if order == "A":
        htltf_raw = np.concatenate([pos, neg])   # raw 64..127 = pos, raw 128..191 = neg
    elif order == "B":
        htltf_raw = np.concatenate([neg, pos])
    else:
        raise ValueError("order must be 'A' or 'B'")

    raw = np.zeros(HT40_N_SUB, dtype=complex)
    raw[_HTLTF_LO:_HTLTF_HI] = htltf_raw

    payload = np.empty(HT40_N_SUB * 2, dtype=np.int8)
    payload[0::2] = np.clip(np.round(raw.imag), -128, 127).astype(np.int8)   # imag first
    payload[1::2] = np.clip(np.round(raw.real), -128, 127).astype(np.int8)

    header = struct.pack(HEADER_FORMAT, WIRE_MAGIC, seq & 0xFFFF, timestamp_us & 0xFFFFFFFF,
                         int(rssi), agc_gain & 0xFF, fft_gain & 0xFF, 1, 1, HT40_N_SUB,
                         1 if valid else 0, 0)
    return header + payload.tobytes()
