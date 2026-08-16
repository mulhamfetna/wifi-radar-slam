#!/usr/bin/env python3
"""Analyse an ESP32 CSI capture: plate-in vs plate-out -> the echo (and later, phantom rate).

This is the laptop side of the static bench. It reads the binary wire records the RX firmware
writes (firmware/common/csi_wire.h), runs the delay pipeline (LOS-reference -> coherent
average -> differential), and reports the detected echo delays.

Two modes:
  --synthesize   generate a DEMO capture (no hardware) in the exact wire format, then analyse
                 it. This is the pre-hardware dry run: it exercises the identical code path the
                 real capture will use.
  <in> <out>     analyse two real .bin captures recorded from the RX board's UART.

Usage:
  python experiments/run_hw_phantom.py --synthesize --plate 12.0
  python experiments/run_hw_phantom.py capture_plate_in.bin capture_plate_out.bin
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

from wifi_radar_slam.hw import ESP32_HT40 as CFG
from wifi_radar_slam.hw.config import C
from wifi_radar_slam.hw.csi import pack_record, parse_stream
from wifi_radar_slam.hw.delay import differential
from wifi_radar_slam.hw.detect import detect_delays
from wifi_radar_slam.hw.synth import Tap, receiver_response, synth_recording

ZP = 16


def synthesize_capture(path: str, taps, n_packets: int, seed: int, rx_response) -> None:
    """Write a .bin of wire records for the given taps -- exactly what the RX firmware emits."""
    rng = np.random.default_rng(seed)
    packets = synth_recording(taps, CFG, n_packets=n_packets, rng=rng,
                              rx_response=rx_response, sto_std_s=4e-9,
                              noise_std=0.02, quantise=False)
    with open(path, "wb") as f:
        for i, h in enumerate(packets):
            # scale into int8 range (as manu_scale would on the chip) and encode
            scaled = h / np.abs(h).max() * 100.0
            f.write(pack_record(scaled, seq=i))
    print(f"  wrote {n_packets} records -> {path}")


def load_capture(path: str) -> np.ndarray:
    """Parse a .bin capture into a (n_valid_packets, 128) stack of fftshifted HT-LTF vectors."""
    data = open(path, "rb").read()
    records = parse_stream(data)
    good = [r for r in records if r.valid and r.n_sub == 192]
    if not good:
        raise SystemExit(f"no valid HT40 records in {path} "
                         f"({len(records)} parsed, 0 valid) -- check cwb/sig_mode/len on capture")
    print(f"  {path}: {len(records)} records, {len(good)} valid HT40")
    return np.stack([r.ht_ltf(CFG, order="A") for r in good])


def analyse(plate_in_path: str, plate_out_path: str) -> int:
    print(f"\nAnalysing:\n  plate-in : {plate_in_path}\n  plate-out: {plate_out_path}")
    stack_in = load_capture(plate_in_path)
    stack_out = load_capture(plate_out_path)

    prof = differential(stack_in, stack_out, CFG, ZP)
    delays = detect_delays(prof, CFG, zero_pad=ZP, pfa=1e-6)

    print(f"\nResolution cell: {CFG.path_cell_m:.2f} m path / {CFG.range_cell_m:.2f} m range")
    if delays.size == 0:
        print("  NO ECHO DETECTED above CFAR threshold.")
        print("  -> Rung-1 kill criterion: if this holds at 10, 12 AND 14 m, report the "
              "negative result.")
        return 1
    print(f"  {delays.size} echo tap(s) detected:")
    for tau in np.sort(delays):
        path_m = C * tau
        implied_d = (path_m + 0.5) / 2.0     # invert excess = 2d - b, b = 0.5
        print(f"    excess delay {tau*1e9:6.1f} ns  ->  path {path_m:6.2f} m  "
              f"->  plate at ~{implied_d:5.2f} m")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("plate_in", nargs="?", help="plate-in .bin capture")
    ap.add_argument("plate_out", nargs="?", help="plate-out .bin capture")
    ap.add_argument("--synthesize", action="store_true",
                    help="generate a demo capture (no hardware) and analyse it")
    ap.add_argument("--plate", type=float, default=12.0, help="demo plate distance (m)")
    ap.add_argument("--packets", type=int, default=500, help="packets per recording")
    args = ap.parse_args(argv)

    if args.synthesize:
        d = args.plate
        tau = CFG.excess_delay_s(d, 0.5)
        rx = receiver_response(CFG)      # ONE static response shared by both recordings
        print(f"DRY RUN: synthesising a {d} m plate capture "
              f"(true excess {tau*1e9:.1f} ns / {2*d-0.5:.1f} m path)")
        synthesize_capture("dryrun_plate_in.bin", [Tap(1.0, 0.0), Tap(0.15, tau)],
                           args.packets, seed=1, rx_response=rx)
        synthesize_capture("dryrun_plate_out.bin", [Tap(1.0, 0.0)],
                           args.packets, seed=2, rx_response=rx)
        return analyse("dryrun_plate_in.bin", "dryrun_plate_out.bin")

    if not (args.plate_in and args.plate_out):
        ap.error("give two .bin captures, or use --synthesize for the dry run")
    return analyse(args.plate_in, args.plate_out)


if __name__ == "__main__":
    sys.exit(main())
