#!/usr/bin/env python3
"""The static-bench field test runner -- capture real ESP32 CSI and analyse it, with prompts.

Designed so that on-site you just run ONE command and follow the prompts. It auto-detects the
RX board's serial port, records the CSI, and does the analysis right there.

USAGE (from the repo root, with the venv python so numpy + pyserial are both present):

  # Rung 0.5 -- the ordering / single-tap check (do this FIRST, anywhere):
  .venv/bin/python experiments/run_bench.py check

  # Rung 1 -- the atomic test (needs the corridor: boards 0.5 m apart, plate at 12 m):
  .venv/bin/python experiments/run_bench.py rung1 --plate 12

Options: --port /dev/ttyUSB0 (skip auto-detect) · --baud 460800 · --packets 1000 · --baseline 0.5

Prereqs: both boards flashed (csi_tx on the TX board, csi_rx on the RX board) and powered via
their COM/FTDI ports. The TX must be transmitting (it does so automatically on boot).
"""
from __future__ import annotations

import argparse
import glob
import sys
import time
from datetime import datetime

import numpy as np

try:
    import serial
except ImportError:
    sys.exit("pyserial missing -- run: .venv/bin/pip install pyserial")

from wifi_radar_slam.hw import ESP32_HT40 as CFG
from wifi_radar_slam.hw.config import C
from wifi_radar_slam.hw.csi import parse_stream
from wifi_radar_slam.hw.delay import coherent_average, differential, raw_cir
from wifi_radar_slam.hw.detect import detect_delays

ZP = 16
CAP_DIR = "data/hw_captures"


# --------------------------------------------------------------------------- serial
def find_rx_port(baud: int) -> str:
    """Return the first ttyUSB/ttyACM port that is streaming CSI1 records."""
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("no /dev/ttyUSB* or /dev/ttyACM* found -- is the RX board plugged in?")
    for p in ports:
        try:
            s = serial.Serial(p, baud, timeout=0.3)
            s.reset_input_buffer()
            t = time.time(); buf = b""
            while time.time() - t < 1.5:
                buf += s.read(8192)
            s.close()
            if b"CSI1" in buf:
                return p
        except Exception:
            continue
    sys.exit(f"none of {ports} is streaming CSI1 at {baud} baud. "
             f"Check the RX firmware/baud, and that the TX is transmitting.")


def capture(port: str, baud: int, n_packets: int, label: str) -> tuple[np.ndarray, bytes]:
    """Capture until >= n_packets valid HT40 records are collected. Returns (stack, raw)."""
    s = serial.Serial(port, baud, timeout=0.3)
    s.reset_input_buffer()
    buf = b""
    print(f"  recording '{label}' ... ", end="", flush=True)
    t0 = time.time()
    while True:
        buf += s.read(32768)
        n = buf.count(b"CSI1")
        if n >= n_packets:
            break
        if time.time() - t0 > n_packets / 20 + 15:   # generous timeout
            print(f"\n  WARNING: only {n} records after {time.time()-t0:.0f}s "
                  f"(TX transmitting? boards in range?)")
            break
    s.close()
    recs = parse_stream(buf)
    good = [r for r in recs if r.valid and r.n_sub == 192]
    dt = time.time() - t0
    print(f"{len(good)} records in {dt:.1f}s ({len(good)/dt:.0f}/s)")
    stack = np.stack([r.ht_ltf(CFG, order="A") for r in good]) if good else np.empty((0, 128))
    return stack, buf


def save(raw: bytes, tag: str) -> str:
    import os
    os.makedirs(CAP_DIR, exist_ok=True)
    path = f"{CAP_DIR}/{datetime.now():%Y%m%d-%H%M%S}_{tag}.bin"
    open(path, "wb").write(raw)
    return path


# --------------------------------------------------------------------------- commands
def cmd_check(args):
    """Rung 0.5 -- capture, build the delay profile, confirm ONE clean tap and pick the order."""
    port = args.port or find_rx_port(args.baud)
    print(f"RX port: {port} @ {args.baud} baud")
    print("Rung 0.5 -- the ordering / single-tap check (boards on a table is fine).")
    _, raw = capture(port, args.baud, args.packets, "check")
    save(raw, "check")
    recs = [r for r in parse_stream(raw) if r.valid and r.n_sub == 192]
    if len(recs) < 10:
        sys.exit("too few records -- fix the link before continuing.")

    g = CFG.path_grid_m(ZP)
    away = (g > 3 * CFG.path_cell_m) & (g < 120)
    for order in ("A", "B"):
        stack = np.stack([r.ht_ltf(CFG, order=order) for r in recs])
        prof = np.abs(coherent_average(stack, CFG, ZP))
        peak = g[prof.argmax()]
        sec = prof[away].max() / prof.max() if away.any() else 0.0
        flag = "  <-- cleaner" if sec < 0.3 else ""
        print(f"  order {order}: LOS peak at {peak:5.2f} m | strongest off-LOS = {sec:.2f}x peak{flag}")
    print("\n  Expect ONE dominant tap near 0 m and a small off-LOS ratio. Use whichever order"
          "\n  gives the single clean tap (first light validated order 'A'). On a bare desk the"
          "\n  channel is near-flat (one LOS tap) -- that is the correct Rung-0.5 result.")


def cmd_rung1(args):
    """Rung 1 -- interleaved plate-in / plate-out, differential, detect the echo."""
    port = args.port or find_rx_port(args.baud)
    d, b = args.plate, args.baseline
    tau = CFG.excess_delay_s(d, b)
    print(f"RX port: {port} @ {args.baud} baud")
    print(f"Rung 1 -- atomic test. Plate at {d} m, baseline {b} m.")
    print(f"Prediction: a differential tap at {tau*1e9:.0f} ns  = {2*d-b:.1f} m path"
          f"  = {(2*d-b)/CFG.path_cell_m:.2f} resolution cells.\n")

    ins, outs = [], []
    raw_all = b""
    rounds = args.rounds
    for i in range(rounds):
        input(f"[round {i+1}/{rounds}] Place the plate at {d} m, then press ENTER to record IN...")
        st, raw = capture(port, args.baud, args.packets, f"plate-in {i+1}")
        ins.append(st); raw_all += raw
        input(f"[round {i+1}/{rounds}] REMOVE the plate, then press ENTER to record OUT...")
        st, raw = capture(port, args.baud, args.packets, f"plate-out {i+1}")
        outs.append(st); raw_all += raw

    stack_in = np.concatenate(ins); stack_out = np.concatenate(outs)
    path = save(raw_all, f"rung1_plate{d:g}m")
    print(f"\n  raw saved: {path}")
    if stack_in.shape[0] < 50 or stack_out.shape[0] < 50:
        sys.exit("too few records for a reliable differential -- check the link.")

    prof = differential(stack_in, stack_out, CFG, ZP)
    delays = detect_delays(prof, CFG, zero_pad=ZP, pfa=1e-6)
    g = CFG.path_grid_m(ZP)

    print("\n" + "=" * 60)
    print(f"  resolution cell: {CFG.path_cell_m:.2f} m path / {CFG.range_cell_m:.2f} m range")
    if delays.size == 0:
        print("  RESULT: no echo tap above CFAR threshold.")
        print("  If this holds at 10, 12 AND 14 m -> report the negative result (Rung 1 kill).")
    else:
        print(f"  RESULT: {delays.size} echo tap(s):")
        for t in np.sort(delays):
            implied = (C * t + b) / 2.0
            hit = "  <-- matches prediction" if abs(C*t - (2*d-b)) < CFG.path_cell_m else ""
            print(f"    {t*1e9:6.1f} ns | {C*t:6.2f} m path | plate ~{implied:5.2f} m{hit}")
    print("=" * 60)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("check", help="Rung 0.5 -- ordering / single-tap check")
    c.set_defaults(func=cmd_check)

    r = sub.add_parser("rung1", help="Rung 1 -- the atomic plate test")
    r.add_argument("--plate", type=float, default=12.0, help="plate distance, m (default 12)")
    r.add_argument("--baseline", type=float, default=0.5, help="TX-RX baseline, m (default 0.5)")
    r.add_argument("--rounds", type=int, default=2, help="in/out interleave rounds (default 2)")
    r.set_defaults(func=cmd_rung1)

    for p in (c, r):
        p.add_argument("--port", default=None, help="serial port (default: auto-detect)")
        p.add_argument("--baud", type=int, default=460800, help="baud (default 460800)")
        p.add_argument("--packets", type=int, default=1000,
                       help="records per capture (default 1000)")

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
