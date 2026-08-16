# Paper 4 — architecture note: why Python for the science, C for the chip

**Date:** 2026-07-15
**Branch:** `paper4-hardware-testbed`
**Status:** decision record. Answers a design question raised during Rung 0: *why is the
processing in Python and not embedded C / an RTOS, for what is ostensibly an embedded task?*

---

## The one-line answer

**It is not Python *instead of* C. It is Python for the offline science layer, and C on a
FreeRTOS toolchain (ESP-IDF) for the on-chip acquisition layer.** The two layers do different
jobs and are correctly written in different languages. There is **no Python on the chip**, and
the timing-critical acquisition **is** embedded C on an RTOS.

---

## The two layers

```
   ┌─────────────────────────────┐         ┌──────────────────────────────────────┐
   │  ON-CHIP ACQUISITION         │  UART   │  OFFLINE SCIENCE                       │
   │  ESP32-S3, ESP-IDF (C)       │ ──────> │  laptop, Python (NumPy/SciPy)          │
   │  = FreeRTOS RTOS             │  binary │                                        │
   │                             │  CSI    │  IFFT, CFAR, coherent averaging,       │
   │  capture packet, stamp,     │  stream │  differential, phantom-rate scoring,   │
   │  filter MAC, pack bytes,    │         │  comparison to the simulation          │
   │  ship over UART             │         │                                        │
   │  -> real-time, C on RTOS    │         │  -> no deadline, Python                 │
   └─────────────────────────────┘         └──────────────────────────────────────┘
        firmware/                                src/wifi_radar_slam/hw/
```

**ESP-IDF is itself a FreeRTOS-based RTOS toolchain.** So the acquisition firmware already is
exactly what an embedded engineer would expect: C, running on the RTOS, doing nothing in the
CSI callback but `xQueueSend`. We are not avoiding C; the real-time part *is* C.

---

## Why the processing layer is Python — four concrete reasons

### 1. It is an offline analysis task, not a real-time one
Rungs 1–6 process **recorded** CSI on a laptop. Nothing is real-time. The chip's only
real-time duty is *capture → stamp → ship the bytes* — that is the C firmware. Once the bytes
are on disk, the IFFT / CFAR / coherent averaging / phantom scoring have **no deadline**. An
RTOS there would solve a problem we do not have.

### 2. Comparability is a research-integrity requirement, not a preference
The phantom rate **must** be computed with the **exact same code** as papers 2 and 3 —
`radar/processing.py`, `eval/phantom.py` — which are NumPy. If the chain were reimplemented in
C, the hardware number would differ from the simulation number **partly because of the
language**, and the difference could never be attributed cleanly to the hardware. A different
chain **confounds** the experiment. The whole point of Rung 4 is that its number lands on the
**same axis** as 89 % / 18.2 % / 0.1 %.

### 3. The two bugs caught at Rung 0 are the argument for fast iteration
Rung 0 found two foundational bugs — a mean-slope alignment error and a
zero-padding-at-the-wrong-end error (the latter shifted every echo by up to **4 m**, and would
have looked **exactly like a phantom reflection** on hardware). Both were killed in **minutes**
by Python + `pytest` on synthetic CSI. In C, each would have meant a compile → flash → capture
→ inspect cycle, with the bug far harder to isolate. **Catching them before spending a cent is
the correct development order** — and it is the discipline the whole paper-4 restart exists to
enforce.

### 4. The scientific stack is battle-tested
`numpy.fft`, `scipy.signal` windows, `scipy.ndimage` CFAR are mature and correct. Writing a
bespoke FFT in C for offline work would spend effort re-earning correctness NumPy already has —
with more, not fewer, opportunities for exactly the kind of indexing bug (Bug 2) we just fixed.

---

## Where C comes back — and it is planned

**Phase 2 (the self-contained vehicle, later) ports the *winning* algorithm on-chip to the
ESP32-S3, in C.** The design already measured the on-chip budget so this is known to be
feasible:

- **194 kflop per sweep**
- **26 KB** working memory
- **~250× headroom** on the ESP32-S3

So the delay pipeline **can** run on the chip. We simply do not port it until Rung 1 has told
us the echo even exists and Rungs 4–6 have told us *which* algorithm (CFAR vs MUSIC, monostatic
vs bistatic) is worth porting. **Building the C on-chip version now — before the echo is
confirmed — would be the exact "layer N+1 before layer N" mistake the restart forbids.**

---

## Summary

| layer | job | language | why |
|---|---|---|---|
| **acquisition** (`firmware/`) | capture CSI, stamp, ship — real-time | **C on ESP-IDF/FreeRTOS** | timing-critical; runs on the chip |
| **science** (`src/.../hw/`) | IFFT, CFAR, phantom rate — offline | **Python (NumPy/SciPy)** | no deadline; must match the sim byte-for-byte; fast TDD |
| **on-chip port** (Phase 2, later) | the *winning* algorithm on the vehicle | **C** (planned) | self-contained car; 194 kflop/sweep, ~250× headroom |

**Right tool per layer. C on the RTOS for acquisition (now), Python for the science (done and
comparable), C again on-chip once there is a winner worth porting (later).**
