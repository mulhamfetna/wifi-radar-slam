# ESP32-S3: the USB port vs the COM port — what's really different, in depth

**Date:** 2026-07-17
**Why this exists:** During the paper-4 hardware bring-up we lost real time to a confusing
problem — the boards flashed fine but the firmware never ran, sitting in "download mode"
forever. The cause was **which of the two USB-C sockets we plugged into.** This document
explains, from the transistor up, why an ESP32-S3 devkit has *two* USB sockets that behave
completely differently, so nobody hits this wall again.

It has two layers:
- **Baby language** sections (🍼) — plain analogies, no jargon.
- **In-depth** sections — the actual electronics, pins, protocols, and IDF settings.

---

## 0. 🍼 The one-paragraph baby version

An ESP32-S3 devkit usually has **two USB sockets**. They look identical but they are not.
One socket (**the COM / UART port**) talks to the chip through a **tiny translator chip** that
also knows how to *politely reboot* the ESP32 into "accept new program" mode and then *politely
reboot it back* into "run the program" mode. The other socket (**the native USB port**) plugs
**straight into the ESP32's own brain**. It can also accept programs, but its "reboot politely"
handshake is fussy, and if the computer holds the wrong wire at the wrong moment the chip gets
**stuck waiting for a new program forever** and never runs the one it already has. For plain
"load my program and watch it run," **use the COM port.** That's the whole lesson.

---

## 1. The two sockets, physically

Most ESP32-S3 devkits (DevKitC-1, and many clones) expose **two USB-C connectors**, often
silk-screened **"UART"** and **"USB"**:

| | **COM / UART port** | **Native USB port** |
|---|---|---|
| Silk label | usually **"UART"** (sometimes "COM") | usually **"USB"** |
| What's behind it | an **external USB-to-UART bridge chip** (FTDI FT232R, or Silicon Labs CP2102, or WCH CH340) | **nothing** — it wires straight to the ESP32-S3's own pins |
| ESP32-S3 pins used | **UART0**: GPIO43 (TX), GPIO44 (RX) | **native USB**: GPIO19 (D−), GPIO20 (D+) |
| Peripheral inside the chip | the chip's **UART0** hardware | the chip's **USB-Serial-JTAG** peripheral |
| On Linux it appears as | **`/dev/ttyUSB0`** (FTDI/CP210x driver) | **`/dev/ttyACM0`** (CDC-ACM driver) |
| USB VID:PID we saw | `0403:6001` (FTDI FT232R) | `303a:1001` (Espressif USB-JTAG) |

> In *our* setup the COM port used an **FTDI FT232R** (`0403:6001` → `ttyUSB0`/`ttyUSB1`), and
> the native port was the S3's built-in **USB-Serial-JTAG** (`303a:1001` → `ttyACM0`). Both are
> "a serial port" to your program, but everything *around* the serial data differs.

### 🍼 Baby version
Think of the ESP32-S3 as a person who only speaks **Morse code** (that's UART — simple beeps).
- The **COM port** puts a **professional interpreter** (the bridge chip) between the person and
  your computer. The interpreter speaks fluent USB to your PC and Morse to the chip, **and**
  knows the secret knock to make the chip say "ok, give me a new program."
- The **native USB port** skips the interpreter — the ESP32-S3 learned to speak USB *itself*.
  Impressive, but it learned the "give me a new program" knock in a fussy way.

---

## 2. The real difference #1 — how each one **reboots the chip to flash it**

To load new firmware, a tool (`esptool`) must put the ESP32-S3 into its **ROM bootloader**
("download mode"), send the program, then reboot it into **run mode**. The chip decides which
mode to enter *at the instant of reset* by reading one pin:

- **GPIO0 (the "BOOT" strap):** HIGH at reset → run the app. LOW at reset → download mode.
- **EN / CHIP_PU (the "reset" pin):** pulsing it low-then-high restarts the chip.

So the sequence to flash is: *hold GPIO0 low → pulse EN → chip wakes in download mode → send
firmware → set GPIO0 high → pulse EN → chip wakes running the app.*

### The COM port does this with a proven hardware circuit
The bridge chip exposes two control lines from the USB side — **DTR** and **RTS** — and the
devkit wires them through the classic **two-transistor auto-reset circuit** to GPIO0 and EN:

```
DTR ──┐                       RTS ──┐
      │  (cross-coupled            │
      │   transistors so the       │
      ▼   two never glitch)        ▼
    GPIO0                         EN
```

`esptool` wiggles DTR and RTS in a known dance; the transistors translate that into a *clean*
GPIO0/EN sequence. This circuit has been used on millions of ESP boards. It **just works** —
flash, then the app runs.

### The native USB port does this *inside the chip*, and it's fussy
There is no external circuit. The **USB-Serial-JTAG peripheral emulates** DTR/RTS→GPIO0/EN
internally. That sounds neater, but:

1. When the **host computer opens the serial port**, the OS often asserts DTR/RTS by default.
   On native USB that can drive GPIO0 **low** right as the chip resets → **download mode.**
2. The reset that fires is reported as `rst:0x15 (USB_UART_CHIP_RESET)` with
   `boot:0x23 (DOWNLOAD(USB/UART0))` — i.e. "I rebooted because USB told me to, and I came up in
   download mode."
3. Different host tools (pyserial, `cat`, esptool, `idf.py monitor`) assert those lines
   differently, so the chip can get **stuck in download mode** no matter what you try from
   software.

**This is exactly what bit us.** On the native port the RX board printed, forever:
```
rst:0x15 (USB_UART_CHIP_RESET), boot:0x23 (DOWNLOAD(USB/UART0))
waiting for download
```
The firmware *was* flashed correctly — the chip just never chose to run it. Every DTR/RTS
combination, every esptool reset mode, and `idf.py monitor` all failed to boot the app. **The
moment we moved to the COM port (FTDI), the app booted on the first try.**

### 🍼 Baby version
Flashing is like swapping a kid's homework:
- **COM port:** there's a gentle robot arm (the transistor circuit) that taps the kid on the
  shoulder ("homework time — hand it over"), takes the new sheet, then taps again ("ok, go
  play"). Smooth every time.
- **Native USB:** the kid taps *their own* shoulder. If your computer happens to be leaning on
  the kid's arm when they try, the tap comes out wrong and the kid just sits there with their
  hand out saying **"give me homework… give me homework…"** forever, never doing the work they
  already have.

---

## 3. The real difference #2 — the port **disappears when the app runs**

On the **native USB** port, the USB hardware *belongs to the ESP32-S3 program.* When the app
boots and re-initializes USB (or crashes and reboots), the USB device **disconnects and
re-enumerates** — the `/dev/ttyACM0` node can vanish and come back, sometimes with a new number.
Your open file handle goes stale and reads **zero bytes**, even though the chip is fine.

On the **COM** port, the **bridge chip owns the USB** and never reboots with the ESP32. The
`/dev/ttyUSB0` node is **rock-stable** across resets, crashes, and reflashes. You can hold it
open and watch the chip reboot underneath you.

### 🍼 Baby version
- **Native USB:** the chip is *both* the actor *and* the telephone. Every time the actor leaves
  the stage to change costume, the phone line drops.
- **COM port:** the telephone (bridge chip) is a separate device bolted to the wall. The actor
  can come and go; the line stays up.

---

## 4. The real difference #3 — **where your `printf` / CSI bytes come out**

This one silently wastes hours. The two ports are wired to **different pins**, so your program's
output only appears on the port whose pins you actually write to.

| you write to… | bytes physically leave on… | you'll see them on… |
|---|---|---|
| `UART0` (default console, `printf`, `ESP_LOG`) | GPIO43/44 | the **COM / `ttyUSB`** port |
| the **USB-Serial-JTAG** driver | GPIO19/20 (native USB) | the **native / `ttyACM`** port |

In ESP-IDF this is the `CONFIG_ESP_CONSOLE_*` choice:
- `CONFIG_ESP_CONSOLE_UART_DEFAULT=y` → logs go out **UART0 → COM port.**
- `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` → logs go out the **native USB port.**

**Our bug:** we first built the receiver to stream its binary CSI over the *native USB-JTAG*,
then plugged into the *COM* port and saw **0 bytes** — the data was leaving pins nobody was
listening to. Flipping the console/stream to **UART0** (to match the COM port) fixed it
instantly. **Rule: the output path in firmware must match the socket you plug the cable into.**

### 🍼 Baby version
The chip has **two mouths**. If you put your ear next to the left mouth but the chip is talking
out of the right mouth, you hear silence — not because it's quiet, but because you're listening
to the wrong mouth. Pick one mouth in the code, and put your ear (the cable) on *that* one.

---

## 5. Side effects, speed, and when to prefer each

| topic | COM / UART bridge | Native USB-Serial-JTAG |
|---|---|---|
| **Reliability of flash + run** | ✅ excellent (proven auto-reset circuit) | ⚠️ fussy; can lock in download mode |
| **Port stability while running** | ✅ never drops | ⚠️ re-enumerates on app reset/crash |
| **Max serial speed** | bridge-limited: FTDI ~3 Mbaud, CP2102 ~1 Mbaud, CH340 ~2 Mbaud | very high (USB bulk, ~MB/s) — not baud-limited |
| **Extra hardware cost** | needs the bridge chip on the board | free — built into the ESP32-S3 |
| **Frees the two GPIOs** | uses GPIO43/44 (UART0) | uses GPIO19/20 (native USB) |
| **Can act as a USB device** (keyboard, mass-storage, CDC) | ❌ no | ✅ yes — that's its point |
| **Best for** | **flashing, logging, our CSI stream — plug-and-go** | products that must *be* a USB device, or need very high throughput, with a proper reset button |

**Practical guidance for this project:** use the **COM port** for everything — flashing,
monitoring, and the CSI byte stream — because it flashes reliably and never drops. Set the
firmware console/stream to **UART0** to match. Reserve the native USB port for later (Phase 2),
if the on-vehicle unit ever needs to *be* a USB device.

> Note: if a board **only** has the native USB port (some ESP32-S3 "SuperMini"/DevKitM clones),
> you can still use it — but keep a finger on the physical **RESET (EN)** button, and if it locks
> in download mode, **power-cycle it** (unplug/replug) to boot the app. A hardware reset with no
> host holding the lines is the reliable escape.

---

## 6. Cheat-sheet: diagnosing which port you're on (Linux)

```bash
lsusb | grep -iE "303a|10c4|1a86|0403"      # 303a=native USB-JTAG · 0403=FTDI · 10c4=CP210x · 1a86=CH340
ls /dev/ttyACM* /dev/ttyUSB*                # ttyACM*=native USB · ttyUSB*=bridge (COM)
# read the boot reason: boot:0x23 (DOWNLOAD...) = stuck in download; boot:0x2/0x8... = running app
```

Symptoms → cause:
- **Flashes fine but never runs; `waiting for download`** → native USB stuck in download mode →
  **switch to the COM port**, or power-cycle / press RESET.
- **App clearly boots (you saw the boot banner) but then 0 bytes** → you're reading the wrong
  mouth → **match `CONFIG_ESP_CONSOLE_*` to the port you're plugged into.**
- **Port number keeps changing / handle goes dead on reset** → native USB re-enumeration → use
  the COM port for a stable node.

---

## 7. What this cost us, in one line

We flashed both boards over the **native USB** port; they sat in **download mode** and never
ran. Every software reset trick failed. **Plugging into the COM (FTDI) port booted the firmware
on the first attempt** — and then the *real* firmware bugs (an HT40 secondary-channel that must
be `ABOVE` on channel 1, and a fixed-rate call that must not `ESP_ERROR_CHECK`) became visible
and fixable. **The port choice wasn't a detail; it was the thing standing between "flashed" and
"running."**
