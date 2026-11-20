# Simulink over UART Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The single-board ESP32 runs a `Simulink` experiment. A Raspberry Pi
sends the motor command `u` (volts) over UART2. The ESP32 replies with
encoder counts in lockstep, homes on the Pi's start pulse, and keeps every
safety stop. This plan also covers the Pi-side link checker and the daemon
fix needed to run without the DAC.

**Architecture:**
- **Firmware (`cart/cart_single`):**
  - a byte-level link tab, `pi_link.ino`;
  - a new experiment tab, `exp_simulink.ino`, in the existing `Experiment`
    framework;
  - Pi GPIO lines in `hardware.ino`.
- **Pi (`picontrol/`):**
  - `pilink.py`: the packet format, the reference implementation, tested
    with pytest;
  - `uart_link_check.py`: drives the link without Simulink;
  - `dac_daemon.py`: skips DAC writes when the MCP4725 is absent.
- **Later plan:** the MATLAB part (spec part C). It needs Simulink and the
  Raspberry Pi support package installed first.

**Tech Stack:**
- Arduino-ESP32 core 3.3.7 (`esp32:esp32:nodemcu-32s`), `HardwareSerial`
  (`Serial2`).
- Python 3.13, pyserial 3.5, pytest.
- On the Pi: `lgpio` and `smbus2` (already used by the daemon).

**Spec:** `docs/superpowers/specs/2026-10-10-simulink-uart-design.md`.
Background:
- `docs/superpowers/specs/2026-10-07-esp32-experiments-design.md` (the
  experiment framework);
- handoff `C:\Users\super\AppData\Local\Temp\ed2-handoff-2026-10-10.md`.

**Repository:** `C:\Users\super\Documents\ED2\ED2-Repo`, branch
`cart-standalone-swingup`. Every task is in this repo.

## Global Constraints

- Commit locally after each task. **Push only when the user asks.**
- **UART2:** RX = GPIO 16 (from Pi GPIO14/pin 8), TX = GPIO 17 (to Pi
  GPIO15/pin 10), **921600 baud, 8N1**. Pi device: `/dev/ttyAMA0`.
- **Pi lines:**
  - GPIO 4 = start pulse in (`INPUT_PULLDOWN`, active HIGH, from Pi GPIO16);
  - GPIO 5 = stop pulse in (`INPUT_PULLDOWN`, active HIGH, from Pi GPIO20;
    strapping pin);
  - GPIO 18 = homing-complete out (to Pi GPIO13), HIGH **only while a
    Simulink run is open**.
- **Command packet (Pi → ESP32), 6 bytes:** `A5 5A` + float32 LE `u` in
  volts, positive = drive right (+x). Dropped (not answered) if NaN,
  infinite or |u| > 5 V.
- **Measurement packet (ESP32 → Pi), 12 bytes:** `A5 5A`, then:
  - cart count int32 LE: `cartDir * cart_position`, 0 at centre, + to the
    right;
  - pendulum count int32 LE: `pend_position - pendDownRef`, 0 hanging;
  - flags;
  - XOR of bytes 2–10.
- **Flags:** `0x01` Simulink run open (commands applied), `0x02`
  homing-complete line HIGH, `0x04` Simulink selected.
- **Timeouts:**
  - 50 ms with no command after the first one ends the run with
    `run end timeout`;
  - no first command within 30 s does the same;
  - while waiting for the first command, a measurement goes out every
    100 ms.
- The motor command goes through the existing `motorVolts(u)` (±2.5 V =
  PWM 180, friction compensation).
- Safety stays in `control.ino` and `loop()`: soft limit, overspeed, limit
  switches, STOP button, serial `stop`, Pi stop pulse. It never moves into
  experiment code.
- **Experiment and state names:** experiment `Simulink`. States `SIMWAIT`
  (before the first command) and `SIM`, single words for the bridge.
- Per run, before `run end`: `result pi_commands N` and `result pi_drops N`.
- Firmware compile command (PowerShell), used by every firmware task:
  ```powershell
  $cli = (Get-ChildItem "$env:LOCALAPPDATA\Programs\arduino-ide" -Filter arduino-cli.exe -Recurse | Select-Object -First 1).FullName; & $cli compile --fqbn esp32:esp32:nodemcu-32s --warnings default "C:\Users\super\Documents\ED2\ED2-Repo\cart\cart_single"
  ```
  Expected: ends with `Sketch uses ... bytes`, no `error:` lines.
- **Arduino tab rules:**
  - All `.ino` tabs are joined into one file: `cart_single.ino` first, then
    the others alphabetically.
  - **Macros and globals shared between tabs live in `cart_single.ino`.**
  - Functions can be called from any tab, because prototypes are
    generated. Don't mark cross-tab functions `static`.
- Python tests: `cd picontrol && python -m pytest tests -q`.
- User-facing text: no em or en dashes in new messages.

## Review Focus

1. **The Simulink model dies or is killed mid-run.** The motor must stop
   within about 50 ms, with `run end timeout`. → Task 5 code (`simStep`),
   rig check Task 6 step 6.
2. **Motor noise garbles a command** into NaN or a huge value. The packet
   must be dropped and counted, never applied, and the run carries on.
   → Task 4 code (`piLinkRead`), `--garble` in Task 2, rig check Task 6
   step 5.
3. **Bytes left in UART2 from an earlier run, or sent while the ESP32 was
   homing.** They must not be applied as the first command of the new run.
   → Task 5 `simStart()` calls `piLinkReset()`, rig check Task 6 step 4.
4. **A Pi start pulse while the console has another experiment selected.**
   The run must be Simulink, and the console must show `Simulink` as
   selected (unsolicited `ok exp Simulink`). → Task 5 `selectSimulink()`,
   rig check Task 6 step 3.
5. **A Pi stop pulse during homing or while waiting for the first
   command.** Homing aborts (existing `homingStopped()`), or the run ends
   `stop`. In both cases GPIO 18 ends LOW. → Task 4 `stopTriggered()`,
   rig check Task 6 step 7.

---

### Task 1: Packet format reference (`picontrol/pilink.py`)

**Files:**
- Create: `picontrol/pilink.py`
- Create: `picontrol/tests/conftest.py`
- Test: `picontrol/tests/test_pilink.py`

**Interfaces:**
- Produces:
  - `HEADER = b"\xa5\x5a"`, `CMD_LEN = 6`, `MEAS_LEN = 12`;
  - `FLAG_READY = 0x01`, `FLAG_HOMED = 0x02`, `FLAG_SIMULINK = 0x04`;
  - `encode_command(u: float) -> bytes`;
  - `decode_command(packet: bytes) -> float` (raises `ValueError`);
  - `encode_measurement(cart: int, pend: int, flags: int) -> bytes`;
  - `@dataclass(frozen=True) Measurement(cart: int, pend: int, flags: int)`
    with properties `ready`, `homed`, `simulink`;
  - `class MeasurementParser`: `feed(data: bytes) -> list[Measurement]`,
    with attributes `bad_checksums: int` and `skipped_bytes: int`.

- [ ] **Step 1: Write the conftest and the failing tests**

`picontrol/tests/conftest.py`:

```python
import os
import sys

# Tests import the picontrol modules (pilink, uart_link_check, dac_daemon) directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

`picontrol/tests/test_pilink.py`:

```python
import math
import struct

import pytest

from pilink import (FLAG_HOMED, FLAG_READY, FLAG_SIMULINK, Measurement, MeasurementParser,
                    decode_command, encode_command, encode_measurement)

# The same example bytes are in the comments of cart/cart_single/pi_link.ino
CMD_HALF = bytes.fromhex("a5 5a 00 00 00 3f")            # u = 0.5 V
CMD_NEG = bytes.fromhex("a5 5a 00 00 a0 bf")             # u = -1.25 V
MEAS_A = bytes.fromhex("a5 5a e8 03 00 00 fb ff ff ff 07 e8")   # cart 1000, pend -5, flags 7
MEAS_B = bytes.fromhex("a5 5a c0 1d fe ff 02 00 00 00 02 dc")   # cart -123456, pend 2, flags 2


def test_encode_command_examples():
    assert encode_command(0.5) == CMD_HALF
    assert encode_command(-1.25) == CMD_NEG


def test_decode_command_round_trip_and_errors():
    assert decode_command(CMD_NEG) == -1.25
    assert math.isnan(decode_command(encode_command(float("nan"))))
    with pytest.raises(ValueError):
        decode_command(CMD_HALF[:5])
    with pytest.raises(ValueError):
        decode_command(b"\x00\x00" + CMD_HALF[2:])


def test_encode_measurement_examples():
    assert encode_measurement(1000, -5, 7) == MEAS_A
    assert encode_measurement(-123456, 2, 2) == MEAS_B


def test_flags():
    m = Measurement(0, 0, FLAG_READY | FLAG_SIMULINK)
    assert m.ready and m.simulink and not m.homed
    assert Measurement(0, 0, FLAG_HOMED).homed


def test_parser_whole_packets():
    p = MeasurementParser()
    assert p.feed(MEAS_A + MEAS_B) == [Measurement(1000, -5, 7), Measurement(-123456, 2, 2)]
    assert p.bad_checksums == 0 and p.skipped_bytes == 0


def test_parser_byte_by_byte():
    p = MeasurementParser()
    got = []
    for b in MEAS_A:
        got += p.feed(bytes([b]))
    assert got == [Measurement(1000, -5, 7)]


def test_parser_skips_noise_before_header():
    p = MeasurementParser()
    assert p.feed(b"\x00\xa5\x13\xff" + MEAS_A) == [Measurement(1000, -5, 7)]
    assert p.skipped_bytes == 4


def test_parser_bad_checksum_resyncs_on_next_packet():
    bad = MEAS_A[:11] + bytes([MEAS_A[11] ^ 0x01])
    p = MeasurementParser()
    assert p.feed(bad + MEAS_B) == [Measurement(-123456, 2, 2)]
    assert p.bad_checksums == 1


def test_parser_header_bytes_inside_data():
    # cart = 0x5AA5 puts "a5 5a" inside the packet body
    pkt = encode_measurement(0x5AA5, 0, 0)
    p = MeasurementParser()
    assert p.feed(pkt + pkt) == [Measurement(0x5AA5, 0, 0)] * 2


def test_command_is_float32_little_endian():
    assert struct.unpack("<f", encode_command(2.5)[2:])[0] == 2.5
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/picontrol && python -m pytest tests -q`
Expected: collection error, `ModuleNotFoundError: No module named 'pilink'`.

- [ ] **Step 3: Write `picontrol/pilink.py`**

```python
"""Packets of the Pi <-> ESP32 UART link used in Simulink mode.

Wiring: Pi GPIO14 (TX, pin 8) -> ESP32 GPIO16, Pi GPIO15 (RX, pin 10) <- ESP32
GPIO17, 921600 baud 8N1, /dev/ttyAMA0 on the Pi. Lockstep: every command the Pi
sends is answered with one measurement.

  Pi -> ESP32 command, 6 bytes:      A5 5A  u (float32 LE, volts, + = drive right)
  ESP32 -> Pi measurement, 12 bytes: A5 5A  cart (int32 LE)  pend (int32 LE)
                                     flags  XOR of bytes 2-10
Cart count: 0 at centre, + to the right. Pendulum count: 0 hanging.
Bytes 2-11 of a measurement are the old 10-byte SPI packet, so the models'
unpack_spi block reads them unchanged. The firmware side is
cart/cart_single/pi_link.ino; both use the examples in tests/test_pilink.py.
"""
import struct
from dataclasses import dataclass

HEADER = b"\xa5\x5a"
CMD_LEN = 6
MEAS_LEN = 12

FLAG_READY = 0x01      # a Simulink run is open; commands are applied
FLAG_HOMED = 0x02      # homing-complete line (ESP32 GPIO 18) is HIGH
FLAG_SIMULINK = 0x04   # the ESP32 has the Simulink experiment selected


def encode_command(u: float) -> bytes:
    return HEADER + struct.pack("<f", u)


def decode_command(packet: bytes) -> float:
    if len(packet) != CMD_LEN or packet[:2] != HEADER:
        raise ValueError(f"not a command packet: {packet.hex(' ')}")
    return struct.unpack("<f", packet[2:])[0]


def _xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def encode_measurement(cart: int, pend: int, flags: int) -> bytes:
    body = struct.pack("<iiB", cart, pend, flags)
    return HEADER + body + bytes([_xor(body)])


@dataclass(frozen=True)
class Measurement:
    cart: int
    pend: int
    flags: int

    @property
    def ready(self) -> bool:
        return bool(self.flags & FLAG_READY)

    @property
    def homed(self) -> bool:
        return bool(self.flags & FLAG_HOMED)

    @property
    def simulink(self) -> bool:
        return bool(self.flags & FLAG_SIMULINK)


class MeasurementParser:
    """Splits a byte stream into measurements. Resyncs on the header, so a lost
    or corrupted byte costs one packet, not the stream."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_checksums = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list:
        self._buf += data
        out = []
        while True:
            i = self._buf.find(HEADER)
            if i < 0:
                keep = 1 if self._buf[-1:] == HEADER[:1] else 0   # may be half a header
                self.skipped_bytes += len(self._buf) - keep
                del self._buf[:len(self._buf) - keep]
                return out
            if i > 0:
                self.skipped_bytes += i
                del self._buf[:i]
            if len(self._buf) < MEAS_LEN:
                return out
            body = bytes(self._buf[2:11])
            if _xor(body) != self._buf[11]:
                self.bad_checksums += 1
                del self._buf[:1]            # look for the next header after this one
                continue
            del self._buf[:MEAS_LEN]
            out.append(Measurement(*struct.unpack("<iiB", body)))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/picontrol && python -m pytest tests -q`
Expected: `10 passed`.

- [ ] **Step 5: Commit**

```bash
cd /c/Users/super/Documents/ED2/ED2-Repo
git add picontrol/pilink.py picontrol/tests/conftest.py picontrol/tests/test_pilink.py
git commit -m "picontrol: pilink.py, packet format of the Pi-ESP32 UART link (Simulink mode)"
```

---

### Task 2: Link checker (`picontrol/uart_link_check.py`)

**Files:**
- Create: `picontrol/uart_link_check.py`
- Test: `picontrol/tests/test_uart_link_check.py`

**Interfaces:**
- Consumes (Task 1): `encode_command`, `decode_command`, `encode_measurement`,
  `MeasurementParser`, `Measurement`, `CMD_LEN`, the flags.
- Produces:
  - `@dataclass LinkStats`: `commands: int`, `replies: int`,
    `rtts: list[float]` (s), `bad_checksums: int`,
    `last: Measurement | None`, `end: str`;
  - `wait_for_ready(ser, parser, timeout_s: float) -> Measurement | None`;
  - `lockstep(ser, parser, duration_s: float, u_of_t, garble_every: int = 0, reply_timeout_s: float = 0.05) -> LinkStats`;
  - `square_wave(amp: float, half_period_s: float = 0.5)`, which returns a
    function of t;
  - `main(argv=None) -> int`.
  - `ser` is any object with `write(bytes)` and `read(n) -> bytes`
    (non-blocking or a short timeout), such as `serial.Serial` or the test
    fake.

- [ ] **Step 1: Write the failing tests**

`picontrol/tests/test_uart_link_check.py`:

```python
import math

from pilink import (FLAG_HOMED, FLAG_READY, FLAG_SIMULINK, CMD_LEN, MeasurementParser,
                    decode_command, encode_measurement)
from uart_link_check import lockstep, square_wave, wait_for_ready

READY = FLAG_READY | FLAG_HOMED | FLAG_SIMULINK


class FakeEsp:
    """Answers each valid command with a measurement, like the firmware.
    Stops answering after `answer_limit` commands (run ended)."""

    def __init__(self, answer_limit=10**9, flags=READY, idle=b""):
        self.out = bytearray(idle)
        self.inbuf = bytearray()
        self.commands = []
        self.answer_limit = answer_limit
        self.flags = flags

    def write(self, data):
        self.inbuf += data
        while len(self.inbuf) >= CMD_LEN:
            u = decode_command(bytes(self.inbuf[:CMD_LEN]))
            del self.inbuf[:CMD_LEN]
            self.commands.append(u)
            if math.isfinite(u) and abs(u) <= 5.0 and len(self.commands) <= self.answer_limit:
                self.out += encode_measurement(len(self.commands), -len(self.commands), self.flags)
        return len(data)

    def read(self, n=1):
        data, self.out = bytes(self.out[:n]), self.out[n:]
        return data


def test_wait_for_ready_skips_not_ready_packets():
    idle = encode_measurement(0, 0, FLAG_SIMULINK) + encode_measurement(5, 6, READY)
    m = wait_for_ready(FakeEsp(idle=idle), MeasurementParser(), timeout_s=0.5)
    assert m is not None and m.cart == 5 and m.ready


def test_wait_for_ready_times_out():
    assert wait_for_ready(FakeEsp(), MeasurementParser(), timeout_s=0.05) is None


def test_lockstep_runs_until_time_up():
    esp = FakeEsp()
    stats = lockstep(esp, MeasurementParser(), 0.05, lambda t: 0.25)
    assert stats.end == "time up"
    assert stats.commands == stats.replies == len(esp.commands) > 0
    assert set(esp.commands) == {0.25}
    assert stats.last.cart == stats.replies and stats.bad_checksums == 0


def test_lockstep_stops_when_esp_stops_answering():
    stats = lockstep(FakeEsp(answer_limit=3), MeasurementParser(), 1.0, lambda t: 0.0)
    assert stats.replies == 3 and stats.commands == 4
    assert stats.end.startswith("no reply")


def test_lockstep_stops_when_ready_flag_drops():
    stats = lockstep(FakeEsp(flags=FLAG_HOMED), MeasurementParser(), 1.0, lambda t: 0.0)
    assert stats.replies == 1 and stats.end.startswith("ESP32 left the run")


def test_garble_sends_nan_and_skips_waiting_for_its_reply():
    esp = FakeEsp()
    stats = lockstep(esp, MeasurementParser(), 0.05, lambda t: 0.1, garble_every=3)
    assert stats.end == "time up"
    nans = [u for u in esp.commands if math.isnan(u)]
    assert len(nans) == len(esp.commands) // 3
    assert stats.replies == len(esp.commands) - len(nans)


def test_square_wave():
    f = square_wave(0.8, 0.5)
    assert f(0.1) == 0.8 and f(0.6) == -0.8 and f(1.1) == 0.8
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/picontrol && python -m pytest tests/test_uart_link_check.py -q`
Expected: `ModuleNotFoundError: No module named 'uart_link_check'`.

- [ ] **Step 3: Write `picontrol/uart_link_check.py`**

```python
#!/usr/bin/env python3
"""Check the Pi <-> ESP32 UART link (Simulink mode) without Simulink.

Run on the Pi, from the picontrol folder:
  python3 uart_link_check.py                      # u = 0 for 10 s
  python3 uart_link_check.py --u-amp 0.8          # +-0.8 V square wave, 0.5 s each way
  python3 uart_link_check.py --garble 100         # every 100th command is NaN (must be dropped)
  python3 uart_link_check.py --gpio               # pulse start first and stop at the end

Without --gpio, start a Simulink run another way: console "exp select Simulink"
then Run, or the START button with Simulink selected. With --gpio, stop
dac_daemon.py first (it owns the start/stop lines).
When the check ends without --gpio, the ESP32 ends the run itself with
"run end timeout" 50 ms after the last command. That is expected.
"""
import argparse
import statistics
import time
from dataclasses import dataclass, field

from pilink import CMD_LEN, MeasurementParser, encode_command

GPIO_HOME_START = 16      # Pi GPIO -> ESP32 GPIO 4 (start pulse)
GPIO_STOP = 20            # Pi GPIO -> ESP32 GPIO 5 (stop pulse)
PULSE_S = 0.2


@dataclass
class LinkStats:
    commands: int = 0
    replies: int = 0
    rtts: list = field(default_factory=list)
    bad_checksums: int = 0
    last: object = None
    end: str = ""


def _next_measurement(ser, parser, queue, deadline):
    while not queue:
        if time.monotonic() > deadline:
            return None
        queue.extend(parser.feed(ser.read(64)))
    return queue.pop(0)


def wait_for_ready(ser, parser, timeout_s):
    """Read the ESP32's 100 ms idle packets until one says a Simulink run is open."""
    queue = []
    deadline = time.monotonic() + timeout_s
    while True:
        m = _next_measurement(ser, parser, queue, deadline)
        if m is None or m.ready:
            return m


def lockstep(ser, parser, duration_s, u_of_t, garble_every=0, reply_timeout_s=0.05):
    stats = LinkStats()
    queue = []
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t >= duration_s:
            stats.end = "time up"
            break
        stats.commands += 1
        if garble_every and stats.commands % garble_every == 0:
            ser.write(encode_command(float("nan")))   # the ESP32 must drop it, not answer
            continue
        sent = time.monotonic()
        ser.write(encode_command(u_of_t(t)))
        m = _next_measurement(ser, parser, queue, sent + reply_timeout_s)
        if m is None:
            stats.end = f"no reply within {reply_timeout_s * 1000:.0f} ms (run ended on the ESP32?)"
            break
        stats.rtts.append(time.monotonic() - sent)
        stats.replies += 1
        stats.last = m
        if not m.ready:
            stats.end = "ESP32 left the run (ready flag off)"
            break
    stats.bad_checksums = parser.bad_checksums
    return stats


def square_wave(amp, half_period_s=0.5):
    return lambda t: amp if int(t / half_period_s) % 2 == 0 else -amp


def _pulse(handle, pin):
    import lgpio
    lgpio.gpio_write(handle, pin, 1)
    time.sleep(PULSE_S)
    lgpio.gpio_write(handle, pin, 0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=10.0, help="lockstep duration")
    ap.add_argument("--u-amp", type=float, default=0.0, help="square wave amplitude in volts (0 = u stays 0)")
    ap.add_argument("--garble", type=int, default=0, metavar="N", help="send NaN as every Nth command")
    ap.add_argument("--gpio", action="store_true", help="pulse start before and stop after")
    ap.add_argument("--gpio-chip", type=int, default=15, help="lgpio chip (dac_daemon.py uses 15 on the Pi 5)")
    ap.add_argument("--ready-timeout", type=float, default=60.0, help="seconds to wait for the run to open")
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.0005)
    ser.reset_input_buffer()
    handle = None
    if args.gpio:
        import lgpio
        handle = lgpio.gpiochip_open(args.gpio_chip)
        for pin in (GPIO_HOME_START, GPIO_STOP):
            lgpio.gpio_claim_output(handle, pin, 0)
        _pulse(handle, GPIO_HOME_START)
        print("Start pulse sent.")

    print("Waiting for a Simulink run (homing + hanging reference take about 15 s)...")
    parser = MeasurementParser()
    m = wait_for_ready(ser, parser, args.ready_timeout)
    if m is None:
        print(f"No ready packet within {args.ready_timeout:.0f} s. Is Simulink selected and running?")
        return 1
    print(f"Ready: cart {m.cart}, pend {m.pend}, flags 0x{m.flags:02x}")

    u_of_t = square_wave(args.u_amp) if args.u_amp else (lambda t: 0.0)
    stats = lockstep(ser, parser, args.seconds, u_of_t, args.garble)
    if handle is not None:
        _pulse(handle, GPIO_STOP)
        print("Stop pulse sent.")

    rate = stats.replies / args.seconds if stats.end == "time up" else float("nan")
    print(f"Ended: {stats.end}")
    print(f"Commands {stats.commands}, replies {stats.replies}, loop rate {rate:.0f} Hz")
    if stats.rtts:
        print(f"Round trip: median {statistics.median(stats.rtts) * 1000:.3f} ms, "
              f"max {max(stats.rtts) * 1000:.3f} ms")
    print(f"Bad checksums {stats.bad_checksums}, skipped bytes {parser.skipped_bytes}")
    if stats.last is not None:
        print(f"Last: cart {stats.last.cart}, pend {stats.last.pend}, flags 0x{stats.last.flags:02x}")
    return 0 if stats.end == "time up" and stats.bad_checksums == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/picontrol && python -m pytest tests -q`
Expected: `17 passed`.

- [ ] **Step 5: Commit**

```bash
cd /c/Users/super/Documents/ED2/ED2-Repo
git add picontrol/uart_link_check.py picontrol/tests/test_uart_link_check.py
git commit -m "picontrol: uart_link_check.py, lockstep link test without Simulink (rate, round trip, garble)"
```

---

### Task 3: Daemon runs without the DAC (`picontrol/dac_daemon.py`)

**Files:**
- Modify: `picontrol/dac_daemon.py`:
  - the header comment (GPIO lines);
  - the GPIO constants comments;
  - the DAC helpers;
  - `run_daemon()` (I2C open and shutdown).
- Test: `picontrol/tests/test_dac_daemon.py`

**Interfaces:**
- Produces: `dac_probe() -> bool`, which sets the module global
  `_dac_present`. `dac_write()` does nothing when `_dac_present` is False.

- [ ] **Step 1: Write the failing test**

`picontrol/tests/test_dac_daemon.py`:

```python
import logging
import sys
import types

import pytest

# The daemon imports Pi-only modules at the top; stub them for the test.
# "def setup_mqtt() -> mqtt.Client" is evaluated at import, so the stub needs Client.
for name in ("lgpio", "smbus2", "paho", "paho.mqtt", "paho.mqtt.client"):
    sys.modules.setdefault(name, types.ModuleType(name))
if not hasattr(sys.modules["paho.mqtt.client"], "Client"):
    sys.modules["paho.mqtt.client"].Client = object

import dac_daemon  # noqa: E402


class FakeBus:
    def __init__(self, present):
        self.present = present
        self.writes = []

    def read_byte(self, addr):
        if not self.present:
            raise OSError(121, "Remote I/O error")
        return 0

    def write_i2c_block_data(self, addr, reg, data):
        self.writes.append((addr, reg, data))


@pytest.fixture(autouse=True)
def reset_dac(monkeypatch):
    monkeypatch.setattr(dac_daemon, "_dac_present", False)


def test_absent_dac_is_skipped_quietly(monkeypatch, caplog):
    bus = FakeBus(present=False)
    monkeypatch.setattr(dac_daemon, "_bus", bus)
    with caplog.at_level(logging.INFO):
        assert dac_daemon.dac_probe() is False
        dac_daemon.dac_set_neutral()
        dac_daemon.dac_write(1.0)
    assert bus.writes == []
    assert "DAC write failed" not in caplog.text
    assert "not found" in caplog.text


def test_no_i2c_bus_counts_as_absent(monkeypatch):
    monkeypatch.setattr(dac_daemon, "_bus", None)
    assert dac_daemon.dac_probe() is False
    dac_daemon.dac_write(2.5)          # must not raise


def test_present_dac_is_written(monkeypatch):
    bus = FakeBus(present=True)
    monkeypatch.setattr(dac_daemon, "_bus", bus)
    assert dac_daemon.dac_probe() is True
    dac_daemon.dac_set_neutral()
    assert len(bus.writes) == 1 and bus.writes[0][0] == dac_daemon.MCP4725_ADDR
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/picontrol && python -m pytest tests/test_dac_daemon.py -q`
Expected: `AttributeError: ... has no attribute '_dac_present'` (from the fixture).

- [ ] **Step 3: Change `dac_daemon.py`**

a) In the header comment, replace:

```python
# GPIO (Pi 5, using lgpio on gpiochip0):
#   GPIO  6  (Pin 31) → ESP32 #1 GPIO 33 : system_ready / reset (owned by Simulink)
#   GPIO 16  (Pin 36) → ESP32 #2 GPIO  4 : homing start trigger
#   GPIO 20  (Pin 38) → ESP32 #2 GPIO  5 : stop trigger
```

with:

```python
# GPIO (Pi 5, using lgpio), single-board ESP32 (cart/cart_single, Simulink mode):
#   GPIO 16  (Pin 36) → ESP32 GPIO  4 : start pulse (the ESP32 homes and opens a Simulink run)
#   GPIO 20  (Pin 38) → ESP32 GPIO  5 : stop pulse
#   GPIO 13  (Pin 33) ← ESP32 GPIO 18 : HIGH while a Simulink run is open
#   The motor command goes to the ESP32 over UART (/dev/ttyAMA0), sent by the
#   Simulink model. The MCP4725 DAC is no longer fitted: DAC writes are skipped
#   when it is absent (dac_probe at startup).
```

b) Replace the GPIO constants block:

```python
GPIO_SYSREADY   = 6         # → ESP32 #1 GPIO 33 : owned by Simulink, not claimed here
GPIO_HOME_START = 16        # → ESP32 #2 GPIO  4 : homing start trigger
GPIO_STOP       = 20        # → ESP32 #2 GPIO  5 : stop trigger
GPIO_HOME_DONE  = 13        # ← ESP32 #2 GPIO 18 : homing complete signal (input)
```

with:

```python
GPIO_HOME_START = 16        # → ESP32 GPIO  4 : start pulse
GPIO_STOP       = 20        # → ESP32 GPIO  5 : stop pulse
GPIO_HOME_DONE  = 13        # ← ESP32 GPIO 18 : HIGH while a Simulink run is open (input)
```

c) After the line `_model_process  = None   # subprocess.Popen handle for running model` add:

```python
_dac_present    = False  # MCP4725 answered on I2C — set by dac_probe()
```

d) Replace `dac_write` with these two functions:

```python
def dac_probe() -> bool:
    """Check once whether the MCP4725 answers. The single-board rig has none:
    the motor command goes over UART, so DAC writes are then skipped."""
    global _dac_present
    try:
        _bus.read_byte(MCP4725_ADDR)
        _dac_present = True
        log.info(f"MCP4725 found at 0x{MCP4725_ADDR:02x}")
    except Exception:
        _dac_present = False
        log.info("MCP4725 not found: DAC writes skipped (motor command goes over UART)")
    return _dac_present

def dac_write(volts: float):
    """Write voltage to MCP4725. Fast write mode (no EEPROM). No-op without a DAC."""
    if not _dac_present:
        return
    val = voltage_to_dac(volts)
    upper = (val >> 4) & 0xFF
    lower = (val & 0x0F) << 4
    try:
        _bus.write_i2c_block_data(MCP4725_ADDR, 0x40, [upper, lower])
        log.info(f"DAC set to {volts:.3f}V (raw={val})")
    except Exception as e:
        log.error(f"DAC write failed: {e}")
```

`_bus.read_byte` on `None` raises `AttributeError`, which `except Exception`
catches, so a missing bus counts as no DAC.

e) In `run_daemon()`, replace:

```python
    except Exception as e:
        log.error(f"Failed to open I2C bus: {e}")
        sys.exit(1)
```

with:

```python
    except Exception as e:
        log.warning(f"Failed to open I2C bus: {e} (running without the DAC)")
        _bus = None
```

and replace:

```python
    # --- Set DAC to neutral immediately at power-up ---
    log.info("Power-up: setting DAC to neutral (2.5V)")
    dac_set_neutral()
```

with:

```python
    # --- Set DAC to neutral immediately at power-up (if fitted) ---
    if dac_probe():
        log.info("Power-up: setting DAC to neutral (2.5V)")
        dac_set_neutral()
```

and in the `finally:` block replace `        _bus.close()` with:

```python
        if _bus is not None:
            _bus.close()
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/picontrol && python -m pytest tests -q`
Expected: `20 passed`. Also check that `grep -n "GPIO_SYSREADY" dac_daemon.py`
prints nothing. If it prints any use, delete that line as well; it was only
a comment reference.

- [ ] **Step 5: Commit**

```bash
cd /c/Users/super/Documents/ED2/ED2-Repo
git add picontrol/dac_daemon.py picontrol/tests/test_dac_daemon.py
git commit -m "dac_daemon: run without the MCP4725 (probe once, skip writes); GPIO comments for the single-board rig"
```

---

### Task 4: Firmware, Pi lines and UART link (`hardware.ino`, `pi_link.ino`)

**Files:**
- Modify: `cart/cart_single/cart_single.ino`:
  - version and header;
  - pin and link macros;
  - `setup()`.
- Modify: `cart/cart_single/hardware.ino`: `readyLed`, `startTriggered`,
  `stopTriggered`, new `piReadyLine`, `piStartHigh`.
- Create: `cart/cart_single/pi_link.ino`

**Interfaces:**
- Consumes: the globals `cart_position`, `pend_position`, `pendDownRef`,
  `cartDir`.
- Produces (used by Task 5):
  - `void piReadyLine(bool on)`, which drives GPIO 18. `readyLed(false)`
    also calls `piReadyLine(false)`.
  - `bool piStartHigh()`.
  - `void piLinkReset()`, which discards UART2 input and zeroes the
    counters.
  - `bool piLinkRead(float* u)`: non-blocking, true when a valid command
    is complete.
  - `void piLinkSend(uint8_t flags)`: one 12-byte measurement.
  - `uint32_t piLinkCommands()`, `uint32_t piLinkDrops()`.
  - Macros in `cart_single.ino`: `PI_FLAG_READY`, `PI_FLAG_HOMED`,
    `PI_FLAG_SIMULINK`, `SIM_TIMEOUT_MS`, `SIM_FIRST_TIMEOUT_MS`,
    `SIM_IDLE_TX_MS`, `HOMING_DONE_PIN`.

- [ ] **Step 1: Version and header (`cart_single.ino`)**

Replace `#define FW_VERSION "11.1-single"` with `#define FW_VERSION "11.2-single"`.

In the header comment, directly before the line `// =============================================================================` that closes it (the line before `#include "driver/pulse_cnt.h"`), add:

```cpp
//
// Changes in 11.2 (Simulink on the Pi; design in
// docs/superpowers/specs/2026-10-10-simulink-uart-design.md):
//   - Experiment "Simulink": the Pi sends the motor command (volts, float32)
//     over UART2 (RX 16 / TX 17, 921600) and gets encoder counts back in
//     lockstep (pi_link.ino, exp_simulink.ino).
//   - Pi lines back: start pulse in (GPIO 4, selects Simulink), stop pulse
//     in (GPIO 5), ready out (GPIO 18, HIGH while a Simulink run is open).
```

In the pin list in the header, after `//   GPIO  2 : Status LED           (on-board LED on most DevKits)` add:

```cpp
//   GPIO  4 : Pi start pulse       (Pi GPIO16 / pin 36, pull-down, active HIGH)
//   GPIO  5 : Pi stop pulse        (Pi GPIO20 / pin 38, pull-down, active HIGH; strapping pin)
//   GPIO 18 : Pi ready line        (to Pi GPIO13 / pin 33, HIGH while a Simulink run is open)
//   GPIO 16 : UART2 RX             (from Pi GPIO14 TX / pin 8)
//   GPIO 17 : UART2 TX             (to Pi GPIO15 RX / pin 10)
```

- [ ] **Step 2: Pin and link macros (`cart_single.ino`)**

After the two lines

```cpp
// --- Status LED: on while homed and running ---
#define STATUS_LED     2
```

add:

```cpp

// --- Raspberry Pi (Simulink mode): GPIO lines and UART2 ---
#define PI_START_PIN       4    // start pulse in: home, then run Simulink
#define PI_STOP_PIN        5    // stop pulse in (strapping pin: keep the pull-down)
#define HOMING_DONE_PIN   18    // out: HIGH while a Simulink run is open
#define PI_RX_PIN         16
#define PI_TX_PIN         17
#define PI_BAUD       921600
// Measurement packet flags (pi_link.ino; same values in picontrol/pilink.py)
#define PI_FLAG_READY     0x01  // a Simulink run is open; commands are applied
#define PI_FLAG_HOMED     0x02  // HOMING_DONE_PIN is HIGH
#define PI_FLAG_SIMULINK  0x04  // Simulink is the selected experiment
#define PI_U_MAX_V        5.0f  // |u| above this = corrupted packet, dropped
// Simulink run timing (exp_simulink.ino)
#define SIM_TIMEOUT_MS         50     // no command for this long ends the run
#define SIM_FIRST_TIMEOUT_MS   30000  // no first command within this ends the run
#define SIM_IDLE_TX_MS         100    // measurement period while waiting for the first command
```

- [ ] **Step 3: `setup()` (`cart_single.ino`)**

Replace:

```cpp
  pinMode(STATUS_LED, OUTPUT);
  readyLed(false);
```

with:

```cpp
  pinMode(STATUS_LED, OUTPUT);
  pinMode(HOMING_DONE_PIN, OUTPUT);
  readyLed(false);            // also drives the Pi ready line LOW
```

After `  pinMode(RIGHT_LIMIT,  INPUT_PULLUP);` add:

```cpp
  pinMode(PI_START_PIN, INPUT_PULLDOWN);   // Pi drives HIGH for a start pulse
  pinMode(PI_STOP_PIN,  INPUT_PULLDOWN);   // Pi drives HIGH for a stop pulse

  // UART2 to the Pi (Simulink mode). 1 KB RX buffer: commands are drained
  // every loop() pass, and stale bytes are flushed when a Simulink run opens.
  Serial2.setRxBufferSize(1024);
  Serial2.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
```

Replace:

```cpp
  Serial.println("Ready. Waiting for START (button or \"home\") to home...");
```

with:

```cpp
  Serial.println("Ready. Waiting for START (button, \"home\" or Pi start pulse) to home...");
```

Replace:

```cpp
  Serial.println(serialStart ? "START from serial" : "START from button");
  delay(200);  // debounce
```

with:

```cpp
  bool fromPi = piStartHigh();
  Serial.println(fromPi ? "START from Pi" : serialStart ? "START from serial" : "START from button");
  delay(200);  // debounce
  if (fromPi) selectSimulink();   // a Pi start pulse always runs Simulink
```

In `loop()`, replace:

```cpp
    if (startTriggered()) {
      delay(200);  // debounce
```

with:

```cpp
    if (startTriggered()) {
      bool fromPi = piStartHigh();
      delay(200);  // debounce
      if (fromPi) selectSimulink();   // a Pi start pulse always runs Simulink
```

`selectSimulink()` and `simService()` are defined in Task 5. Task 4 does not
compile on its own. It is compiled at the end of Task 5, and both tasks are
committed together there.

- [ ] **Step 4: Pi lines (`hardware.ino`)**

Replace:

```cpp
// Status LED: on while homed and running (replaces the homing-done line to the Pi)
void readyLed(bool on) {
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
}
```

with:

```cpp
// Pi ready line (GPIO 18 -> Pi GPIO13): HIGH only while a Simulink run is open
// (set by simStart). dac_daemon.py launches the model when it goes HIGH.
void piReadyLine(bool on) {
  digitalWrite(HOMING_DONE_PIN, on ? HIGH : LOW);
}

// Status LED: on while homed and running. Turning it off (run end, brake,
// homing) also drops the Pi ready line.
void readyLed(bool on) {
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
  if (!on) piReadyLine(false);
}
```

Replace:

```cpp
bool startTriggered() {
  return (digitalRead(START_BTN) == LOW) || serialStart;
}

bool stopTriggered() {
  // STOP_BTN is normally closed wired to GND:
  //   resting = closed = LOW = not stopped
  //   pressed = open   = HIGH (pulled up) = stopped
  return (digitalRead(STOP_BTN) == HIGH) || serialStop;
}
```

with:

```cpp
bool piStartHigh() { return digitalRead(PI_START_PIN) == HIGH; }

bool startTriggered() {
  return (digitalRead(START_BTN) == LOW) || serialStart || piStartHigh();
}

bool stopTriggered() {
  // STOP_BTN is normally closed wired to GND:
  //   resting = closed = LOW = not stopped
  //   pressed = open   = HIGH (pulled up) = stopped
  return (digitalRead(STOP_BTN) == HIGH) || serialStop || digitalRead(PI_STOP_PIN) == HIGH;
}
```

Also replace the section title line `// Trigger helpers — unify button and USB serial inputs` with
`// Trigger helpers — unify button, USB serial and Pi GPIO inputs`.

- [ ] **Step 5: Create `pi_link.ino`**

```cpp
// =============================================================================
// UART link to the Raspberry Pi (Simulink mode), UART2: RX 16, TX 17, 921600.
// Byte handling only; exp_simulink.ino decides what to do with a command.
//   Pi -> ESP32 command, 6 bytes:      A5 5A  u (float32 LE, volts, + = right)
//     u = 0.5   -> A5 5A 00 00 00 3F
//     u = -1.25 -> A5 5A 00 00 A0 BF
//   ESP32 -> Pi measurement, 12 bytes: A5 5A  cart (int32 LE)  pend (int32 LE)
//                                      flags  XOR of bytes 2-10
//     cart 1000, pend -5, flags 0x07       -> A5 5A E8 03 00 00 FB FF FF FF 07 E8
//     cart -123456, pend 2, flags 0x02     -> A5 5A C0 1D FE FF 02 00 00 00 02 DC
// Same examples as picontrol/tests/test_pilink.py. Bytes 2-11 are the old SPI
// packet, so the Simulink models' unpack_spi reads them unchanged.
// Counts in the old rig's conventions: cart 0 at centre, + to the right;
// pendulum 0 hanging (measured hanging reference).
// =============================================================================
uint8_t  plBuf[4];
int      plState    = 0;    // 0: want A5, 1: want 5A, 2..5: next data byte index + 2
uint32_t plCommands = 0;    // valid commands this run
uint32_t plDrops    = 0;    // complete packets with an impossible u (NaN, inf, > 5 V)

// Discard anything waiting (bytes from an earlier run or sent during homing)
void piLinkReset() {
  while (Serial2.available()) Serial2.read();
  plState    = 0;
  plCommands = 0;
  plDrops    = 0;
}

uint32_t piLinkCommands() { return plCommands; }
uint32_t piLinkDrops()    { return plDrops; }

// Non-blocking. Returns true when one valid command is complete (*u = volts);
// bytes after it stay in the buffer for the next call.
bool piLinkRead(float* u) {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (plState == 0) { if (b == 0xA5) plState = 1; continue; }
    if (plState == 1) { plState = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0); continue; }
    plBuf[plState - 2] = b;
    if (++plState < 6) continue;
    plState = 0;
    float v;
    memcpy(&v, plBuf, 4);                 // the ESP32 is little-endian, like the packet
    if (!isfinite(v) || fabsf(v) > PI_U_MAX_V) { plDrops++; continue; }
    plCommands++;
    *u = v;
    return true;
  }
  return false;
}

void piLinkSend(uint8_t flags) {
  int32_t cart = cartDir * cart_position;          // 0 at centre, + to the right
  int32_t pend = pend_position - pendDownRef;      // 0 hanging
  uint8_t p[12] = { 0xA5, 0x5A };
  memcpy(p + 2, &cart, 4);
  memcpy(p + 6, &pend, 4);
  p[10] = flags;
  uint8_t x = 0;
  for (int i = 2; i <= 10; i++) x ^= p[i];
  p[11] = x;
  Serial2.write(p, sizeof(p));
}
```

(No commit here. The firmware compiles once Task 5 adds `selectSimulink()`
and `simService()`.)

---

### Task 5: Firmware, the Simulink experiment (`exp_simulink.ino`)

**Files:**
- Create: `cart/cart_single/exp_simulink.ino`
- Modify: `cart/cart_single/zz_registry.ino` (register)
- Modify: `cart/cart_single/control.ino` (`endRun` reports the link counts)
- Modify: `cart/cart_single/cart_single.ino` (`loop()` calls `simService()`)

**Interfaces:**
- Consumes (Task 4): `piReadyLine`, `piLinkReset`, `piLinkRead`,
  `piLinkSend`, `piLinkCommands`, `piLinkDrops`, the `PI_FLAG_*` and
  `SIM_*` macros. From the framework: `motorVolts(float)`, `stopMotor()`,
  `readEncoders()`, `announce()`, the globals `phase`, `curExp`, `uOut`,
  `vRefOut`, `lastPrint`, `theta`, `xPos`, `expEndReason`, `runActive`.
- Produces:
  - `Experiment EXP_SIMULINK` (name `"Simulink"`);
  - `void selectSimulink()`, which prints `ok exp Simulink` when the
    selection changes so the bridge and console follow;
  - `bool simulinkSelected()`;
  - `void simService()`, called on every `loop()` pass;
  - `void simFinish()`, called by `endRun` for Simulink runs.

- [ ] **Step 1: Create `exp_simulink.ino`**

```cpp
// =============================================================================
// Simulink: the Raspberry Pi runs the controller (spec 2026-10-10).
// Lockstep over UART2 (pi_link.ino): simService() runs on every loop() pass,
// applies each command from the Pi with motorVolts(u) at once and answers
// with a fresh measurement. The Simulink models run a 1 ms step, faster than
// the 2 ms control step, so this can't wait for runStep(). Until the first
// command a measurement goes out every SIM_IDLE_TX_MS, so the model starts
// with valid counts. simStep() (2 ms) only ends the run on a timeout; the
// safety checks stay in control.ino / loop(), and once they brake or stop,
// commands are ignored and not answered.
// A Pi start pulse selects this experiment (setup() / loop()). GPIO 18 tells
// the Pi the run is open: HIGH from simStart() until the run ends.
// =============================================================================
extern Experiment EXP_SIMULINK;

unsigned long simStartMs    = 0;
unsigned long simLastCmdMs  = 0;
unsigned long simLastIdleTx = 0;
bool          simGotCmd     = false;

bool simulinkSelected() { return curExp == &EXP_SIMULINK; }
bool simRunOpen()       { return simulinkSelected() && phase == PH_EXP; }

void selectSimulink() {
  if (simulinkSelected()) return;
  curExp = &EXP_SIMULINK;
  Serial.println("ok exp Simulink");   // the bridge follows the selection
}

uint8_t simFlags() {
  uint8_t f = 0;
  if (simRunOpen())                         f |= PI_FLAG_READY;
  if (digitalRead(HOMING_DONE_PIN) == HIGH) f |= PI_FLAG_HOMED;
  if (simulinkSelected())                   f |= PI_FLAG_SIMULINK;
  return f;
}

void simStart() {
  stopMotor();
  uOut = vRefOut = 0;
  piLinkReset();                 // nothing from before this run is applied
  simStartMs = simLastIdleTx = millis();
  simGotCmd  = false;
  piReadyLine(true);
  announce("SIMWAIT");
  Serial.println("Simulink: waiting for commands from the Pi on UART2...");
  piLinkSend(simFlags());
}

// Every loop() pass. Outside an open Simulink run, Pi bytes are discarded.
void simService() {
  if (!simRunOpen()) {
    while (Serial2.available()) Serial2.read();
    return;
  }
  float u;
  if (piLinkRead(&u)) {
    motorVolts(u);
    readEncoders();
    piLinkSend(simFlags());
    simLastCmdMs = millis();
    if (!simGotCmd) {
      simGotCmd = true;
      announce("SIM");
    }
  } else if (!simGotCmd && millis() - simLastIdleTx >= SIM_IDLE_TX_MS) {
    simLastIdleTx = millis();
    piLinkSend(simFlags());
  }
}

bool simStep(float dt) {
  unsigned long now = millis();
  if (!simGotCmd) {
    stopMotor();
    uOut = vRefOut = 0;
    if (now - simStartMs > SIM_FIRST_TIMEOUT_MS) {
      Serial.println("Simulink: no command from the Pi within 30 s (is the model running?)");
      expEndReason = "timeout";
      return false;
    }
    return true;
  }
  if ((long)(now - simLastCmdMs) > SIM_TIMEOUT_MS) {
    Serial.printf("Simulink: no command for %lu ms, stopping.\n", now - simLastCmdMs);
    expEndReason = "timeout";
    return false;
  }
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("SIM u %6.3f V  th %7.1f deg  x %6.1f mm  cmds %lu  drops %lu\n",
                  uOut * U_FULL_SCALE_V, theta * RAD_TO_DEG, xPos * 1000.0f,
                  (unsigned long)piLinkCommands(), (unsigned long)piLinkDrops());
  }
  return true;
}

const char* simState() { return simGotCmd ? "SIM" : "SIMWAIT"; }

// Called by endRun() before "run end": the link counts become run results
void simFinish() {
  Serial.printf("result pi_commands %lu\n", (unsigned long)piLinkCommands());
  Serial.printf("result pi_drops %lu\n", (unsigned long)piLinkDrops());
}

// Safety on (soft limit, overspeed); no automatic re-home: the Pi decides
Experiment EXP_SIMULINK = { "Simulink", "Simulink on the Raspberry Pi", false, true, simStart, simStep, simState };
```

`(long)(now - simLastCmdMs)`: `simLastCmdMs` is set in `simService()`
within the same `loop()` pass, before `runStep()`. The cast keeps a same-ms
update from wrapping.

- [ ] **Step 2: Register it (`zz_registry.ino`)**

After `  EXPS[N_EXPS++] = &EXP_FRICTION;` add:

```cpp
  EXPS[N_EXPS++] = &EXP_SIMULINK;
```

- [ ] **Step 3: Report the link counts at run end (`control.ino`)**

In `endRun`, replace:

```cpp
  if (runActive) Serial.printf("run end %s\n", reason);
```

with:

```cpp
  if (runActive && simulinkSelected()) simFinish();   // result pi_commands / pi_drops
  if (runActive) Serial.printf("run end %s\n", reason);
```

- [ ] **Step 4: Call the link service every loop pass (`cart_single.ino`)**

At the top of `loop()`, replace:

```cpp
void loop() {
  readEncoders();
  pollSerial();
```

with:

```cpp
void loop() {
  readEncoders();
  pollSerial();
  simService();   // Pi commands: applied and answered at once (Simulink runs only)
```

- [ ] **Step 5: Compile**

Run the firmware compile command from Global Constraints.
Expected: `Sketch uses ... bytes`, no `error:` lines. Common fixes:
- `'selectSimulink' was not declared`: the function must not be `static`.
- `PI_FLAG_READY was not declared`: the macros must be in `cart_single.ino`
  (Task 4 step 2), not in a later tab.

- [ ] **Step 6: Check the logger tests still pass**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/cart && python -m pytest tests -q`
Expected: all pass. The logger is unchanged; this confirms nothing else
moved.

- [ ] **Step 7: Commit (Tasks 4 and 5)**

```bash
cd /c/Users/super/Documents/ED2/ED2-Repo
git add cart/cart_single/cart_single.ino cart/cart_single/hardware.ino cart/cart_single/pi_link.ino \
        cart/cart_single/exp_simulink.ino cart/cart_single/zz_registry.ino cart/cart_single/control.ino
git commit -m "cart_single v11.2: Simulink experiment, lockstep UART2 link to the Pi, Pi start/stop/ready lines"
```

---

### Task 6: Rig check (user runs, Claude reads the logs)

**Files:** none (results go in Task 7's docs if anything differs).

**Interfaces:**
- Consumes: Tasks 1–5.
- Produces: the spec's "done when" for parts A and B.

Prepare:
- **PC:** flash `cart/cart_single` and start `python cart/serial_log.py`
  (or the console bridge).
- **Pi:**
  - `sudo raspi-config` → Interface Options → Serial Port: login shell
    **No**, serial hardware **Yes**, then reboot;
  - copy `picontrol/pilink.py` and `picontrol/uart_link_check.py` into the
    Pi's `picontrol` folder;
  - `python3 -m pip install pyserial` (if missing).

Ask the user to run these steps in order. Claude reads the newest
`cart/logs/*.log` (`grep -a`) and the script output after each step.

- [ ] **Step 1: Boot banner and regression.**
  - The banner shows `v11.2-single`.
  - `exp list` includes `exp Simulink | Simulink on the Raspberry Pi`.
  - Run SwingBal once and FreeSwing once from the console. Both end with
    `run end done` (SwingBal: STOP after it balances).
- [ ] **Step 2: Lockstep at u = 0.**
  - Console: `exp select Simulink`, then Run.
  - On the Pi: `python3 uart_link_check.py --seconds 60` (paced at 1 kHz
    like a model; add `--rate 0` once to see the link's top speed).
  - Expected script output:
    - `Ended: time up`, `missed replies 0` (a few are tolerated; each is
      one lost packet, also counted in `result pi_drops`);
    - loop rate 1000 Hz;
    - median round trip under 0.5 ms;
    - `Bad checksums 0`;
    - last cart count near 0, pendulum near 0 (hanging).
  - Expected log: `>> SIMWAIT`, `>> SIM`, then about 60 s later
    `Simulink: no command for ...`, `result pi_commands N` (N close to the
    script's commands), `result pi_drops 0`, `run end timeout`.
- [ ] **Step 3: Pi start pulse** (review focus 4).
  - Console: select **FreeSwing**.
  - On the Pi, with `dac_daemon` stopped
    (`sudo systemctl stop dac_daemon`, or Ctrl-C if it runs in a
    terminal): `python3 uart_link_check.py --gpio --seconds 5`.
  - Expected log: `START from Pi` or a homing start, `ok exp Simulink`,
    `run start Simulink`, then `run end stop` (the script's stop pulse).
  - The console picker shows Simulink.
- [ ] **Step 4: Stale bytes** (review focus 3).
  - Console: select Simulink and press Run. While it homes, run
    `python3 uart_link_check.py --blast --seconds 20` on the Pi. It writes
    0.8 V commands without waiting and should finish during the hanging
    reference.
  - Expected: the cart does not move after homing; `>> SIMWAIT`, and the
    `D` lines show `u` = 0.000 until a real check runs:
    `python3 uart_link_check.py --seconds 5 --u-amp 0.3 --half-period 0.2`.
- [ ] **Step 5: Moving and garbled commands** (review focus 2).
  - `python3 uart_link_check.py --seconds 10 --u-amp 0.3 --half-period 0.2 --garble 50`.
    (Measured 2026-10-19: at 0.5 V the cart reaches 1 m/s in 0.35 s and hits the
    soft limit inside one 0.5 s half-period, so keep the amplitude and half-period small.)
  - Expected: the cart shuttles both ways, `D` lines show `u` = ±0.800, the
    script ends `time up`, and the log shows `result pi_drops` equal to
    about commands / 50, with the run not stopped by the drops.
  - If the cart drifts into the soft limit instead (`run end soft-limit`),
    that is the safety working. Repeat with `--u-amp 0.2`.
- [ ] **Step 6: Model killed** (review focus 1).
  - Run `python3 uart_link_check.py --seconds 30 --u-amp 0.3 --half-period 0.2` and press
    Ctrl-C after about 5 s.
  - Expected log: `Simulink: no command for 5x ms, stopping.`, then
    `run end timeout`. The cart stops, and the `D` lines after the last
    command span no more than about 60 ms.
  - Then select Simulink, press Run, and start no script. After about 30 s:
    `no command from the Pi within 30 s`, `run end timeout`.
- [ ] **Step 7: Stops** (review focus 5).
  - Pi stop pulse during homing:
    `python3 -c "import lgpio,time; h=lgpio.gpiochip_open(15); lgpio.gpio_claim_output(h,20,0); lgpio.gpio_write(h,20,1); time.sleep(0.2); lgpio.gpio_write(h,20,0)"`
    while the cart homes. Expected: `STOP during homing — aborted.`
  - The STOP button during `SIMWAIT`. Expected: `run end stop`.
  - A Pi stop pulse right after centring (during the 0.3 s settle, before
    `Hanging reference`): the same one-liner, sent the moment the cart stops
    at the centre. Expected: the run ends `stop` (the pulse is latched),
    not a 30 s `timeout`.
  - On the Pi, `pinctrl get 13` (or `gpioget`) reads LOW after each stop.
- [ ] **Step 8: The daemon without the DAC.**
  - Copy `picontrol/dac_daemon.py` to the Pi and start it.
  - Expected: `MCP4725 not found: DAC writes skipped`, and no
    `DAC write failed` lines over a start/stop cycle sent through MQTT.
    The model launch fails until part C provides a `_UART` model; that is
    expected here.

If any step fails, use superpowers:systematic-debugging against the logs.
Fix the firmware or script, then repeat from the failed step.

---

### Task 7: Docs

**Files:**
- Modify: `docs/esp32.md` (new section before `## Handoff summary`)
- Modify: `docs/raspberry-pis.md` (under `## \`dac_daemon.py\` — the main program`)
- Modify: `docs/superpowers/specs/2026-10-07-esp32-experiments-design.md` (status note)

**Interfaces:**
- Consumes: Tasks 1–6, plus any numbers measured in Task 6 (round trip,
  loop rate).

- [ ] **Step 1: `docs/esp32.md`**

Insert before `## Handoff summary`:

```markdown
## Single board, Simulink on the Pi (v11.2)

`cart/cart_single` (one ESP32) runs the experiment `Simulink`: the Raspberry Pi
runs the Feedback model and sends the motor command over UART; the ESP32
homes, measures, drives the motor and keeps every safety stop. Design:
`docs/superpowers/specs/2026-10-10-simulink-uart-design.md`.

| Pi header pin | Pi signal | ESP32 GPIO | Use |
|---|---|---|---|
| 8 | GPIO14, UART TX | 16 (UART2 RX) | commands |
| 10 | GPIO15, UART RX | 17 (UART2 TX) | measurements |
| 6 | GND | GND | common ground |
| 36 | GPIO16 | 4 | start pulse (selects Simulink, homes) |
| 38 | GPIO20 | 5 | stop pulse |
| 33 | GPIO13 | 18 | HIGH while a Simulink run is open |

UART2: 921600 baud 8N1, `/dev/ttyAMA0` on the Pi (raspi-config: serial login
shell off, serial hardware on). Lockstep: each command is answered at once
with one measurement.

- **Command, Pi to ESP32 (6 bytes):** `A5 5A` + `u` float32 LE in volts
  (+ = right). Applied with `motorVolts(u)` (±2.5 V full scale, friction
  compensation from the Friction experiment). NaN, infinite or |u| > 5 V is
  dropped and counted.
- **Measurement, ESP32 to Pi (12 bytes):** `A5 5A`, cart count int32 LE (0 at
  centre, + right), pendulum count int32 LE (0 hanging), flags (`0x01` run
  open, `0x02` ready line HIGH, `0x04` Simulink selected), XOR of bytes 2–10.
  Bytes 2–11 are the old SPI packet.
- **Run:** start pulse → homing → hanging reference → `>> SIMWAIT` (ready line
  HIGH, a measurement every 100 ms) → first command → `>> SIM`. Ends on the Pi
  stop pulse, STOP, soft limit, overspeed, limit switch, 50 ms without a
  command, or no first command within 30 s (`run end timeout`). Before
  `run end`: `result pi_commands N` and `result pi_drops N`.
- **Check without Simulink:** `picontrol/uart_link_check.py` (see its
  `--help`); packet code and tests in `picontrol/pilink.py`,
  `picontrol/tests/`.
```

Then add one line under that list with the Task 6 numbers, for example
`Measured 2026-10-xx: median round trip 0.21 ms, max 0.9 ms, 1000 Hz, 0 bad checksums in 60 s.`,
using the actual values from the Task 6 step 2 output.

- [ ] **Step 2: `docs/raspberry-pis.md`**

Directly under the heading `## \`dac_daemon.py\` — the main program`, add:

```markdown
> **Single-board rig (2026-10):** there is no MCP4725 any more. The Simulink
> model sends the motor command to the ESP32 over `/dev/ttyAMA0` (see
> `docs/esp32.md`, "Single board, Simulink on the Pi"). The daemon probes the
> DAC once at startup and skips DAC writes when it is absent; the start/stop
> pulses, the GPIO 13 wait and the model launch are unchanged. GPIO 6
> (system ready) is no longer wired. Link check without Simulink:
> `python3 picontrol/uart_link_check.py --help`.
```

- [ ] **Step 3: Mark the old spec's later stages superseded**

In `docs/superpowers/specs/2026-10-07-esp32-experiments-design.md`, replace
the line `Status: draft for review` with:

```markdown
Status: stages 0–2 built. Stages 3–8 superseded on 2026-10-10: the Feedback
experiments run as Simulink models on the Pi instead
(`2026-10-10-simulink-uart-design.md`). SwingBal, FreeSwing and Friction stay
in the firmware.
```

- [ ] **Step 4: Commit**

```bash
cd /c/Users/super/Documents/ED2/ED2-Repo
git add docs/esp32.md docs/raspberry-pis.md docs/superpowers/specs/2026-10-07-esp32-experiments-design.md
git commit -m "docs: single-board Simulink mode (wiring, packets, run sequence); daemon without DAC; old experiments stages 3-8 superseded"
```

---

## Later: MATLAB model copies (spec part C), separate plan

Part C waits until this PC has **Simulink and the Simulink Support Package
for Raspberry Pi Hardware**. Today MATLAB R2026b is installed without either.
The script has to be written against the installed block library: the exact
Serial Receive/Transmit block paths and parameter names, and whether
921600 baud can be set on `/dev/ttyAMA0`. Writing it now would mean guessing
those names. Once the installs are done, write
`docs/superpowers/plans/YYYY-MM-DD-simulink-uart-models.md` from spec part C:
`make_uart_models.m`, `_UART` copies, the 1968 counts/rev gain, R2025b
compatibility, and rig check 6.

## Self-review notes

| Spec item | Task |
|---|---|
| Wiring: UART2 16/17, Pi lines 4/5/18 | 4 |
| Run sequence: start pulse selects Simulink, ready line only during the run, idle packets every 100 ms, lockstep, ends | 4, 5 |
| 50 ms and 30 s timeouts, `run end timeout` | 5 |
| Command validity (NaN, inf, > 5 V), header resync, drop count as `result` lines | 4, 5 |
| Measurement packet, count conventions, flags | 1, 4 |
| `motorVolts(u)` mapping, safety outside the experiment, no auto re-home | 5 |
| Button/console/serial `run` keep running the selected experiment | 4 (only the Pi pulse calls `selectSimulink`) |
| USB `D` lines during Simulink runs | unchanged `runStep()`; checked in Task 6 |
| B: `pilink.py` + tests, `uart_link_check.py` | 1, 2 |
| D: daemon without the DAC | 3 |
| E: docs, old spec superseded | 7 |
| C: MATLAB | separate plan (needs installs) |

Not covered on purpose: console labels for `SIMWAIT`/`SIM`. Unknown states
already show as "Running" (stage 2), and the spec lists the console as
unchanged.
