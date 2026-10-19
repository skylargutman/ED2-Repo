# Simulink on the Pi over UART: Design

Date: 2026-10-10
Status: draft for review

## Goal

The Feedback lab models run in Simulink on the Raspberry Pi again, now on the
single-board rig (one ESP32, `cart/cart_single`). The Pi sends the motor
command over a UART. The MCP4725 DAC, the ADS1115 and the SPI link are gone.
The ESP32 measures, homes, drives the motor and enforces safety. Simulink
does all the control.

### What was decided

- **Simulink replaces the built-in experiments.** Stages 3–8 of
  `2026-10-07-esp32-experiments-design.md` (experiments 3–11 in firmware)
  are dropped. Three built-in modes stay as rig tools:
  - **SwingBal**: proven swing-up and balance, the demo and rig check.
  - **FreeSwing**: pendulum period and decay.
  - **Friction**: its stored breakaway voltages are the compensation that
    `motorVolts(u)` applies to every Simulink command.
- **Link:** Pi GPIO UART to ESP32 UART2 for commands and measurements. The
  Pi's GPIO lines handle start, stop and homing complete.
- **ESP32 to Pi:** the old 10-byte SPI packet (raw counts, flags,
  checksum) behind an `A5 5A` header, so the models' `unpack_spi` and their
  count scaling carry over.
- **Pi to ESP32:** `u` in volts as a float32. The ESP32 maps it with
  `motorVolts(u)`, the same as the built-in experiments (±2.5 V = PWM 180,
  friction compensation on the ESP32). The models' own Friction
  Compensation and DAC blocks are removed.
- **Lockstep timing:** the Pi sets the pace. Each command it sends makes
  the ESP32 reply with a fresh measurement.
- **ESP32 USB stays free.** The `pendulum-remote` console and
  `cart/serial_log.py` can watch and record Simulink runs as experiment
  `Simulink`.

### Out of scope

- New Simulink controllers. Gains in the copied models are re-tuned by the
  students and team on the rig.
- The Django site, MQTT topics and the `pendulum-remote` console. No changes
  are needed: `Simulink` appears in `exp list` like any experiment.
- Removing the old two-board sketches (`cart_control`, `esp32_encoder_v9`).
  They stay in the repo for reference.

## Wiring (already on the rig)

| Pi header pin | Pi signal | ESP32 GPIO | Use |
|---|---|---|---|
| 8 | GPIO14, UART TX | 16 (UART2 RX) | commands, Pi to ESP32 |
| 10 | GPIO15, UART RX | 17 (UART2 TX) | measurements, ESP32 to Pi |
| 6 | GND | GND | common ground |
| 36 | GPIO16 | 4 | start pulse (input, pull-down, active HIGH) |
| 38 | GPIO20 | 5 | stop pulse (input, pull-down, active HIGH) |
| 33 | GPIO13 | 18 | homing complete (output, HIGH = homed and running) |

- Both boards use 3.3 V logic, so no level shifting is needed.
- GPIO 5 is an ESP32 strapping pin. With a pull-down and the Pi holding it
  LOW at boot it is safe, but the Pi must not drive it HIGH while the ESP32
  resets.
- Pi setup: in `raspi-config`, turn the serial login shell **off** and the
  serial hardware **on**, then use `/dev/ttyAMA0` (with `dtparam=uart0=on` in
  `/boot/firmware/config.txt`). **Not `/dev/serial0`:** on the Pi 5 it points
  to `ttyAMA10`, the 3-pin debug connector (found on the rig 2026-10-19).
- The old system-ready line (Pi GPIO 6 to ESP32 #1 GPIO 33) is **not
  connected**. GPIO 33 is now the right limit switch.

## Run sequence

1. `picontrol/dac_daemon.py` gets an MQTT `sta` command and pulses **start**
   (GPIO 16, 200 ms).
2. **The ESP32 selects `Simulink`** whatever experiment was selected, homes,
   and measures the hanging reference as usual. When the run opens it raises
   **homing complete** (GPIO 18) and waits with the motor off. GPIO 18 is
   HIGH only while a Simulink run is open: from the end of the hanging
   reference until the run ends. Other experiments leave it LOW, so the
   daemon never launches a model against a different experiment.
3. The daemon sees GPIO 13 HIGH and launches the model's `.elf`. It writes
   no DAC values: with no MCP4725 present, DAC writes are skipped (part D).
4. **While waiting** for the first command, the ESP32 sends a measurement
   packet every 100 ms, so the model starts with valid counts.
5. **Lockstep:** for each valid command packet the ESP32:
   - applies `motorVolts(u)`;
   - reads the encoders;
   - sends one measurement packet straight away.

   The model's sample time sets the loop rate. Every `*PiMod2.slx` uses a
   1 ms fixed step, and the SPI block inherits it, so the loop runs at 1 kHz.
6. **The run ends** on any of:
   - a Pi stop pulse (GPIO 20);
   - the STOP button or serial `stop`;
   - soft limit, overspeed or limit switch (the existing checks);
   - **no command for 50 ms** after the first one (`run end timeout`);
   - **no first command within 30 s** of homing (`run end timeout`, for
     example if the model failed to launch).

   The motor turns off, GPIO 18 goes LOW and the ESP32 returns to idle.
   No automatic re-home.
7. The START button, the console and serial `run` keep running the
   **selected** experiment. Selecting `Simulink` from the console and
   pressing Run also works: the ESP32 homes and waits for the Pi's commands
   (useful with the link check in part B).

## Packets (UART2)

**Baud rate: 921600, 8N1.** A round trip is 18 bytes, about 0.2 ms, which
leaves most of the models' 1 ms step for Linux scheduling jitter. At 230400
it would take 0.8 ms, too tight at 1 kHz. The baud rate is one `#define` on
the ESP32 and one setting in Simulink.

### ESP32 to Pi: measurement, 12 bytes

| Byte | Content |
|---|---|
| 0–1 | header `A5 5A` |
| 2–5 | cart count, int32 little-endian |
| 6–9 | pendulum count, int32 little-endian |
| 10 | flags |
| 11 | XOR of bytes 2–10 |

Bytes 2–11 are byte for byte the old SPI packet, so `unpack_spi` reads them
unchanged.

**Counts use the old rig's conventions:**
- **Cart:** 0 at centre (set by homing), increasing to the right. The ESP32
  sends `cartDir * cart_position`, so the breadboard's reversed encoder is
  hidden.
- **Pendulum:** 0 hanging (the measured hanging reference), sent as
  `pend_position - pendDownRef`. The direction is as before.

**The pendulum encoder has 1968 counts/rev**, not the 2048 assumed in the
models. The model copies (part C) correct the scaling gain.

**Flags:**

| Bit | Meaning |
|---|---|
| 0x01 | ready: a Simulink run is open and commands are applied |
| 0x02 | homing complete (same as GPIO 18) |
| 0x04 | Simulink mode (the selected experiment is `Simulink`) |

### Pi to ESP32: command, 6 bytes

| Byte | Content |
|---|---|
| 0–1 | header `A5 5A` |
| 2–5 | `u`, float32 little-endian, volts. Positive drives the cart right (+x). |

The ESP32 receiver:
- Searches for `A5 5A` byte by byte, so a lost byte costs one packet, not
  the run.
- **Drops** a packet if `u` is NaN or infinite, or if |u| > 5 V (not a
  plausible command, so the packet is probably corrupted). A dropped packet
  is not answered and counts toward the 50 ms timeout. Values between 2.5
  and 5 V are clamped to ±2.5 V by `motorVolts`.
- Ignores packets outside a Simulink run.
- Counts commands and dropped packets. Both are reported before
  `run end` as `result pi_commands N` and `result pi_drops N`. The bridge
  already attaches `result` lines to the run, and the `run end` line keeps
  its single-word reason.

## Firmware (`cart/cart_single/`)

- **`cart_single.ino`:**
  - pins `PI_START_PIN 4`, `PI_STOP_PIN 5`, `HOMING_DONE_PIN 18`,
    `PI_RX_PIN 16`, `PI_TX_PIN 17`;
  - `PI_BAUD 921600`;
  - `SIM_TIMEOUT_MS 50`, `SIM_FIRST_TIMEOUT_MS 30000`, `SIM_IDLE_TX_MS 100`;
  - `Serial2.begin` in `setup()`;
  - the version goes to 11.2.
- **`hardware.ino`:**
  - `startTriggered()` and `stopTriggered()` also read the Pi lines.
    A Pi start pulse sets a flag that makes the next run use `Simulink`.
  - `piReadyLine(bool)` drives GPIO 18. `readyLed(false)` also clears
    it, so every run end and brake drops the line.
- **New tab `pi_link.ino`:**
  - packet receive (header search, validity check);
  - packet send (`sendMeasurement(flags)`);
  - the drop counter.

  It is pure byte handling with no control logic. The encode/decode
  functions have comments that give example bytes matching the Python tests
  in part B.
- **New tab `exp_simulink.ino`:** `EXP_SIMULINK` with `safety = true` and
  `autoRehome = false`.
  - `start()` resets the timers and the first-command wait.
  - `step()` polls `pi_link`, applies `motorVolts(u)`, replies, and handles
    the two timeouts by setting `expEndReason = "timeout"`.
  - The state names on `>>` and `status` lines are `SIMWAIT` (waiting for
    the first command) and `SIM`.
  - **Lockstep vs the 2 ms step:** commands arrive at 1 kHz, faster than
    the 2 ms control step. `loop()` polls `pi_link` on every pass. When a
    valid command arrives while the run is in `PH_EXP`, `loop()`:
    - applies `motorVolts(u)` straight away;
    - reads the encoders;
    - sends the reply.

    The 2 ms `runStep()` still runs the safety checks and the timeouts, and
    writes the `D` line (500 Hz, latest `u`). Once safety switches to
    braking or idle, commands are ignored and not answered, so a model
    can't fight the brake.
- **`zz_registry.ino`:** registers `EXP_SIMULINK` after Friction.
  `SignCheck` and `StepTest` stay as bring-up checks.
- **USB `D` lines** keep working during Simulink runs: `u` is the commanded
  volts and `ref` is `nan`. The console records the run as `Simulink` with
  the usual CSV and `.mat` files.
- Safety (soft limit, overspeed, limit switches, STOP, Pi stop) stays in
  `control.ino` and `loop()`, never in `exp_simulink.ino`.

## Pi side (`picontrol/`)

### B. `uart_link_check.py` (new)

A Python check of the link with no Simulink:
1. Pulse start and wait for GPIO 13.
2. Run lockstep for N seconds, sending `u = 0`, or a small square wave with
   `--u-amp`.
3. Print:
   - the loop rate;
   - the round-trip time (median and max);
   - checksum failures;
   - the cart and pendulum counts and flags;
   - how the run ended.
4. Pulse stop.

Packet encode/decode lives in a small module, `picontrol/pilink.py`, with
pytest tests in `picontrol/tests/`. The tests use the same example bytes as
the firmware comments.

### D. `dac_daemon.py`

- At startup, probe the MCP4725 once. If it is absent, log that once and
  skip every DAC write. Today every write logs "DAC write failed".
- No other behaviour changes. The pulses, the GPIO 13 wait, the `.elf`
  launch and MQTT stay as they are.
- Remove the stale GPIO 6 / system-ready comments.

## MATLAB (C)

The models are
`MATLAB/Feedback Inverted Pendulum/RealTime Models/*PiMod2.slx`. All share:
- input: `SPI Controller Transfer` into `Encoder_data` (`unpack_spi`);
- output: the `Feedback DAC1` subsystem (Friction Compensation, then
  `voltToMCP4725`, then `I2C Controller Write`);
- also a GPIO Write for system ready, and MQTT Publish.

The script `MATLAB/.../make_uart_models.m`:
- builds two reusable subsystems:
  - **UART Receive:** Raspberry Pi Serial Receive on `/dev/ttyAMA0` at
    921600, header `A5 5A`, 10 data bytes into the existing `unpack_spi`;
  - **UART Send:** pack `u` as a single, then Serial Transmit with the
    header.
- saves a **copy** of each model (suffix `_UART`), with:
  - the SPI block replaced by UART Receive;
  - `Feedback DAC1` replaced by UART Send (the signal into it was already
    `u` in volts);
  - the system-ready GPIO Write removed;
  - the pendulum counts-per-rev gain corrected to 1968.
- leaves the original models untouched.

The script is built for **R2025b**, which the team uses. The copies are
generated by code, not saved from a newer release.

**Prerequisite:** this PC has MATLAB R2026b without Simulink or the
Raspberry Pi support package. Install both, or run the script on a team PC.

## Testing

- **Firmware:** compiles with `arduino-cli` (`esp32:esp32:nodemcu-32s`). The
  rig check follows.
- **Python:** pytest for `pilink.py` (encode/decode, checksum, header
  resync) and for the daemon's DAC-absent path (I2C mocked).
- **Rig check, in this order, each logged over USB:**
  1. **SwingBal, FreeSwing and Friction** still run from the console
     (regression).
  2. `uart_link_check.py` with `u = 0`:
     - the loop holds 1 kHz;
     - the round trip is under 0.5 ms median;
     - there are no checksum failures over 60 s;
     - the counts match the USB `D` lines.
  3. `uart_link_check.py --u-amp 0.8`: the cart moves both ways and `D`
     shows the commanded `u`.
  4. **Timeouts:**
     - kill `uart_link_check.py` mid-run: `run end timeout` within 50 ms of
       the last command, motor off;
     - start without the Pi script: `run end timeout` after 30 s.
  5. **Stops:** the Pi stop pulse, the STOP button and the soft limit (by
     commanding a steady `u`) each end the run with the right reason.
  6. **Simulink:** one `_UART` model launched by the daemon through MQTT
     (the first model to try is chosen during part C).

## Build order

| Part | Contents | Done when |
|---|---|---|
| A | ESP32: Pi GPIO lines, `pi_link.ino`, `exp_simulink.ino`, v11.2 | Compiles; rig checks 1 and 4 (button/console side) pass |
| B | `picontrol/pilink.py` + tests, `uart_link_check.py` | Tests pass; rig checks 2–5 pass on the Pi |
| C | `make_uart_models.m`, `_UART` model copies | A copy builds for the Pi; rig check 6 runs |
| D | `dac_daemon.py` DAC-absent path | Test passes; the daemon runs clean without the MCP4725 |
| E | Docs: single-board + Pi section in `docs/esp32.md` and `docs/raspberry-pis.md`; mark experiments-spec stages 3–8 superseded | — |

A and B can be checked together with the Pi but no Simulink. C needs the
MATLAB installs.

## Risks and notes

- **Motor noise on the UART:** it corrupted the old board-to-board UART.
  The measurement has a checksum. The command has none (approved 6-byte
  format) but is range-checked, and a garbled float almost always lands
  outside ±5 V or is NaN. If the drop counter shows corrupted commands
  getting through, add a checksum byte (7-byte command) in both
  `pilink.py` and the Simulink pack block.
- **Simulink serial and 921600 baud:** if the Pi support package can't
  set 921600 on `/dev/ttyAMA0`, use 460800 (round trip about 0.4 ms). Below
  that a 1 ms step is too tight. The fallback is a 2 ms step in the `_UART`
  copies, but only for models whose discrete blocks tolerate the change.
  Check this in part C.
- **1 kHz on Linux:** Simulink overruns on the Pi mean late commands, not
  wrong ones. The 50 ms timeout only catches a stalled model.
  `uart_link_check.py` reports the jitter (max round trip) to show the
  headroom.
- **Re-tuning:** the models' gains were tuned for Feedback's amplifier and
  the old DAC path with its PWM cap of 47. Here ±2.5 V maps to PWM 180 with
  compensation, so expect to lower the gains at first.
- **124° pendulum-encoder dead zone:** unchanged, and it affects models that
  swing through it.
