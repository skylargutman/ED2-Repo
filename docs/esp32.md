# ESP32 Firmware

Two ESP32s split the real-time work. Neither has WiFi or MQTT — they are wired
peripherals of the control Pi.

| | ESP32 #1 | ESP32 #2 |
|---|---|---|
| Sketch | `cart/esp32_encoder_v9/esp32_encoder_v9.ino` | `cart/cart_control/cart_control.ino` |
| Version | 9.2 | 10.0 |
| Job | Read both quadrature encoders | Drive the motor, home the cart, standalone swing-up/balance |
| To the Pi | SPI slave (data) | GPIO triggers in, 1 signal out |
| To each other | UART2 @ GPIO 16/17 | UART2 @ GPIO 16/17 |

Together they replace the original MCP23017 / PCI-1711 data acquisition path
from the Feedback Instruments rig.

---

# ESP32 #1 — Encoder

Reads two quadrature encoders using the ESP32's hardware **PCNT** (pulse
counter) peripheral, so counting costs no CPU time and never misses edges.

## Pins

| GPIO | Function |
|---|---|
| 39 / 36 | Motor encoder A / B |
| 35 / 34 | Pendulum encoder A / B |
| 19 | SPI MISO — data to Pi |
| 23 | SPI MOSI — unused, Pi sends nothing |
| 18 | SPI SCLK |
| 5 | SPI CS |
| 17 / 16 | UART2 TX / RX to ESP32 #2 |

GPIO 34–39 are input-only on the ESP32 — appropriate for encoder inputs, and
they cannot be accidentally driven.

## SPI packet — 10 bytes, CS-framed

| Byte | Contents |
|---|---|
| 0–3 | motor count — int32, little-endian |
| 4–7 | pendulum count — int32, little-endian |
| 8 | status flags |
| 9 | XOR checksum of bytes 0–8 |

Status flag bits:

```
bit 0  FLAG_SYSTEM_READY      0x01
bit 1  FLAG_HOMING_COMPLETE   0x02
```

The checksum matters: a mis-clocked SPI read that silently shifted the position
by a byte would feed a garbage setpoint straight into a controller driving a
real motor. The consumer must verify byte 9 and discard bad frames.

Buffers are declared `DMA_ATTR` because the ESP-IDF SPI slave driver requires
DMA-capable, 32-bit-aligned memory.

## UART to ESP32 #2

Sends cart and pendulum position every `UART_INTERVAL_MS = 2` (500 Hz) as a
7-byte framed packet (encoder v9.2 / motor v10.0 and later — **flash both
boards together**):

```
0xFF | m[6:0] | m[13:7] | m[15:14] | p[6:0] | p[13:7] | p[15:14]
       int16 cart count              low 16 bits of pendulum count
```

ESP32 #2 unwraps the pendulum value into a 32-bit count. The PCNT units run
with `accum_count`, so the counts (and the SPI values to the Pi) keep going
past ±32767 instead of resetting to 0.

`ENC_DIAG 1` adds a second debug line with per-channel edge counts and levels
— the quickest way to tell a dead encoder channel from a wiring fault. A count
that only moves between 0 and 1 means one channel is not toggling (found on the
pendulum B channel: a dead level-shifter channel, replaced by a
4.7k-to-5V pull-up + 10k/22k divider into GPIO 34).

A partial version of the same fault shows up only in part of the rotation: the
pendulum angle freezes at one angle (flickering by one count) mid-swing, then
resumes — behind by the counts it missed if the pendulum went through the zone.
Seen at ~56° from hanging, bob toward +x, in a STEP log (`th` stuck at 124.02 /
124.20 while `w` was −6.7 rad/s). Symptoms downstream: the upright reference
drifts, and a caught pendulum is balanced about the wrong angle.

Data bytes are 7-bit so they can never equal the `0xFF` header. The old 3-byte
format (`0xFF | pos_low | pos_high`) put `0xFF` in the high byte for every
position from −256 to −1; the motor board read that as a new header, its
position froze in that range, and centering hunted back and forth whenever the
centre landed near 0.

Receives the 4-byte sequence `AA 55 A5 5A` back, meaning homing complete,
which sets `FLAG_HOMING_COMPLETE` in the SPI status byte and **zeroes both
encoder counts** — so after homing, count 0 is the centre.

This used to be a single `0xAA` byte. Motor noise on the line produces garbage
bytes, 1 in 256 of them is `0xAA`, and the counts were being zeroed mid-run
("Homing complete confirmed" three times in 3 s): the cart position and
pendulum angle jumped under the controller. The debug line now prints
`UART noise:` — the number of bytes received that were not part of a message.
It should stay at 0; if it climbs while the motor runs, that wire needs
shielding, a twisted pair with ground, or a series resistor.

---

# ESP32 #2 — Motor Control

Drives a **BTS7960 (IBT-2)** H-bridge. Reads its command from an **ADS1115**
ADC watching the Pi's MCP4725 DAC output — so the control signal path is
Simulink → DAC → analog → ADC → PWM.

## Pins

| GPIO | Function |
|---|---|
| 25 / 26 | RPWM / LPWM to IBT-2 |
| 27 | START button (`INPUT_PULLUP`, active LOW) |
| 14 | STOP button (`INPUT_PULLUP`, active LOW) |
| 32 / 33 | Left / right limit switch (`INPUT_PULLUP`, active LOW) |
| 21 / 22 | I²C SDA / SCL to ADS1115 |
| 17 / 16 | UART2 TX / RX to ESP32 #1 |
| 4 | Pi homing-start trigger (`INPUT_PULLDOWN`, active HIGH) |
| 5 | Pi stop trigger (`INPUT_PULLDOWN`, active HIGH) |
| 18 | Homing-complete output → Pi GPIO 13 |

Buttons are active-LOW with internal pullups (a disconnected wire reads "not
pressed"). The Pi triggers are active-HIGH with pulldowns, so an unpowered or
disconnected Pi cannot assert them.

## Safety behaviour

This firmware is the last line of defence — worth understanding before changing
anything:

1. **Motor outputs are driven LOW at boot**, before anything else, to prevent
   runaway on power-up.
2. **Limit switches override everything** in `loop()`, except during homing
   (where hitting a limit is the expected outcome).
3. `systemEnabled` starts false. Nothing moves until START is pressed or the Pi
   pulses GPIO 4.
4. `MAX_PWM = 47` (of 255) caps the motor at under a fifth of full power.
5. After homing, the motor ignores the DAC until it reads the 2.5 V neutral,
   and stops again if the signal sits near 0 V for 200 ms. A disconnected
   Pi/DAC reads ~0 V, which would otherwise mean "full speed left".
6. `DEADBAND = 0.05 V` around the 2.5 V neutral stops jitter from creeping the
   cart.
7. The Pi's stop trigger on GPIO 5 mirrors the physical STOP button exactly —
   remote and local stops go down the same path.

## Homing routine — `doHoming()`

Runs before every experiment. The controller needs a centred cart and a known
encoder reference.

```
1. Drive LEFT  until left limit       → record left_pos
2. Drive RIGHT until right limit      → record right_pos
3. track_total  = |right_pos - left_pos|
   track_center = left_pos + track_total/2
   reject if track_total < MIN_TRACK_COUNTS (100)  → encoder wiring fault
4. Drive to centre with proportional speed,
   stopping within CENTER_TOLERANCE (20 counts ≈ 1.5 mm)
5. Wait for the cart to stop coasting (settle)
6. Serial2.write(AA 55 A5 5A)  → tell ESP32 #1 (it zeroes its counts here)
7. GPIO 18 HIGH         → tell the Pi
```

Homing speed is set per direction: `HOMING_PWM_LEFT = 50`,
`HOMING_PWM_RIGHT = 60`. On this rig the cart would not keep moving right at 45
and stalled intermittently going left at 45. Tune each to the lowest value that
produces no `Stall` lines in the serial log. Needing more drive to the right
points at a mechanical or driver asymmetry that also affects experiments.

If the encoder moves less than 5 counts in 400 ms, the firmware applies a short
breakaway kick (`BREAKAWAY_PWM = 70`), and gives up with an error after 5 kicks
without real progress. Every phase also has a 30 s timeout.

Each phase first waits for the *currently held* limit switch to release before
treating the opposite switch as an error. Without that, starting a homing run
with the cart already parked on a limit would immediately fail.

Step 4 ramps PWM down as the cart nears centre
(`map(|error|, CENTER_TOLERANCE, 500, CENTER_MIN_PWM, homingPwm(dir))`), which
avoids overshooting past the tolerance band. `CENTER_MIN_PWM = 40` (+10 when
moving right) — it must stay above the stall threshold; at 20 the cart crawled
the last few hundred counts on breakaway kicks.

Any failure returns false, leaves the motor stopped, and requires a START press
to retry. The Pi meanwhile times out after `HOMING_TIMEOUT_S = 30`.

## Normal operation

```c
raw       = ads.readADC_SingleEnded(0);
voltage   = ads.computeVolts(raw);
deviation = voltage - 2.5;          // NEUTRAL_VOLTAGE

|deviation| < 0.05  → stopMotor()   // DEADBAND
deviation > 0       → driveRight(map(...) capped at MAX_PWM)
deviation < 0       → driveLeft(...)
```

Loop period is 10 ms (100 Hz).

The 2.5 V neutral convention is shared across the system: `dac_daemon.py` sets
the DAC to 2.5 V at startup and whenever no model is running, so the resting
state of the whole chain is "motor stopped".

## Standalone swing-up + balance (no Pi)

With `STANDALONE_SWINGUP 1` the motor board ignores the DAC and runs its own
controller at 500 Hz (one step per UART packet) after homing:

```
HANG     motor off; average the pendulum count over exactly 2 swing periods
         (2.24 s) — whole swings average to the centre wherever the window
         starts. Accepted if the swing is < 600 counts (~±54°). The pendulum
         barely loses energy, so waiting for it to stop would take minutes,
         and the middle of min/max over 1.5 s was off by ~3.4°
CHECK    (SIGN_CHECK 1 only) motor off, print angle/position forever
STEP     (STEP_TEST 1 only) speed loop test: ±0.3 m/s for 0.6 s each,
         3 cycles, printing every 20 ms, then DONE (motor off)
SWING    energy pumping on the speed loop: cart speed command SWING_V
         (0.4 m/s) opposite to the bob's side, reversing as the bob passes
         the bottom (SWING_PHASE_DEG), scaled by (E_target − E)/0.6 clamped
         to ±1 — proportional, and negative (removing energy) above the
         target of +0.02. E is the height of the last swing end (`Eend` =
         cos θ_end − 1, exact), or the running E within 30° of upright (also
         covers full rotations, which have no swing end); the running E reads
         ~0.25 high at the bottom where the reversal happens. A 50 % minimum
         push arrived too fast to catch, then pumped a spinning pendulum up
         to 16 rad/s. Only catches arrivals slower
         than 2 rad/s (faster ones need > 0.8 m/s of cart speed change),
         minus a pull back to centre (K_X_SWING m/s per m). No outward speed
         beyond 60 % of the half-track (until back inside 70 % of that).
         Reversals are triggered by the bob CROSSING sides; with no crossing
         for 0.8 s while nearly at rest (E < −1.9) the cart reverses on a
         timer to start a swing. Crossing-based, so a steady angle offset
         can't cancel the timer's reversal. The E gate matters: big swings
         slow down near the top, crossings get > 0.8 s apart, and an ungated
         timer reversed early every swing (stalled ~50° from upright).
BALANCE  entered inside ±0.25 rad (and < 4 rad/s). Commands a cart
         acceleration, integrated into the speed-loop command:
         a = 62.8·θ + 11.2·θ' + 19.9·x + 15.9·v   (m/s², BAL_POLE 5)
         back to SWING if |θ| > 0.6 rad
```

The balance gains are computed at startup (`balanceGains()`, printed after
homing) to put all four closed-loop poles at `−BAL_POLE`, from the pendulum
model `θ'' = ω0²·θ − (ω0²/g)·a`, which holds for any mass distribution. A
simulation from a logged catch (θ 10°, θ' −0.86 rad/s, cart −50 mm at
−0.46 m/s) settles in ~1 s with ≤ 225 mm cart travel for speed-loop lags of
30–100 ms at pole 5; pole 7 falls at 100 ms lag.

The Feedback SwingHoldPendulum PID gains were tried first and do **not**
transfer: its cart signal is scaled by a negative factor (`Counts->Meters =
−0.156/2048`) and it drove a force-like amplifier. On this rig they caught the
pendulum and then drove the cart away from it into the limit.

**The pendulum encoder gives 1968 counts per revolution**, not the 2000 a
500-line ×4 encoder suggests (measured to a mark: 1967/turn backward,
1969/turn forward). With 2000, upright was 2.9° off, and each full rotation
during swing-up added another 5.9°. To re-measure: run `cart/serial_log.py`,
hold the pendulum at a mark for 3 s, turn N turns to the mark, hold 3 s, and
divide the change in `P:` by N.

**First successful balance** (after the CPR fix): 18 s, angle within ±0.5°,
cart within ±5 mm — but parked at −302 mm with the angle reading +5.5°,
i.e. a 5.5° upright offset of unknown origin (no full rotations that run). The
simulation reproduces it exactly (parks at −303 mm). Balance therefore
**self-trims**: 2 s after a catch, the upright trim drifts at
`−0.04 · x` rad/s until the cart parks at centre (~30 s for 5.5°; faster
rates fight the balance and went unstable in simulation). The trim is printed
on `BAL` lines — a consistent value across runs is a constant bias worth
putting in `PEND_TRIM_DEG`.

An upright offset δ (wrong `PEND_COUNTS_PER_REV` or hanging reference) makes
the cart settle off-centre by about `62.8·δ / 19.9` — ~160 mm per 3° — so a
balanced cart that parks well off-centre means trim `PEND_TRIM_DEG`.

Conventions: `θ > 0` = top leans toward the right limit, `x > 0` = cart right
of centre, `u > 0` = drive right. `cartDir` is learned during homing;
`PEND_SIGN` must be set by hand (below). The motor command `u` maps to PWM
`RUN_MIN_PWM`…`SPEED_MAX_PWM` (30…180, +10 when driving right), starting just
under the stall PWM so small corrections still move the cart.

### Cart speed loop

Swing-up commands a cart **speed**, not a PWM:
`u = V_FF·v_ref + KV_P·(v_ref − v)` (0.25 and 1.5), mapped to PWM
30…`SPEED_MAX_PWM` (180). Balance still maps `u` straight to PWM 30…120.

Why: driven by PWM directly the cart took about half a pendulum swing to
reverse. Every PWM-based swing-up fought that lag — flipping at the bottom
damped the swing, flipping at the swing ends pumped but ran the cart across
the whole track, and every centring pull strong enough to stop that braked the
cart mid-swing and took the energy back out (best reached ~75° off the bottom).
With the loop the cart reverses quickly, the reversal can land when the bob is
fastest, and travel is fixed by the commanded speed.

Safety: limit switches and STOP behave as before (immediate motor stop).
Software faults — the cart would pass 85 % of the half-track, predicted from
`|x| + v²/(2·3 m/s²)` while moving outward, or exceeds 1.8 m/s — **brake**
through the speed loop (`BRAKE` state, ≤ 0.6 s) and then disable. The first
version only cut the motor once the cart was already at 85 %; coasting slows
only ~1.3 m/s², so at 0.8 m/s it slammed into the end rail. All need START to
re-home. In run mode a limit switch must read pressed for 5 ms
(`LIMIT_CONFIRM_MS`): full-power reversals put noise spikes on the limit
inputs and caused false hits near the centre of the track.

### Bring-up

1. Flash **both** boards (encoder v9.2 + motor v10.0), `SIGN_CHECK 1`.
2. Press START, let it home. It prints `>> CHECK` once the pendulum hangs still.
3. Push the cart by hand toward the **right** limit: `x` must go positive.
   (If not, homing's `cartDir` is wrong — check the limit switch wiring.)
4. Lift the pendulum to upright and lean the top toward the **right**: `th`
   must read a small **positive** angle, near 0 when straight up. If it is
   negative, set `PEND_SIGN -1`. If upright reads far from 0, fix
   `PEND_COUNTS_PER_REV`.
   Reading this by eye is easy to get wrong (it was, on this rig). The
   reliable check is physics: in any SWING/STEP log, when the cart
   **accelerates** toward +x, a hanging bob must swing toward −x (`th`
   moving from ±180° toward **negative** values, i.e. −170°, −160°…).
   If it moves toward +170°, +160°… instead, flip `PEND_SIGN`. This rig
   needs `PEND_SIGN −1`.
5. Time 10 small swings, set `PEND_OMEGA0 = 2π / (time / 10)`.
6. Set `SIGN_CHECK 0`, `STEP_TEST 1`, flash, keep a hand on STOP. Check the
   STEP lines: `v` should reach ±0.3 within ~0.1 s of each `vr` flip and hold
   there without oscillating.
7. Set `STEP_TEST 0`, flash: swing-up.

### Tuning

| Symptom | Change |
|---|---|
| Step test: `v` slow to reach `vr`, or stays short of it | raise `KV_P` (and `V_FF` if it stays short) |
| Step test: `v` overshoots and rings around `vr` | lower `KV_P` or raise `VEL_FILTER_S` |
| Swing never grows | raise `SWING_V`; if the cart lags the reversals, raise `SWING_PHASE_DEG` (10–20°) |
| Swing grows but the cart runs toward the ends | lower `SWING_V` or raise `K_X_SWING` |
| Stalls a few tens of degrees short of upright (`Eend` stuck below 0) | lower `SWING_E_SLOW` (stronger push for a small shortfall) or raise `SWING_E_TARGET` a little |
| Reaches the top too fast to catch (`w` > 2 at ±14°) | raise `SWING_E_SLOW` or lower `SWING_E_TARGET`; don't raise `CATCH_RATE` — fast catches saturate the cart |
| Spins over the top repeatedly | check `Eend` follows the running `E` near the top (`SWING_TOP_ZONE`); the negative push should slow the spin within a few turns |
| Catches then falls within ~0.5 s | check `PEND_SIGN`; raise `BAL_POLE` (5 → 6) if too soft |
| Balances but oscillates faster and faster | lower `BAL_POLE` (speed-loop lag too large for it) or raise `VEL_FILTER_S` |
| Balances but parks off-centre / drifts to one end | check `PEND_COUNTS_PER_REV` (1968), then adjust `PEND_TRIM_DEG` (~160 mm per 3°) |

## Handoff summary

```
ESP32 #1 ──AA 55 A5 5A (UART)──► sets FLAG_HOMING_COMPLETE in SPI byte 8 ──► Pi
ESP32 #2 ──GPIO 18 HIGH─────────────────────────────────────────────► Pi GPIO 13
```

Homing completion is reported over **two independent paths**. `dac_daemon.py`
waits on the GPIO 13 line; the SPI status bit is available to the Simulink
model.
