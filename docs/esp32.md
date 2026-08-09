# ESP32 Firmware

Two ESP32s split the real-time work. Neither has WiFi or MQTT — they are wired
peripherals of the control Pi.

| | ESP32 #1 | ESP32 #2 |
|---|---|---|
| Sketch | `esp32_encoder_v9.ino` | `esp32_motor_v9.5.ino` |
| Version | 9.0 | 9.5 |
| Job | Read both quadrature encoders | Drive the motor, home the cart |
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

Sends cart position every `UART_INTERVAL_MS = 4` (250 Hz) as a 3-byte framed
packet:

```
0xFF | pos_low | pos_high      → int16 cart encoder count
```

Receives a single byte `0xAA` back, meaning homing complete, which sets
`FLAG_HOMING_COMPLETE` in the SPI status byte.

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
4. `MAX_PWM = 80` (of 255) caps the motor at roughly a third of full power.
5. `DEADBAND = 0.05 V` around the 2.5 V neutral stops jitter from creeping the
   cart.
6. The Pi's stop trigger on GPIO 5 mirrors the physical STOP button exactly —
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
5. Serial2.write(0xAA)  → tell ESP32 #1
6. GPIO 18 HIGH         → tell the Pi
```

Each phase first waits for the *currently held* limit switch to release before
treating the opposite switch as an error. Without that, starting a homing run
with the cart already parked on a limit would immediately fail.

Step 4 ramps PWM down as the cart nears centre
(`map(|error|, CENTER_TOLERANCE, 500, 20, HOMING_PWM)`), which avoids
overshooting past the tolerance band and oscillating.

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

## Handoff summary

```
ESP32 #1 ──0xAA (UART)──► sets FLAG_HOMING_COMPLETE in SPI byte 8 ──► Pi
ESP32 #2 ──GPIO 18 HIGH─────────────────────────────────────────────► Pi GPIO 13
```

Homing completion is reported over **two independent paths**. `dac_daemon.py`
waits on the GPIO 13 line; the SPI status bit is available to the Simulink
model.
