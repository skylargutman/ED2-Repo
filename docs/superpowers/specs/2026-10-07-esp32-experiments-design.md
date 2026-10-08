# ESP32 Experiments in the Remote Console: Design

Date: 2026-10-07
Status: draft for review

## Goal

Students can run the Feedback 33-936 lab exercises on the single-board rig
(one ESP32, no Raspberry Pi, no DAC, no Simulink) from the `pendulum-remote`
console. They pick an experiment, set its parameters, run it, watch it live,
and download the run data for analysis and controller design in MATLAB. This
follows the workflow of the Feedback manual *33-936S Control Experiments*.

### What was decided

- Experiments are **built-in modes in the ESP32 firmware**. One flash holds all
  of them. They are picked and tuned at runtime. Students change parameters;
  they do not build new controllers (a later "external controller" mode can add
  that without redoing this work).
- Frontend: the **`pendulum-remote` console** (bridge + plain HTML/JS). The
  Django site is not changed.
- Every run is downloadable as **CSV and `.mat`**.
- All **11 Feedback experiments**, built in the order of the table below, plus
  the existing swing-up and balance.
- Run data is streamed at **500 Hz over USB serial at 921600 baud**.

### Out of scope

- Django site changes, MQTT, the Pi/Simulink path.
- Student-built controllers (Simulink, or the future external-controller mode).
- Pushing to GitHub (IP hold). Commits stay local.

## Experiments

Each run starts with homing and the hanging reference, as now. Units follow
the Feedback manual so its MATLAB steps carry over: cart position `x` in
metres, pendulum angle `theta` in radians, motor command `u` in volts (±2.5).
Gains have the manual's meaning, but the values must be re-tuned for this
motor (as the manual's identification exercises intend). Feedback's example
values (for example cart PID P = 27.84, I = 50, D = 3.9) are reference only. Each
experiment ships with conservative defaults confirmed on the rig.

| # | Console name | Feedback model / exercise | Behaviour on the ESP32 | Student parameters |
|---|---|---|---|---|
| 1 | Pendulum free swing | PendulumTest | Motor off. Record while the student swings and releases the pendulum. Ends after a set time or on STOP. | duration |
| 2 | Static friction | PendulumFriction, Ex. 3 | Ramp `u` up from 0 until the cart moves (encoder change past a threshold), stop, settle, repeat in the other direction. Report both breakaway voltages. They become the shared friction compensation (stored in flash, `defaults` resets them). | ramp rate (V/s) |
| 3 | Cart identification | CartIdent, Ex. 4 | 20 s sum of sines on `u`, optional added random signal. | amplitude, noise on/off, noise amplitude |
| 4 | Crane identification | CraneIdent, Ex. 5 | Sum of sines on `u` for 10 s, then motor off while the pendulum swings freely; total 100 s as in the manual. | amplitude, total duration |
| 5 | Inverted identification | InvPendIdent, Ex. 6 | Swing up and balance (existing controller), then add a sum-of-sines excitation `r` to the balance command for 40 s. Records `u` and `r`. | excitation amplitude, duration |
| 6 | Cart PID | CartControl, Ex. 8 | PID on cart position, with the derivative filtered (as the manual requires). Follows a sine or square setpoint. | P, I, D, setpoint shape, amplitude, frequency |
| 7 | Pendulum swing-up | PendSwingUp, Ex. 10 | The manual's rule (Fig. 25): constant `u = ±ua`, reversed when the angular speed crosses zero in the lower zone and when crossing the zone border. Ends when within ±θs of upright (no hold). | ua (≤ 1 V, as the manual cautions), θs |
| 8 | Pendulum stabilisation (PD) | PendStabPD, Ex. 12 | Motor off until the student lifts the pendulum to within θb of upright, then PD on angle + PD on cart (Fig. 26). Returns to motor off if it falls past a drop angle. | angle P/D, cart P/D, θb |
| 9 | Crane control | CraneStab, Ex. 13 | Pendulum hanging. The cart follows a setpoint while PD on angle and cart damps the swing. | angle P/D, cart P/D, setpoint shape, amplitude, frequency |
| 10 | Swing up and hold | SwingHoldPendulum(Extra), Ex. 14 | Experiment 7's swing-up, then experiment 8's PD hold, with the "Extra" cart-position term during swing-up. | both sets + cart term |
| 11 | Up and down | UpDownPendulum, Ex. 15 | Repeats until the run time ends: swing up and hold for T_up, release, crane-stabilise at x = 0 / hanging, follow a sine for T_down. | T_up, T_down, run time, both controllers' gains |
| — | Swing-up & balance | (current firmware) | Energy swing-up + pole-placement balance on the cart speed loop, unchanged. | existing parameters |

Experiments 2–4, 6–11 drive the motor command `u` directly (like Feedback),
not the cart speed loop. Experiment 5 and the current mode keep using the
speed-loop balance, because it is the proven stabilising controller on this rig.

## Firmware (`cart/cart_single/`)

### File layout

Arduino compiles every `.ino/.cpp/.h` in the sketch folder.

- `cart_single.ino`: `setup()` and `loop()` only.
- `hardware.*`: pins, encoders (PCNT), motor output, limit switches, homing.
- `safety.*`: soft-limit braking, overspeed, limit-switch and STOP handling.
- `protocol.*`: serial command parser, parameter table, the `D` data line.
- `motor_cmd.*`: volts → PWM (below).
- `exp_*.cpp`: one file per experiment, plus `exp_swingbal.cpp` for the current
  swing-up and balance, moved without behaviour changes.

### Experiment interface

```cpp
struct Experiment {
  const char* name;        // e.g. "CartPID" (no spaces; used in commands and file names)
  const char* title;       // e.g. "Cart PID (Ex. 8)"
  Param*      params;      // this experiment's [online] settings
  int         nParams;
  void  (*start)();        // after homing + hanging reference
  bool  (*step)(float dt); // every 2 ms; returns false when finished
  float ref;               // current setpoint (for the D line), NAN if none
};
```

Experiments read the shared state (`x`, `xDot`, `theta`, `thetaDot`, `energy`),
which is computed once per step as now. They output through `motorVolts(u)`
(direct) or `speedDrive(v)` (speed loop). They never touch pins or safety
checks.

### Motor command

`motorVolts(u)`: clamp `u` to ±2.5 V. Below a small deadband, motor off. Above
it, add the friction-compensation offset for the direction of travel and map to
PWM within the existing caps (`RUN_MIN_PWM` … `SPEED_MAX_PWM`, +right offset).
Until experiment 2 has been run, the offsets default to the values implied by
the current `RUN_MIN_PWM` / `RUN_RIGHT_EXTRA_PWM`. A student's gains can
therefore never exceed the motor limits.

### Safety

Outside experiment code, applied every step in every experiment:

- **Soft limit:** predicted stopping distance passes 85 % of the half-track.
  Brake through the speed loop, then stop.
- **Overspeed:** cart speed above 1.8 m/s.
- **Limit switches:** 5 ms confirmation. Motor off immediately.
- **STOP:** button or serial, also during homing.

A run ended by safety reports the reason on its `run end` line. Automatic
re-homing after a soft limit stays limited to the current swing-up & balance
mode.

### Serial protocol (USB, 921600 baud)

Existing replies keep their prefixes (`ok`, `err`, `param`, `params end`,
`status`).

| Command | Reply / effect |
|---|---|
| `exp list` | `exp NAME \| title` per experiment, then `exp end` |
| `exp select NAME` | `ok exp NAME` (idle only) or `err …` |
| `params` | Shared settings + the selected experiment's, same `param` line format as now |
| `set NAME VALUE` | As now; limits enforced on the ESP32 |
| `run` (alias `home`) | Home, hanging reference, then run the selected experiment |
| `stop`, `defaults`, `io` | As now |
| `status` | `status STATE EXP` |

Run output:

```
run start NAME P=27.8 I=50 D=3.9 ...      (all current params)
D <t_ms> <x_m> <theta_rad> <u_V> <ref>     (every 2 ms)
result <key> <value>                        (e.g. result friction_pos_V 0.21)
run end <reason>                           (done | stop | soft-limit | overspeed | limit-switch | error)
```

`theta` in `D` lines is **continuous** (not wrapped to ±π), 0 = upright and
+π = hanging at run start, matching the manual's θ = π hanging convention. A
wrapped angle would jump between +π and −π on every crane swing and break
identification. `ref` is `nan` when the experiment has no setpoint. Experiment 5
also needs `r`, so its `D` lines carry one extra trailing field.

Readable event lines (`>> STATE`, fault messages) still appear between `D`
lines. The existing `SWING`/`BAL` telemetry lines are kept for the current mode.
`cart/serial_log.py` switches to 921600 baud; its rest-drift helper keeps
working.

## Bridge (`pendulum-remote/bridge/bridge.py`)

- **Serial:** opens at 921600 baud. Auto-detect as now. Sends `exp list` and
  `params` on connect and after `exp select`.
- **Parsing:** `exp`, `param`, `status`, `run start`, `D`, `result` and
  `run end` lines, plus the existing ones.
- **Recording:**
  - `run start` opens a recording with the experiment name and parameters.
  - Every `D` line is stored.
  - Every 10th goes to browsers (50 Hz).
  - `result` lines are attached to the run.
  - `run end` writes `runs/<YYYY-MM-DD_HHMMSS>_<NAME>.csv` and `.mat`.
  - A serial drop mid-run saves what was received, with reason `disconnected`.
- **CSV:** header `t,x,theta,u,ref` (+`r` for experiment 5), `t` in seconds.
  Parameters and results go in `#` comment lines at the top.
- **`.mat`:** MAT-file level 5, written by a small built-in writer (no SciPy at
  runtime). Variables:
  - `t x theta u ref` (+`r`): column vectors;
  - `simout = [x theta u t ref]`;
  - `params`: a struct;
  - `results`: a struct.
- **HTTP:** `GET /runs` lists the saved runs; `GET /runs/<file>` downloads one
  (path checked against the `runs/` folder).
- **Control:** the control lock, the heartbeat stop and stop-by-anyone are
  unchanged. `exp select`, `set`, `defaults` and `run` need control.
- **Mock mode:** a simulated rig that answers `exp list`/`params` and plays a
  plausible run (homing delay → `run start` → `D` lines → `run end`), so the
  console can be built without hardware. It does not simulate the controllers.

## Console (`pendulum-remote/frontend/`)

- **Experiment picker:**
  - grouped *Identification* (1–5), *Control* (6–9), *Combined* (10–11), and
    *Swing-up & balance*;
  - each entry shows its manual exercise number and a one-line description;
  - selecting needs control and an idle rig.
- **Parameters:** generated from `params` as now, so adding an experiment needs
  no frontend change.
- **Charts:**
  - angle and cart position as now, plus a new **motor command `u`** chart;
  - the setpoint `ref` is drawn over cart position when present;
  - the angle chart uses the continuous angle in experiments 3–4 and 9 (hanging
    work).
- **Start dialog:** states what will move for the selected experiment.
- **Runs panel:**
  - one row per run this session: experiment, start time, duration, end reason,
    results;
  - **CSV** and **.mat** download buttons;
  - also lists earlier runs from `GET /runs`.

## Testing

- **Firmware:**
  - every change compiles with `arduino-cli` (`esp32:esp32:nodemcu-32s`);
  - behaviour is confirmed on the rig from `cart/logs/` (student/user runs,
    Claude reads the logs);
  - control math lives in small pure functions (PID with filtered derivative,
    swing-up switching rule, volts → PWM, sum-of-sines);
  - an experiment is done only after a logged rig run with its default gains.
- **Bridge:** pytest, no hardware. Covers:
  - line parsing;
  - recorder (500 Hz in, 50 Hz out, `run end` and disconnect cases);
  - CSV contents;
  - `.mat` read back with `scipy.io.loadmat` (test-only dependency);
  - the `/runs` path check;
  - the control lock;
  - the heartbeat stop during a run.
- **Console:** checked in a browser against mock mode, then against the rig at
  each stage.

## Build order

Each stage ends with a working, logged run.

| Stage | Contents | Done when |
|---|---|---|
| 0 | Split `cart_single.ino` into files; 921600 baud; experiment framework + protocol; move swing-up & balance into `exp_swingbal.cpp`; set `PEND_TRIM_DEG` ≈ 5.5 | Swing-up & balance behaves as before, with `D` lines in the log |
| 1 | Bridge recording, CSV/`.mat`, `/runs`; console picker, `u` chart, runs panel; mock mode | A swing-up & balance run downloads and loads in MATLAB |
| 2 | Experiments 1 Free swing, 2 Static friction | Friction values reported and used by `motorVolts` |
| 3 | Experiments 3 Cart ID, 4 Crane ID, 5 Inverted ID | `.mat` files import into the System Identification Toolbox |
| 4 | Experiment 6 Cart PID | Cart tracks the setpoint with the default gains |
| 5 | Experiments 7 Swing-up, 8 PD stabilisation | Both work on the rig |
| 6 | Experiments 9 Crane control, 10 Swing up and hold | Both work on the rig |
| 7 | Experiment 11 Up and down | Full cycle repeats |
| 8 | Docs: single-board section in `docs/esp32.md`; short student guide per experiment | — |

## Risks and notes

- **921600 baud on the USB-serial chip:** the earlier 64-byte garbled lines
  (USB, motor off) may get more frequent. The bridge drops malformed `D` lines
  and counts them per run. If drops exceed ~0.1 %, fall back to 460800 baud and
  250 Hz `D` lines.
- **124° pendulum-encoder dead zone:** measured at ≤1° per run. It mainly
  affects experiments 4, 5 and 11, which swing through it repeatedly. The data
  is recorded as is; the student guide notes it.
- **Direct-drive experiments (`u` in volts) on this motor** behave differently
  from Feedback's force amplifier. Default gains come from rig runs, not the
  manual.
- **`PEND_TRIM_DEG`** (≈5.5° upright offset): set it at stage 0 so experiments 8
  and 10 start from the right upright.
