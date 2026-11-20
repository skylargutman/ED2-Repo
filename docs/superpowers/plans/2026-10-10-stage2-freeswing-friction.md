# Stage 2: Free Swing and Static Friction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the first two Feedback experiments to the single-board rig: **1 Pendulum free swing** (motor off, record the decay) and **2 Static friction** (ramp the motor until the cart moves, both directions). The friction results are stored in flash and used as the friction compensation of `motorVolts()`, which every later direct-drive experiment uses.

**Architecture:**
- **Firmware.** Each experiment is a new firmware tab implementing the existing `Experiment` interface (`cart/cart_single`, see stage 0–1). A new `friction.ino` tab owns the volts → PWM mapping (raw and compensated) and the flash storage (ESP32 `Preferences`).
- **Bridge and console.** No protocol changes: they already list experiments from `exp list`, record runs, and show `result` lines on run rows. They only gain names, help text and a mock for the new experiments.

**Tech Stack:** Arduino-ESP32 core 3.3.7 (`esp32:esp32:nodemcu-32s`), `Preferences.h`; Python 3.13 + pytest for the mock; vanilla JS console.

**Spec:** `docs/superpowers/specs/2026-10-07-esp32-experiments-design.md` (ED2-Repo), rows 1–2 of the experiment table, the "Motor command" section, and build-order stage 2. Builds on `docs/superpowers/plans/2026-10-07-experiments-foundation.md` (stages 0–1, done).

**Repositories:**
- **A** = `C:\Users\super\Documents\ED2\ED2-Repo`, branch `cart-standalone-swingup` (GitHub: skylargutman/ED2-Repo)
- **B** = `C:\Users\super\Documents\ED2\pendulum-remote`, branch `experiments` (GitHub: TitanSquish/pendulum-remote)

## Global Constraints

- Commit locally after each task. **Push only when the user asks** (the IP hold was lifted 2026-10-10, but pushes are still the user's call).
- Serial: 921600 baud; data lines `D <t_ms> <x_m> <theta_rad> <u_V> <ref>`; results as `result <key> <value>` between `run start` and `run end` (the bridge already records them).
- Motor command `u` is in volts, **±2.5 V = full scale** (`U_FULL_SCALE_V`), mapped to PWM up to `SPEED_MAX_PWM` (180).
- Experiment names (firmware ↔ console): **`FreeSwing`**, **`Friction`**. State names on `>>` / `status` lines: **`FREE`**, **`FRICTION`** (single words: the bridge takes the first word after `status`).
- Safety (soft limit, overspeed, limit switches, STOP) stays in `control.ino` / `loop()`, never in experiment code.
- Parameter limits are enforced on the ESP32 (`PARAMS` table).
- Console copy: no em/en dashes in UI text.
- Firmware compile command (PowerShell), used by every firmware task:
  ```powershell
  $cli = (Get-ChildItem "$env:LOCALAPPDATA\Programs\arduino-ide" -Filter arduino-cli.exe -Recurse | Select-Object -First 1).FullName; & $cli compile --fqbn esp32:esp32:nodemcu-32s --warnings default "C:\Users\super\Documents\ED2\ED2-Repo\cart\cart_single"
  ```
  Expected: ends with `Sketch uses ... bytes`, no `error:` lines.
- Firmware has no unit-test harness. Its gate is compile plus a logged rig run (Task 5), and the control math sits in small functions with comments giving the expected values.

## Review Focus

1. **The cart never moves during the friction test** (motor power off, cart blocked, belt slipping). The run must end with a clear message and must **not** overwrite the stored compensation. → Task 3 code (`frStep` "did not move" path), and the Task 5 rig check with the 24 V kill switch off.
2. **A measured breakaway above the allowed range** (stiff belt, > 1.5 V). It is clamped to the limit with a warning, never stored out of range. → Task 3 code (`frStore`), checked in the Task 5 log.
3. **First boot or invalid flash contents** (never measured, or a value outside 0–1.5 V). The compiled defaults are used and the boot banner says so. → Task 1 code (`loadFriction`), Task 5 rig check after reflashing.
4. **A state name the console doesn't know** (any future experiment). The console must show "Running" with settings locked, never "Ready" with Start enabled mid-run. → Task 4 Step 4 browser check.
5. **`defaults` after a friction measurement.** It restores the compiled values **and** clears flash, so a reboot does not bring the old measurement back. → Task 1 code, Task 5 rig check.

## Before you start

- Repo B has three uncommitted changes from 2026-10-10: the new `requirements.txt` and the edited `bridge/requirements.txt` and `bridge/requirements-dev.txt`. Ask the user whether to commit them first. If yes:
  ```bash
  cd /c/Users/super/Documents/ED2/pendulum-remote
  git add requirements.txt bridge/requirements.txt bridge/requirements-dev.txt
  git commit -m "requirements.txt at the repo root (uvicorn[standard] for WebSockets); bridge files point to it"
  ```

---

### Task 1: Friction compensation and flash storage (`friction.ino`)

**Files:**
- Create: `cart/cart_single/friction.ino`
- Modify: `cart/cart_single/cart_single.ino` (include, two `[online]` variables, two `PARAMS` rows, `loadFriction()` call in `setup()`)
- Modify: `cart/cart_single/control.ino` (remove the old `motorVolts`)
- Modify: `cart/cart_single/protocol.ino` (`set` and `defaults` persist / clear)

**Interfaces:**
- Consumes: `driveRight(int)`, `driveLeft(int)`, `stopMotor()` (hardware.ino); globals `uOut`, `U_FULL_SCALE_V`, `U_EPS`, `SPEED_MAX_PWM`, `RUN_MIN_PWM`, `RUN_RIGHT_EXTRA_PWM`.
- Produces:
  - `void motorRawVolts(float u)`: no compensation; `u` in volts, > 0 = drive right (+x). PWM = |u| / 2.5 × 180.
  - `void motorVolts(float u)`: compensated; output starts at `FRIC_POS_V` / `FRIC_NEG_V` and rises linearly to full scale at |u| = 2.5 V.
  - `void loadFriction()`, `void saveFriction()`, `void clearFriction()`.
  - Globals `float FRIC_POS_V`, `float FRIC_NEG_V` (volts).
  - Params `FRIC_POS_V`, `FRIC_NEG_V`, owned by experiment `"Friction"`.
  - For both motor functions, `uOut` = u / 2.5, so the D line's `u` column is the commanded volts.

- [ ] **Step 1: Add the include and the compensation variables (`cart_single.ino`)**

After the line `#include <WiFi.h>` add:

```cpp
#include <Preferences.h>
```

After the line `float V_MAX           = 0.8f;     // m/s: speed command cap   [online]` add:

```cpp

// --- Friction compensation for motorVolts() (measured by the Friction experiment) ---
// Breakaway motor command per direction, in volts on the motorRawVolts() scale
// (2.5 V = SPEED_MAX_PWM). Defaults equal the offsets the speed loop has always
// used (RUN_MIN_PWM, + RUN_RIGHT_EXTRA_PWM to the right): 0.556 / 0.417 V.
// Saved in flash when measured or set; reloaded at boot; "defaults" restores
// these compiled values and clears flash.
float FRIC_POS_V = (RUN_MIN_PWM + RUN_RIGHT_EXTRA_PWM) * U_FULL_SCALE_V / SPEED_MAX_PWM;   // [online]
float FRIC_NEG_V = RUN_MIN_PWM * U_FULL_SCALE_V / SPEED_MAX_PWM;                           // [online]
#define FRIC_MAX_V          1.5f     // limit for stored values (PARAMS range)
```

- [ ] **Step 2: Add the two parameter rows (`cart_single.ino`)**

After the line `  { NULL,       "V_MAX",           &V_MAX,           0.2f,   1.0f,  "m/s",     "top cart speed" },` add:

```cpp
  // Friction compensation used by motorVolts() (measured and saved by "Friction")
  { "Friction", "FRIC_POS_V",      &FRIC_POS_V,      0.0f,   FRIC_MAX_V, "V",  "breakaway command driving right (+x)" },
  { "Friction", "FRIC_NEG_V",      &FRIC_NEG_V,      0.0f,   FRIC_MAX_V, "V",  "breakaway command driving left (-x)" },
```

- [ ] **Step 3: Create `friction.ino`**

```cpp
// =============================================================================
// Motor command in volts (Feedback convention, +-U_FULL_SCALE_V = full scale)
// and the friction compensation stored in flash.
//   motorRawVolts(u): no compensation, PWM = |u| / 2.5 V * SPEED_MAX_PWM. Used
//                     by the Friction experiment to find the breakaway point.
//   motorVolts(u):    compensated. Output jumps to the breakaway command of the
//                     direction, then rises linearly to full scale at 2.5 V:
//                     v = off + |u| * (2.5 - off) / 2.5.
//                     Example, off = 0.6 V: u = 0.1 V -> 0.676 V -> PWM 49;
//                     u = 2.5 V -> 2.5 V -> PWM 180.
// =============================================================================
Preferences frictionPrefs;

static int voltsToPwm(float v) {
  return constrain((int)(v / U_FULL_SCALE_V * SPEED_MAX_PWM + 0.5f), 0, SPEED_MAX_PWM);
}

void motorRawVolts(float u) {
  u = constrain(u, -U_FULL_SCALE_V, U_FULL_SCALE_V);
  uOut = u / U_FULL_SCALE_V;
  int pwm = voltsToPwm(fabsf(u));
  if (pwm == 0)   stopMotor();
  else if (u > 0) driveRight(pwm);
  else            driveLeft(pwm);
}

void motorVolts(float u) {
  u = constrain(u, -U_FULL_SCALE_V, U_FULL_SCALE_V);
  uOut = u / U_FULL_SCALE_V;
  if (fabsf(u) < U_EPS * U_FULL_SCALE_V) { stopMotor(); return; }
  float off = u > 0 ? FRIC_POS_V : FRIC_NEG_V;
  float v   = off + fabsf(u) * (U_FULL_SCALE_V - off) / U_FULL_SCALE_V;
  int   pwm = voltsToPwm(v);
  if (u > 0) driveRight(pwm);
  else       driveLeft(pwm);
}

static bool fricValid(float v) { return v >= 0.0f && v <= FRIC_MAX_V; }   // false for NaN

// Boot: take the stored values if present and in range, else keep the defaults
void loadFriction() {
  frictionPrefs.begin("friction", true);                 // read-only
  float p = frictionPrefs.getFloat("pos", NAN);
  float n = frictionPrefs.getFloat("neg", NAN);
  frictionPrefs.end();
  if (fricValid(p) && fricValid(n)) {
    FRIC_POS_V = p;
    FRIC_NEG_V = n;
    Serial.printf("Friction compensation from flash: +%.3f V / -%.3f V\n", p, n);
  } else {
    Serial.printf("Friction compensation: defaults +%.3f V / -%.3f V (not measured yet)\n",
                  FRIC_POS_V, FRIC_NEG_V);
  }
}

void saveFriction() {
  frictionPrefs.begin("friction", false);
  frictionPrefs.putFloat("pos", FRIC_POS_V);
  frictionPrefs.putFloat("neg", FRIC_NEG_V);
  frictionPrefs.end();
}

void clearFriction() {
  frictionPrefs.begin("friction", false);
  frictionPrefs.clear();
  frictionPrefs.end();
}
```

- [ ] **Step 4: Remove the old `motorVolts` from `control.ino`**

Delete these lines from `control.ino` (the new definition lives in `friction.ino`):

```cpp
// Motor command in volts (Feedback convention, +-U_FULL_SCALE_V = full scale),
// through the same output mapping and PWM caps as the speed loop
void motorVolts(float u) {
  driveUCap(u / U_FULL_SCALE_V, SPEED_MAX_PWM);
}

```

- [ ] **Step 5: Load at boot (`cart_single.ino`)**

Replace:

```cpp
  captureParamDefaults();   // the starting values become the "defaults"
```

with:

```cpp
  captureParamDefaults();   // the starting values become the "defaults"
  loadFriction();           // measured friction compensation, if saved
```

- [ ] **Step 6: Persist on `set`, clear on `defaults` (`protocol.ino`)**

Replace:

```cpp
    for (int i = 0; i < N_PARAMS; i++) *PARAMS[i].value = PARAMS[i].def;
    Serial.println("ok defaults");
```

with:

```cpp
    for (int i = 0; i < N_PARAMS; i++) *PARAMS[i].value = PARAMS[i].def;
    clearFriction();          // otherwise a reboot would bring the old measurement back
    Serial.println("ok defaults");
```

Replace:

```cpp
    *PARAMS[i].value = f;
    Serial.printf("ok %s %g\n", PARAMS[i].name, f);
```

with:

```cpp
    *PARAMS[i].value = f;
    if (strncmp(PARAMS[i].name, "FRIC_", 5) == 0) saveFriction();   // survives a reboot
    Serial.printf("ok %s %g\n", PARAMS[i].name, f);
```

- [ ] **Step 7: Compile**

Run the firmware compile command. Expected: `Sketch uses ... bytes`, no errors.
Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/cart/cart_single && grep -n "void motorVolts" *.ino`
Expected: exactly one hit, in `friction.ino`.

- [ ] **Step 8: Commit**

```bash
cd /c/Users/super/Documents/ED2/ED2-Repo
git add cart/cart_single
git commit -m "cart_single: friction compensation in flash, motorRawVolts/motorVolts in friction.ino"
```

---

### Task 2: Experiment 1, Pendulum free swing (`exp_freeswing.ino`)

**Files:**
- Create: `cart/cart_single/exp_freeswing.ino`
- Modify: `cart/cart_single/cart_single.ino` (one `[online]` variable, one `PARAMS` row)
- Modify: `cart/cart_single/zz_registry.ino`

**Interfaces:**
- Consumes: `struct Experiment`, `announce()`, `stopMotor()`, globals `theta`, `thetaDot`, `xPos`, `energy`, `uOut`, `vRefOut`, `lastPrint`, `PRINT_MS`.
- Produces: `Experiment EXP_FREESWING` (`"FreeSwing"`, state `"FREE"`); param `FS_DURATION_S`.

- [ ] **Step 1: Add the setting (`cart_single.ino`)**

After the `#define FRIC_MAX_V` line from Task 1 add:

```cpp

// --- Free swing (experiment 1): motor off, recording time ---
float FS_DURATION_S   = 30.0f;    // s   [online]
```

After the two `"Friction"` rows in `PARAMS` add:

```cpp
  // FreeSwing
  { "FreeSwing", "FS_DURATION_S",  &FS_DURATION_S,   5.0f, 180.0f,  "s",       "how long to record the free swing" },
```

- [ ] **Step 2: Create `exp_freeswing.ino`**

```cpp
// =============================================================================
// Experiment 1: Pendulum free swing (Feedback PendulumTest). Motor off. The
// student lifts the pendulum, lets go, and the run records it swinging down
// for FS_DURATION_S seconds (damping and natural frequency from the .mat).
// =============================================================================
unsigned long fsStartMs = 0;

void fsStart() {
  stopMotor();
  uOut = vRefOut = 0;
  fsStartMs = millis();
  announce("FREE");
  Serial.println("Free swing: lift the pendulum and let it go. Recording...");
}

bool fsStep(float dt) {
  stopMotor();
  uOut = vRefOut = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.printf("FREE th %7.1f deg  w %6.2f  x %6.1f mm  E %5.2f\n",
                  theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, energy);
  }
  return millis() - fsStartMs < (unsigned long)(FS_DURATION_S * 1000.0f);
}

const char* fsState() { return "FREE"; }

// Motor off the whole time: no software soft limit (the cart can be pushed by hand)
Experiment EXP_FREESWING = { "FreeSwing", "Pendulum free swing (Ex. 1)", false, false, fsStart, fsStep, fsState };
```

- [ ] **Step 3: Register it (`zz_registry.ino`)**

Replace:

```cpp
  EXPS[N_EXPS++] = &EXP_SWINGBAL;
  EXPS[N_EXPS++] = &EXP_SIGNCHECK;
```

with:

```cpp
  EXPS[N_EXPS++] = &EXP_SWINGBAL;
  EXPS[N_EXPS++] = &EXP_FREESWING;
  EXPS[N_EXPS++] = &EXP_SIGNCHECK;
```

- [ ] **Step 4: Compile**

Run the firmware compile command. Expected: `Sketch uses ... bytes`, no errors.

- [ ] **Step 5: Commit**

```bash
git add cart/cart_single
git commit -m "cart_single: FreeSwing experiment (motor off, timed recording)"
```

---

### Task 3: Experiment 2, Static friction (`exp_friction.ino`)

**Files:**
- Create: `cart/cart_single/exp_friction.ino`
- Modify: `cart/cart_single/cart_single.ino` (setting, two constants, `PARAMS` row)
- Modify: `cart/cart_single/zz_registry.ino`

**Interfaces:**
- Consumes (Task 1): `motorRawVolts(float)`, `saveFriction()`, `FRIC_POS_V`, `FRIC_NEG_V`, `FRIC_MAX_V`. From stages 0–1: `struct Experiment`, `announce()`, `stopMotor()`, `cart_position`, `xPos`, `uOut`, `lastPrint`, `PRINT_MS`, `U_FULL_SCALE_V`.
- Produces: `Experiment EXP_FRICTION` (`"Friction"`, state `"FRICTION"`); param `FR_RAMP_VPS`; result lines `result friction_pos_V <V>` and `result friction_neg_V <V>`.

- [ ] **Step 1: Add the setting and constants (`cart_single.ino`)**

After the `float FS_DURATION_S` line from Task 2 add:

```cpp

// --- Static friction (experiment 2): ramp until the cart moves ---
float FR_RAMP_VPS     = 0.5f;     // V/s ramp of the raw motor command   [online]
#define FR_MOVE_COUNTS      13       // ~1 mm of cart travel = "it moved"
#define FR_SETTLE_MS        1000     // motor off between the two directions
```

After the `"FreeSwing"` row in `PARAMS` add:

```cpp
  // Friction (the measured values are FRIC_POS_V / FRIC_NEG_V above)
  { "Friction", "FR_RAMP_VPS",     &FR_RAMP_VPS,     0.1f,   2.0f,  "V/s",     "how fast the motor command ramps up" },
```

- [ ] **Step 2: Create `exp_friction.ino`**

```cpp
// =============================================================================
// Experiment 2: Static friction (Feedback PendulumFriction, Ex. 3). Ramps the
// raw motor command up from 0 at FR_RAMP_VPS until the cart has moved
// FR_MOVE_COUNTS (~1 mm), first driving right, then (after the cart settles)
// driving left. The two breakaway commands are reported as results and become
// the friction compensation of motorVolts() (saved in flash).
// If the cart doesn't move by 2.5 V (motor power off, cart blocked) the run
// ends and the stored values are left unchanged.
// =============================================================================
int           frPhase   = 0;     // 0 ramp right, 1 settle, 2 ramp left, 3 store
float         frU       = 0;     // current raw command magnitude (V)
float         frPosV    = 0;     // measured breakaway driving right (V)
int32_t       frStartCount = 0;
unsigned long frPhaseMs = 0;

void frStart() {
  frPhase      = 0;
  frU          = 0;
  frStartCount = cart_position;
  stopMotor();
  announce("FRICTION");
  Serial.printf("Friction: ramping right at %.2f V/s until the cart moves...\n", FR_RAMP_VPS);
}

// Clamp to the stored range and save. Returns the value actually stored.
float frStore(float v, const char* dir) {
  if (v > FRIC_MAX_V) {
    Serial.printf("WARNING: breakaway driving %s was %.3f V, above %.1f V: stored as %.1f V. "
                  "Check the belt tension.\n", dir, v, FRIC_MAX_V, FRIC_MAX_V);
    return FRIC_MAX_V;
  }
  return v;
}

bool frStep(float dt) {
  if (frPhase == 0 || frPhase == 2) {
    int dir = frPhase == 0 ? 1 : -1;
    frU += FR_RAMP_VPS * dt;
    if (frU > U_FULL_SCALE_V) {
      stopMotor();
      uOut = 0;
      Serial.printf("Friction: the cart did not move driving %s up to %.1f V "
                    "(motor power off or cart blocked?). Stored values unchanged.\n",
                    dir > 0 ? "right" : "left", U_FULL_SCALE_V);
      return false;
    }
    motorRawVolts(dir * frU);
    if (abs(cart_position - frStartCount) >= FR_MOVE_COUNTS) {
      stopMotor();
      Serial.printf("result friction_%s_V %.3f\n", dir > 0 ? "pos" : "neg", frU);
      if (dir > 0) frPosV = frU;
      frPhase  += 1;
      frPhaseMs = millis();
      if (frPhase == 3) {
        FRIC_POS_V = frStore(frPosV, "right");
        FRIC_NEG_V = frStore(frU, "left");
        saveFriction();
        Serial.printf("Friction compensation saved: +%.3f V / -%.3f V\n", FRIC_POS_V, FRIC_NEG_V);
        return false;
      }
    }
  } else if (frPhase == 1) {
    stopMotor();
    uOut = 0;
    if (millis() - frPhaseMs >= FR_SETTLE_MS) {
      frPhase      = 2;
      frU          = 0;
      frStartCount = cart_position;
      Serial.println("Friction: ramping left...");
    }
  }
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.printf("FRICTION phase %d  u %5.3f V  moved %ld counts  x %6.1f mm\n",
                  frPhase, frU, (long)abs(cart_position - frStartCount), xPos * 1000.0f);
  }
  return true;
}

const char* frState() { return "FRICTION"; }

Experiment EXP_FRICTION = { "Friction", "Static friction (Ex. 3)", false, true, frStart, frStep, frState };
```

- [ ] **Step 3: Register it (`zz_registry.ino`)**

Replace:

```cpp
  EXPS[N_EXPS++] = &EXP_FREESWING;
```

with:

```cpp
  EXPS[N_EXPS++] = &EXP_FREESWING;
  EXPS[N_EXPS++] = &EXP_FRICTION;
```

- [ ] **Step 4: Compile and check the registry order**

Run the firmware compile command. Expected: `Sketch uses ... bytes`, no errors.
Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/cart/cart_single && grep -n "EXPS\[N_EXPS++\]" zz_registry.ino`
Expected, in this order: `EXP_SWINGBAL`, `EXP_FREESWING`, `EXP_FRICTION`, `EXP_SIGNCHECK`, `EXP_STEPTEST`.

- [ ] **Step 5: Commit**

```bash
git add cart/cart_single
git commit -m "cart_single: Friction experiment (breakaway both directions, saved as motorVolts compensation)"
```

---

### Task 4: Console and mock for the new experiments

**Files:**
- Modify: `pendulum-remote/frontend/app.js` (states, unknown-state fallback, experiment info, labels, help, parameter group)
- Modify: `pendulum-remote/bridge/bridge.py` (mock experiments, parameters, simulated runs)
- Create: `pendulum-remote/bridge/tests/test_mock.py`

**Interfaces:**
- Consumes: firmware names from Tasks 1–3 (`FreeSwing`/`FREE`, `Friction`/`FRICTION`; params `FS_DURATION_S`, `FR_RAMP_VPS`, `FRIC_POS_V`, `FRIC_NEG_V`; results `friction_pos_V`, `friction_neg_V`).
- Produces: UI only; the mock mirrors the firmware lines.

- [ ] **Step 1: Write the failing mock tests (`bridge/tests/test_mock.py`)**

```python
from bridge import MockBridge


class QuietMock(MockBridge):
    """MockBridge without the browser side: pushes are collected, not broadcast."""

    def __init__(self, runs_dir):
        super().__init__(runs_dir)
        self.pushed = []

    def push(self, msg):
        self.pushed.append(msg)


def test_mock_lists_the_stage2_experiments_in_firmware_order(tmp_path):
    m = QuietMock(tmp_path)
    m._cmd("exp list")
    assert [e["name"] for e in m.experiments] == ["SwingBal", "FreeSwing", "Friction", "SignCheck", "StepTest"]


def test_mock_friction_shows_its_settings_and_the_shared_ones(tmp_path):
    m = QuietMock(tmp_path)
    m._cmd("exp select Friction")
    m._cmd("params")
    assert {p["name"] for p in m.params} == {"FR_RAMP_VPS", "FRIC_POS_V", "FRIC_NEG_V", "KV_P", "V_FF", "V_MAX"}


def test_mock_freeswing_shows_its_duration(tmp_path):
    m = QuietMock(tmp_path)
    m._cmd("exp select FreeSwing")
    m._cmd("params")
    assert "FS_DURATION_S" in {p["name"] for p in m.params}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd /c/Users/super/Documents/ED2/pendulum-remote && python -m pytest bridge/tests/test_mock.py -q`
Expected: 3 failures (the experiment list lacks `FreeSwing`/`Friction`; `exp select Friction` is an unknown experiment).

- [ ] **Step 3: Update the mock (`bridge/bridge.py`)**

Replace:

```python
MOCK_EXPERIMENTS = [("SwingBal", "Swing-up & balance"),
                    ("SignCheck", "Sign check (motor off)"),
                    ("StepTest", "Speed loop step test")]
```

with:

```python
MOCK_EXPERIMENTS = [("SwingBal", "Swing-up & balance"),
                    ("FreeSwing", "Pendulum free swing (Ex. 1)"),
                    ("Friction", "Static friction (Ex. 3)"),
                    ("SignCheck", "Sign check (motor off)"),
                    ("StepTest", "Speed loop step test")]
```

Replace:

```python
    (None, "V_MAX", 0.8, 0.2, 1.0, "m/s", "top cart speed"),
]
MOCK_RUN_STATE = {"SwingBal": "SWING", "SignCheck": "CHECK", "StepTest": "STEP"}
```

with:

```python
    (None, "V_MAX", 0.8, 0.2, 1.0, "m/s", "top cart speed"),
    ("Friction", "FRIC_POS_V", 0.5556, 0, 1.5, "V", "breakaway command driving right (+x)"),
    ("Friction", "FRIC_NEG_V", 0.4167, 0, 1.5, "V", "breakaway command driving left (-x)"),
    ("FreeSwing", "FS_DURATION_S", 30, 5, 180, "s", "how long to record the free swing"),
    ("Friction", "FR_RAMP_VPS", 0.5, 0.1, 2.0, "V/s", "how fast the motor command ramps up"),
]
MOCK_RUN_STATE = {"SwingBal": "SWING", "FreeSwing": "FREE", "Friction": "FRICTION",
                  "SignCheck": "CHECK", "StepTest": "STEP"}
```

In `MockBridge._sample`, replace:

```python
        return 0.0, math.pi + 0.1 * math.sin(5.6 * t), 0.0        # CHECK: motor off
```

with:

```python
        if s == "FREE":                                             # released from 60 deg, decaying
            return 0.0, math.pi + math.radians(60) * math.exp(-0.02 * t) * math.cos(5.6 * t), 0.0
        if s == "FRICTION":                                         # ramp right 0-1.6 s, settle, ramp left
            if t < 1.6:
                return (0.001 if t > 1.4 else 0.0), math.pi, 0.5 * t
            if t < 2.6:
                return 0.001, math.pi, 0.0
            return (0.0 if t > 3.8 else 0.001), math.pi, -0.5 * (t - 2.6)
        return 0.0, math.pi + 0.1 * math.sin(5.6 * t), 0.0        # CHECK: motor off
```

In `MockBridge._run`, replace:

```python
            elif s == "STEP" and t > 3.6:
```

with:

```python
            elif s == "FREE" and t > self.vals["FS_DURATION_S"]:
                self._emit("run end done")
                self._goto("IDLE", now)
            elif s == "FRICTION" and t > 4.0:
                self._emit("result friction_pos_V 0.700")
                self._emit("result friction_neg_V 0.600")
                self.vals["FRIC_POS_V"], self.vals["FRIC_NEG_V"] = 0.7, 0.6
                self._emit("Friction compensation saved: +0.700 V / -0.600 V")
                self._emit("run end done")
                self._goto("IDLE", now)
            elif s == "STEP" and t > 3.6:
```

Replace:

```python
            if self.sim_state in ("SWING", "BALANCE", "STEP", "CHECK"):
```

with:

```python
            if self.sim_state in ("SWING", "BALANCE", "STEP", "CHECK", "FREE", "FRICTION"):
```

- [ ] **Step 4: Run the bridge tests**

Run: `python -m pytest bridge/tests -q`
Expected: all pass (27 earlier + 3 new = 30).

- [ ] **Step 5: Console names, help and states (`frontend/app.js`)**

In `STATES`, replace:

```js
  CHECK:   ["Sign check", "Motor off. Move the cart and pendulum by hand and check the readings."],
```

with:

```js
  CHECK:   ["Sign check", "Motor off. Move the cart and pendulum by hand and check the readings."],
  FREE:    ["Free swing", "Motor off. Lift the pendulum and let it go: the swing is being recorded."],
  FRICTION: ["Measuring friction", "Slowly raising the motor command until the cart moves, right then left."],
  RUN:     ["Running", "The selected experiment is running."],
```

Replace:

```js
  if (!STATES[s]) s = s === "STEP TEST" ? "STEP" : "IDLE";
```

with:

```js
  // A state this console doesn't know is still a running rig: never show "Ready"
  if (!STATES[s]) s = s === "STEP TEST" ? "STEP" : "RUN";
```

In `EXP_INFO`, after the `SwingBal: [...]` entry (before `SignCheck:`), add:

```js
  FreeSwing: ["Motor off. Records the pendulum swinging freely after you lift it and let go (Feedback exercise 1).",
              "The cart will home to both ends, then the motor turns off. When it says Free swing, lift the pendulum to one side and let it go."],
  Friction: ["Ramps the motor slowly until the cart moves, both ways. The results become the friction compensation (Feedback exercise 3).",
             "The cart will home to both ends, then creep a few millimetres right and then left while the motor command ramps up."],
```

Replace:

```js
  { title: "Cart speed loop", names: ["KV_P", "V_FF", "V_MAX"] },
];
```

with:

```js
  { title: "Cart speed loop", names: ["KV_P", "V_FF", "V_MAX"] },
  { title: "Friction compensation", names: ["FRIC_POS_V", "FRIC_NEG_V"] },
];
```

In `LABELS`, replace:

```js
  V_MAX: "Top speed",
};
```

with:

```js
  V_MAX: "Top speed",
  FS_DURATION_S: "Recording time",
  FR_RAMP_VPS: "Ramp rate",
  FRIC_POS_V: "Breakaway, driving right",
  FRIC_NEG_V: "Breakaway, driving left",
};
```

In `HELP`, replace:

```js
  V_MAX: "Speed limit for the cart.",
};
```

with:

```js
  V_MAX: "Speed limit for the cart.",
  FS_DURATION_S: "How long to record after the motor turns off.",
  FR_RAMP_VPS: "Slower is more accurate. 0.5 V/s takes about 2 s per direction.",
  FRIC_POS_V: "Measured by this experiment and saved on the rig. Used to overcome friction.",
  FRIC_NEG_V: "Measured by this experiment and saved on the rig. Used to overcome friction.",
};
```

- [ ] **Step 6: Browser check against the mock**

Run: `python bridge/bridge.py --mock` and open http://localhost:8000. Take control. Expected:
1. The picker lists "Pendulum free swing (Ex. 1)" and "Static friction (Ex. 3)" under **Identification**.
2. Selecting **Static friction** shows "Ramp rate" under *Experiment settings*, and the two breakaway fields under **Friction compensation**, plus the speed-loop group.
3. Starting it shows "Measuring friction". After about 6 s the run row reads `... done · friction_pos_V 0.7, friction_neg_V 0.6`. Clicking the experiment again shows the breakaway fields at 0.7 / 0.6.
4. **Review focus 4:** in DevTools run `onMessage({kind: "state", state: "WIBBLE"})`. The heading reads "Running", the picker and parameter fields are disabled, and Start is disabled.
5. Selecting **Pendulum free swing** with the recording time set to 5 s: the run ends by itself (`done`) and the angle chart shows a decaying swing.

- [ ] **Step 7: Commit**

```bash
cd /c/Users/super/Documents/ED2/pendulum-remote
git add frontend/app.js bridge/bridge.py bridge/tests/test_mock.py
git commit -m "console + mock: FreeSwing and Friction experiments; unknown states show as Running"
```

---

### Task 5: Rig check and docs

**Files:**
- Modify: `pendulum-remote/README.md` (experiment list line)

**Interfaces:**
- Consumes: everything above.
- Produces: stage 2 "done when" from the spec: *friction values reported and used by `motorVolts`*.

- [ ] **Step 1: README (`pendulum-remote/README.md`)**

Replace:

```markdown
- **Experiment** picker: choose what the next run does (swing-up & balance,
  rig checks; the Feedback lab experiments arrive in later stages).
```

with:

```markdown
- **Experiment** picker: choose what the next run does: swing-up & balance,
  pendulum free swing (Feedback ex. 1), static friction (ex. 3; its results are
  saved on the rig as the friction compensation), and rig checks. The other
  Feedback experiments arrive in later stages.
```

Commit:

```bash
cd /c/Users/super/Documents/ED2/pendulum-remote
git add README.md
git commit -m "README: free swing and static friction experiments"
```

- [ ] **Step 2: Rig session (user runs, Claude reads the log and runs)**

Ask the user to flash `cart/cart_single`, then run `python cart/serial_log.py` and type each line:
1. `exp list`
2. `exp select Friction`, then `run`. Wait for `run end done`.
3. `params`
4. **Review focus 1:** turn the 24 V kill switch **off**, then `run`. Wait for "did not move" and `run end done`. Then turn the 24 V back on.
5. `params` again.
6. Press the ESP32's EN (reset) button, wait for the banner, then `exp select Friction` and `params`.
7. `exp select FreeSwing`, `set FS_DURATION_S 20`, `run`. After `>> FREE`, lift the pendulum to about 60°, let go, and wait for `run end done`.
8. **Review focus 5:** `defaults`, reset the board again, `exp select Friction`, `params`.

Then read the newest `cart/logs/*.log`. Expected:
- `exp FreeSwing | Pendulum free swing (Ex. 1)` and `exp Friction | Static friction (Ex. 3)` are listed.
- Step 2: `>> FRICTION`, then `result friction_pos_V` and `result friction_neg_V`, each roughly 0.5–1.0 V (homing stalls at PWM 50–60, i.e. 0.7–0.83 V on this scale), `Friction compensation saved: ...`, `run end done`. **Review focus 2:** if either value exceeds 1.5 V, a `WARNING ... stored as 1.5 V` line appears and the stored value is 1.5.
- Step 3: `param FRIC_POS_V` / `FRIC_NEG_V` show the measured values.
- Step 4: `the cart did not move driving right up to 2.5 V ... Stored values unchanged.`, `run end done`, and step 5's `params` still shows the step 2 values.
- Step 6 (**review focus 3**): the boot banner is followed by `Friction compensation from flash: +X V / -Y V` with the step 2 values.
- Step 7: `>> FREE`, about 20 s of `D` lines with `u` = 0.000, and `run end done` without STOP.
- Step 8: after reset, `Friction compensation: defaults +0.556 V / -0.417 V (not measured yet)`.

- [ ] **Step 3: Check the free-swing data (Claude)**

Run (after the user has also done one Free swing run from the console, so a `.mat` exists in `pendulum-remote/runs/`):
```bash
cd /c/Users/super/Documents/ED2/pendulum-remote && python -c "
import glob, numpy as np, scipy.io
f = sorted(glob.glob('runs/*FreeSwing*.mat'))[-1]; m = scipy.io.loadmat(f)
t, th = m['t'][:, 0], m['theta'][:, 0] - np.pi
z = np.where(np.diff(np.sign(th)) != 0)[0]
print(f, len(t), 'samples; zero crossings', len(z), '; period %.3f s' % (2 * np.mean(np.diff(t[z][len(z)//4:]))))"
```
Expected: a period of 1.10–1.14 s (the measured 1.12 s, ω0 = 5.61 rad/s), and `theta` oscillating around π.

---

## Self-review notes

**Spec coverage (stage 2):**

| Spec item | Task |
|---|---|
| Experiment 1: motor off, records the release, ends after a set time or on STOP, parameter `duration` | 2 |
| Experiment 2: ramps `u` until movement, both directions, reports both breakaway voltages, parameter ramp rate | 3 |
| Experiment 2's results become the shared friction compensation, stored in flash; `defaults` resets them | 1 and 3 |
| `motorVolts`: clamp ±2.5 V, deadband, add the direction's offset, map to PWM within the existing caps | 1 |
| Console picker entries and the "done when" (friction values reported and used) | 4 and 5 |

**Deliberately not here:**
- Experiments 3–11 (later stages).
- The `docs/esp32.md` single-board section (stage 8).
- The deferred minors from the stage 0–1 review.
