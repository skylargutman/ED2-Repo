# Experiments Foundation (Stages 0–1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the single-board firmware into a runtime experiment framework that streams 500 Hz run data, and make the `pendulum-remote` console pick experiments, record every run, and offer CSV/.mat downloads. The current swing-up and balance runs exactly as before inside the new framework.

**Architecture:**
- **Firmware.** The firmware (`cart/cart_single`, Arduino, ESP32) is split into sketch tabs. A small `Experiment` interface runs the selected experiment after homing and the hanging reference. Every 2 ms step prints one `D` data line between `run start` and `run end`, over USB serial at 921600 baud.
- **Bridge.** The bridge (`pendulum-remote/bridge`, Python, FastAPI) records those lines into runs. It saves each run as CSV + MATLAB Level 5 `.mat` + a `.json` summary, and serves them at `/runs`.
- **Console.** The console (`pendulum-remote/frontend`, plain JS) gets an experiment picker, a motor-command chart and a runs panel.

**Tech Stack:**
- Firmware: Arduino-ESP32 core 3.3.7 (`esp32:esp32:nodemcu-32s`), C++ sketch tabs.
- Bridge: Python 3.13, FastAPI, uvicorn, pyserial; pytest + SciPy + httpx for tests only.
- Console: vanilla JS/HTML/CSS.

**Spec:** `docs/superpowers/specs/2026-10-07-esp32-experiments-design.md` (ED2-Repo). This plan covers build stages 0 and 1. Stages 2–8 get their own plans after this one is verified on the rig.

**Repositories:**
- **A** = `C:\Users\super\Documents\ED2\ED2-Repo` (firmware, logger, docs; branch `cart-standalone-swingup`)
- **B** = `C:\Users\super\Documents\ED2\pendulum-remote` (bridge, console; local git only)

## Global Constraints

- **NEVER `git push`** either repo (IP hold). Commit locally **only if the user has approved local commits** for this work. Otherwise skip every "Commit" step and leave changes uncommitted.
- USB serial runs at **921600 baud** everywhere: firmware, `bridge.py`, `cart/serial_log.py`, Arduino Serial Monitor.
- Data line format, exactly: `D <t_ms> <x_m> <theta_rad> <u_V> <ref>` (+ optional trailing `r`).
  - `t_ms` = ms since `run start`.
  - `theta` is **continuous**: 0 = upright, +π = hanging at run start (upright reached the other way reads 2π).
  - `u` in volts, ±2.5 = full scale.
  - `ref` = `nan` when there is no setpoint.
- Run bracket lines: `run start NAME K=V ...` and `run end REASON`, with REASON ∈ `done | stop | soft-limit | overspeed | limit-switch`. The bridge adds `interrupted | disconnected`.
- Experiment names (firmware ↔ console), now: `SwingBal`, `SignCheck`, `StepTest`. Reserved for later stages: `FreeSwing`, `Friction`, `CartIdent`, `CraneIdent`, `InvPendIdent`, `CartPID`, `SwingUp`, `PendStab`, `CraneCtl`, `SwingHold`, `UpDown`.
- Bridge runtime dependencies stay `fastapi`, `uvicorn`, `pyserial`. SciPy and httpx are **test-only** (`bridge/requirements-dev.txt`).
- Console copy: no em/en dashes in UI text (the existing `clean()` rule).
- Firmware compile command (PowerShell), used by every firmware task:
  ```powershell
  $cli = (Get-ChildItem "$env:LOCALAPPDATA\Programs\arduino-ide" -Filter arduino-cli.exe -Recurse | Select-Object -First 1).FullName; & $cli compile --fqbn esp32:esp32:nodemcu-32s --warnings default "C:\Users\super\Documents\ED2\ED2-Repo\cart\cart_single"
  ```
  Expected: ends with `Sketch uses ... bytes` and no `error:` lines.

**Implementation choices made while planning (flag to the user at review):**
- The spec's `exp_*.cpp` files become Arduino **`.ino` tabs**. Tabs share one translation unit, so the existing globals need no `extern`/header plumbing.
- Experiment parameters stay in the **one `PARAMS` table**, each tagged with its experiment (`NULL` = shared), instead of a per-experiment array.
- The `SIGN_CHECK` / `STEP_TEST` compile switches become the **`SignCheck` / `StepTest` experiments**: no reflash needed to run rig checks.
- Each saved run also gets a **`.json` summary**, which is what `GET /runs` lists. CSV and `.mat` stay the student downloads.

## Review Focus

These inputs are not exercised by the main tests unless pinned. Each one has a test in the task that owns the code.

1. **Serial cable unplugged or ESP32 reset mid-run.** The partial run is still saved with reason `disconnected` (or `interrupted` if a new `run start` arrives first), never lost. → Task 3 (`test_abort_keeps_partial_run`, `test_restart_without_end_closes_previous_as_interrupted`), Task 5 (`test_serial_lost_saves_partial_run`).
2. **Garbled `D` lines from USB noise** (seen on 2026-10-07 as 64-byte `����` chunks). These are dropped and counted, and recording continues. → Task 3 (`test_garbled_d_lines_are_counted_not_recorded`).
3. **A run that stops after a few ms (zero samples), or settings whose names aren't valid MATLAB identifiers.** Files are still written and still load in MATLAB/SciPy. → Task 4 (`test_empty_run_and_odd_names`).
4. **Download URLs with `..` or other file types, or without the token when `BRIDGE_TOKEN` is set.** These get 404/401 and no file outside `runs/` is served. → Task 5 (`test_runs_endpoints`).
5. **Two runs saved in the same second** (fast auto re-home). Both files survive with distinct names. → Task 3 (`test_save_run_never_overwrites`).

## Before you start

- Repo A has uncommitted work from 2026-10-07: `cart/cart_single/`, `cart/serial_log.py`, `cart/cart_control/cart_control.ino`, `docs/esp32.md`, and this plan and spec. Repo B may have uncommitted changes too.
- Ask the user whether to make a **local baseline commit** of that existing work before Task 1. If yes (Repo A):
  ```bash
  cd /c/Users/super/Documents/ED2/ED2-Repo
  git add cart/cart_single cart/serial_log.py cart/cart_control/cart_control.ino docs/esp32.md docs/superpowers
  git commit -m "cart: single-board firmware v11.0, logger commands, experiments spec and plan"
  ```
  Never push.

---

### Task 1: Split `cart_single.ino` into tabs (no behaviour change)

**Files:**
- Modify: `cart/cart_single/cart_single.ino` (keeps config, globals, `setup()`, `loop()`)
- Create: `cart/cart_single/hardware.ino`, `cart/cart_single/protocol.ino`, `cart/cart_single/control.ino`, `cart/cart_single/exp_swingbal.ino`

**Interfaces:**
- Consumes: the current `cart_single.ino` (v11.0-single, 1366 lines).
- Produces:
  - `hardware.ino`: motor helpers, limit/trigger helpers, `setupPcnt`, `readEncoders`, stall detection, homing, `runHoming()`, `runHomingSteps()`.
  - `protocol.ino`: serial commands.
  - `control.ino`: `wrapPi`, `halfTrackM`, `driveUCap`, `speedDrive`, `standaloneStart`, `enterState`, `updateState`, `stepHang`, `standaloneStep`.
  - `exp_swingbal.ino`: `swingV`, `balanceGains`, `balanceStart`, `balanceV`.
  - All shared globals (including the former "Standalone swing-up + balance" globals block) live in `cart_single.ino`.

Arduino concatenates the main `.ino` first, then the other tabs alphabetically, and inserts function prototypes before the first function (`setup()`). Global **variables and types must be defined before the tabs that use them**, which is why every shared global stays in the main tab.

- [ ] **Step 1: Run the split script**

Save as `C:\Users\super\AppData\Local\Temp\claude\split_tabs.py` (or any scratch path) and run it from `cart/cart_single`:

```python
"""Split cart_single.ino into Arduino tabs by its section banners. Moves lines only."""
from pathlib import Path

src = Path("cart_single.ino")
L = src.read_text(encoding="utf-8").split("\n")


def starts(prefix):
    hits = [i for i, s in enumerate(L) if s.startswith(prefix)]
    assert len(hits) == 1, (prefix, hits)
    return hits[0]


def banner_before(prefix):
    i = starts(prefix)
    assert L[i - 1].startswith("// ====="), prefix
    return i - 1


i_motor = banner_before("// Motor helpers")
i_usb = banner_before("// USB serial command interface")
i_enc = banner_before("// Encoders ")
i_glob = banner_before("// Standalone swing-up + balance")
i_wrap = starts("float wrapPi(float a) {")
i_sw = starts("// Cart speed command for swing-up")
i_step = starts("void standaloneStep() {")
i_setup = banner_before("// Setup")
assert i_motor < i_usb < i_enc < i_glob < i_wrap < i_sw < i_step < i_setup

parts = {
    "cart_single.ino": L[:i_motor] + L[i_glob:i_wrap] + L[i_setup:],
    "hardware.ino": L[i_motor:i_usb] + L[i_enc:i_glob],
    "protocol.ino": L[i_usb:i_enc],
    "control.ino": L[i_wrap:i_sw] + L[i_step:i_setup],
    "exp_swingbal.ino": L[i_sw:i_step],
}
assert sorted(sum(parts.values(), [])) == sorted(L), "a line was lost or duplicated"
for name, lines in parts.items():
    Path(name).write_text("\n".join(lines).rstrip("\n") + "\n", encoding="utf-8", newline="\n")
print({k: len(v) for k, v in parts.items()})
```

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/cart/cart_single && python -X utf8 /path/to/split_tabs.py`
Expected: a dict of five files with line counts, and no AssertionError.

- [ ] **Step 2: Compile**

Run the firmware compile command from Global Constraints.
Expected: `Sketch uses ... bytes`, no errors. If an error says a variable "was not declared in this scope", that variable's definition sits in a later tab than its use. Move the definition into `cart_single.ino`'s globals; don't change code.

- [ ] **Step 3: Confirm it is a pure move**

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo && git diff --stat -- cart/cart_single`
Expected: `cart_single.ino` loses about 900 lines. The four new files are untracked (`git status` lists them). Nothing else in the repo changed.

- [ ] **Step 4: Commit (only if approved)**

```bash
git add cart/cart_single
git commit -m "cart_single: split into tabs (hardware, protocol, control, exp_swingbal), no behaviour change"
```

---

### Task 2: Experiment framework, 921600 baud, D lines, rig checks as experiments

**Files:**
- Modify: `cart/cart_single/cart_single.ino`, `cart/cart_single/hardware.ino`, `cart/cart_single/exp_swingbal.ino`, `cart/serial_log.py`
- Replace whole content: `cart/cart_single/protocol.ino`, `cart/cart_single/control.ino`
- Create: `cart/cart_single/exp_checks.ino`, `cart/cart_single/zz_registry.ino`

**Interfaces:**
- Consumes (Task 1): the tab layout; existing functions `readEncoders()`, `runHoming()`, `stopMotor()`, `readyLed(bool)`, `driveUCap(float,int)`, `speedDrive(float)`, `updateState()`, `swingV()`, `balanceGains()`, `balanceStart()`, `balanceV()`; globals `theta, thetaDot, xPos, xDot, energy, energyEnd, uOut, vRefOut, stateDt, pend_position, pendDownRef, lastPrint, stepStart, autoRehomes, brakeRehome, brakeStart, balTrim, balStartMs`.
- Produces (used by later stages' experiments):
  - `struct Experiment { const char* name; const char* title; bool autoRehome; bool safety; void (*start)(); bool (*step)(float dt); const char* (*state)(); }`
  - Globals `Experiment* curExp`, `float expRef` (setpoint for D lines, `NAN` = none), `enum RunPhase { PH_IDLE, PH_HANG, PH_EXP, PH_BRAKE }` and `RunPhase phase`.
  - `void motorVolts(float u)`: volts → PWM.
  - `void announce(const char* state)` prints `>> STATE`.
  - `void registerExperiments()` in `zz_registry.ino`. Later stages add one `EXPS[N_EXPS++] = &EXP_X;` line each.
  - `Param` gains a first field `const char* exp`.
  - Serial: `exp list`, `exp select NAME`, `run`, `status STATE EXP`, `run start`, `D`, `run end`.

- [ ] **Step 1: Edit the main tab's header and config (`cart_single.ino`)**

Replace `// Version: 11.0` with `// Version: 11.1`.

After the line `//   - New serial command "io": counts and switch states, for bring-up.` insert:

```cpp
//
// Changes in 11.1 (experiments framework; design in
// docs/superpowers/specs/2026-10-07-esp32-experiments-design.md):
//   - Code split into tabs: hardware.ino, protocol.ino, control.ino,
//     exp_*.ino (one per experiment), zz_registry.ino (experiment list).
//   - Experiments chosen at runtime: "exp list", "exp select NAME", "run".
//     SwingBal (swing-up + balance), SignCheck and StepTest; the last two
//     replace the SIGN_CHECK / STEP_TEST compile switches.
//   - USB serial at 921600 baud. Each run prints "run start NAME K=V ...",
//     one "D t_ms x theta u ref" line per 2 ms step, then "run end REASON".
//   - PEND_TRIM_DEG 5.5 (measured upright offset, 2026-10-07).
```

Replace `#define FW_VERSION "11.0-single"` with `#define FW_VERSION "11.1-single"`.

Replace these four lines:

```cpp
// 1 = motor stays OFF after homing; prints angle and position so the signs
// can be checked by hand. Set to 0 only after the check passes (docs/esp32.md).
// Passed on the single-board build 2026-10-07 (PEND_SIGN -1 still correct).
#define SIGN_CHECK         0
```

with:

```cpp
// Sign check = the "SignCheck" experiment: motor off after homing, angle and
// position printed so the signs can be checked by hand (docs/esp32.md).
// Passed on the single-board build 2026-10-07 (PEND_SIGN -1 still correct).
```

Replace:

```cpp
// Upright offset (degrees). If the balanced cart creeps steadily one way,
// nudge this by 0.2-0.5 deg until it holds still.
#define PEND_TRIM_DEG       0.0f
```

with:

```cpp
// Upright offset (degrees). 2026-10-07, single-board build: the balance
// self-trim settled at 5.52 deg with the cart parked at centre, and a whole
// run lost only 7 pendulum counts (1.3 deg), so the offset is fixed, not lost
// counts. Two-board runs on 2026-10-02 settled at 5.5-6.4 deg.
#define PEND_TRIM_DEG       5.5f
```

After the line `#define U_EPS               0.02f` insert:

```cpp
// Motor command in volts, Feedback convention: +-2.5 V = full scale (u = 1)
#define U_FULL_SCALE_V      2.5f
```

Replace:

```cpp
// --- Speed loop step test ---
// 1 = after HANG, instead of swinging up: +STEP_V for STEP_MS, -STEP_V for
// STEP_MS, STEP_CYCLES times, then stop. Prints speed every STEP_PRINT_MS.
```

with:

```cpp
// --- Speed loop step test (the "StepTest" experiment) ---
// +STEP_V for STEP_MS, -STEP_V for STEP_MS, STEP_CYCLES times, then the run
// ends. Prints speed every STEP_PRINT_MS.
```

Delete the line `#define STEP_TEST           0`.

- [ ] **Step 2: Replace the controller-state types and the parameter table (`cart_single.ino`)**

Replace:

```cpp
// Standalone controller state (declared up here for the Arduino prototype
// generator, like StallGuard below)
enum BalState { BAL_HANG, BAL_CHECK, BAL_SWING, BAL_BALANCE, BAL_STEP, BAL_DONE, BAL_BRAKE };
BalState balState = BAL_HANG;
```

with:

```cpp
// --- Experiments (types declared up here for the Arduino prototype generator) ---
// One experiment = what runs after homing + the hanging reference. control.ino
// owns the run (D lines, safety, run end); the experiment only computes the
// motor command each 2 ms step. Instances live in exp_*.ino, the list in
// zz_registry.ino.
struct Experiment {
  const char* name;          // command / file name, no spaces ("SwingBal")
  const char* title;         // shown in the console
  bool        autoRehome;    // re-home and run again after a soft-limit stop
  bool        safety;        // soft limit + overspeed checks (off for motor-off checks)
  void        (*start)();    // once, when the run starts
  bool        (*step)(float dt);   // every 2 ms step; false = finished
  const char* (*state)();    // state name for "status" ("SWING", "BALANCE", ...)
};
#define MAX_EXPS 16
Experiment* EXPS[MAX_EXPS];
int         N_EXPS = 0;
Experiment* curExp = NULL;   // selected experiment (EXPS[0] at boot)
float       expRef = NAN;    // current setpoint for the D line, NAN = none

enum RunPhase { PH_IDLE, PH_HANG, PH_EXP, PH_BRAKE };
RunPhase      phase      = PH_IDLE;
bool          runActive  = false;    // "run start" printed, "run end" not yet
unsigned long runStartMs = 0;
```

Delete the line:

```cpp
const char* BAL_NAMES[] = { "HANG", "CHECK", "SWING", "BALANCE", "STEP TEST", "DONE", "BRAKE" };
```

Replace the `struct Param { ... };` definition and the whole `Param PARAMS[] = { ... };` table with:

```cpp
struct Param {
  const char* exp;      // experiment that uses it; NULL = shared by all
  const char* name;
  float*      value;
  float       lo, hi;
  const char* unit;
  const char* desc;
  float       def;      // filled in by captureParamDefaults()
};
Param PARAMS[] = {
  // SwingBal: balance
  { "SwingBal", "BAL_MODE",        &BAL_MODE,        0.0f,   1.0f,  "-",       "0 = gains from BAL_POLE, 1 = manual gains" },
  { "SwingBal", "BAL_POLE",        &BAL_POLE,        3.0f,   7.0f,  "rad/s",   "balance stiffness (all four gains computed from it)" },
  { "SwingBal", "BAL_K_TH",        &BAL_K_TH,        0.0f, 200.0f,  "m/s2/rad",  "manual gain: pendulum angle" },
  { "SwingBal", "BAL_K_THD",       &BAL_K_THD,       0.0f,  40.0f,  "m/s2/(rad/s)", "manual gain: pendulum angular speed" },
  { "SwingBal", "BAL_K_X",         &BAL_K_X,       -50.0f, 100.0f,  "m/s2/m",  "manual gain: cart position" },
  { "SwingBal", "BAL_K_V",         &BAL_K_V,       -50.0f,  80.0f,  "m/s2/(m/s)", "manual gain: cart speed" },
  { "SwingBal", "CATCH_ANGLE_DEG", &CATCH_ANGLE_DEG, 5.0f,  20.0f,  "deg",     "start balancing inside this angle from upright" },
  { "SwingBal", "CATCH_RATE",      &CATCH_RATE,      1.0f,   3.0f,  "rad/s",   "...and only if turning slower than this" },
  { "SwingBal", "DROP_ANGLE_DEG",  &DROP_ANGLE_DEG, 20.0f,  45.0f,  "deg",     "give up balancing past this angle" },
  // SwingBal: swing-up
  { "SwingBal", "SWING_V",         &SWING_V,         0.1f,   0.6f,  "m/s",     "swing-up cart speed" },
  { "SwingBal", "SWING_E_TARGET",  &SWING_E_TARGET, -0.1f,   0.2f,  "-",       "energy target (0 = just reaches upright)" },
  { "SwingBal", "SWING_E_SLOW",    &SWING_E_SLOW,    0.2f,   1.5f,  "-",       "how gradually the push eases off near the top" },
  { "SwingBal", "SWING_PHASE_DEG", &SWING_PHASE_DEG, 0.0f,  90.0f,  "deg",     "push timing lead (0 = at the bottom, 90 = at swing ends)" },
  { "SwingBal", "K_X_SWING",       &K_X_SWING,       0.0f,   3.0f,  "1/s",     "pull back toward centre while swinging" },
  // Shared: cart speed loop (also used by the brake and by StepTest)
  { NULL,       "KV_P",            &KV_P,            0.5f,   3.0f,  "1/(m/s)", "speed loop gain" },
  { NULL,       "V_FF",            &V_FF,            0.1f,   0.5f,  "1/(m/s)", "speed loop feed-forward" },
  { NULL,       "V_MAX",           &V_MAX,           0.2f,   1.0f,  "m/s",     "top cart speed" },
};
```

- [ ] **Step 3: Replace the serial setup in `setup()` and the whole `loop()` (`cart_single.ino`)**

Replace:

```cpp
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== ESP32 Single-Board Pendulum Controller v" FW_VERSION " ===");
  captureParamDefaults();   // the starting values become the "defaults"
```

with:

```cpp
  // D lines run at 500 Hz (~20 KB/s): a TX buffer keeps printing from
  // blocking the control step, and 921600 baud leaves plenty of headroom
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  Serial.println();
  Serial.println("=== ESP32 Single-Board Pendulum Controller v" FW_VERSION " ===");
  registerExperiments();    // fills EXPS[], selects EXPS[0]
  captureParamDefaults();   // the starting values become the "defaults"
```

Replace everything from the line `// Loop — safety checks, then one swing-up/balance step per 2 ms sample` to the end of the file with:

```cpp
// Loop — hardware safety, then one experiment step per 2 ms sample
// =============================================================================
void loop() {
  readEncoders();
  pollSerial();

  // --- Hardware safety: limit switches stop the motor ---
  // Must read pressed for LIMIT_CONFIRM_MS: hard motor reversals put noise
  // spikes on the limit inputs (false hits at x = -121 mm and +64 mm, both at
  // full-power reversals). A real hit lasts far longer than that.
  static unsigned long limitSince = 0;
  static bool          limitSeen  = false;
  if (anyLimitHit()) {
    if (!limitSeen) { limitSeen = true; limitSince = millis(); }
  } else {
    limitSeen = false;
  }
  if (limitSeen && millis() - limitSince >= LIMIT_CONFIRM_MS && systemEnabled) {
    stopMotor();
    Serial.printf("LIMIT HIT (%s) — motor stopped. Press START to re-home.\n",
                  leftLimitHit() ? "left" : "right");
    endRun("limit-switch");
  }

  // --- Stop trigger (button or serial) ---
  if (stopTriggered() && systemEnabled) {
    stopMotor();
    Serial.println("STOP triggered — motor stopped. Press START to re-home.");
    endRun("stop");
    serialStop = false;
    delay(200);
    return;
  }

  // --- Not enabled: hold motor stopped, re-home on START ---
  if (!systemEnabled) {
    stopMotor();
    if (startTriggered()) {
      delay(200);  // debounce
      autoRehomes = 0;   // a manual START gives a fresh retry budget
      runHoming();
    } else {
      delay(10);
    }
    return;
  }

  // --- Hanging reference, then the selected experiment ---
  runStep();
}
```

(The `// ====...` banner line above `// Loop —` stays as it is.)

- [ ] **Step 4: Point homing at the new run start (`hardware.ino`)**

In `runHomingSteps()` replace `  standaloneStart();` with `  runStart();`.

- [ ] **Step 5: Replace the whole of `protocol.ino`**

```cpp
// =============================================================================
// USB serial command interface (for the bridge / website)
// One command per line. Replies start with "ok ", "err ", "param ",
// "params end", "status ", "exp " or "io ", so they can be told apart from
// telemetry ("D ", "SWING", "BAL", "HANG", ...).
//   home | start | run  home, then run the selected experiment (only while idle)
//   stop                stop now (always accepted, also during homing)
//   status              -> status STATE EXPERIMENT
//   exp list            -> exp NAME | title  (one per experiment), then exp end
//   exp select NAME     experiment for the next run (only while idle)
//   params              -> param NAME VALUE MIN MAX DEFAULT UNIT | description
//                          (shared settings + the selected experiment's)
//   set NAME VALUE      change a setting (only while idle, within its limits)
//   defaults            restore every setting to its default (only while idle)
//   io                  -> io cart N pend N left 0|1 right 0|1 start 0|1 stop 0|1
//                          pins cartA cartB pendA pendB (encoder pin levels)
// =============================================================================
bool isIdle() { return !systemEnabled && !homingActive; }

const char* stateName() {
  if (homingActive)      return "HOMING";
  if (!systemEnabled)    return "IDLE";
  if (phase == PH_HANG)  return "HANG";
  if (phase == PH_BRAKE) return "BRAKE";
  if (phase == PH_EXP)   return curExp->state();
  return "IDLE";
}

// A setting is visible when it is shared or belongs to the selected experiment
bool paramVisible(int i) {
  return PARAMS[i].exp == NULL || strcmp(PARAMS[i].exp, curExp->name) == 0;
}

void captureParamDefaults() {
  for (int i = 0; i < N_PARAMS; i++) PARAMS[i].def = *PARAMS[i].value;
}

int findParam(const char* name) {
  for (int i = 0; i < N_PARAMS; i++)
    if (paramVisible(i) && strcasecmp(PARAMS[i].name, name) == 0) return i;
  return -1;
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    if (!paramVisible(i)) continue;
    const Param& p = PARAMS[i];
    Serial.printf("param %s %g %g %g %g %s | %s\n",
                  p.name, *p.value, p.lo, p.hi, p.def, p.unit, p.desc);
  }
  Serial.println("params end");
}

// "run start NAME K=V ..." with every setting the run uses, so each saved
// run records what produced it
void printRunStart() {
  Serial.printf("run start %s", curExp->name);
  for (int i = 0; i < N_PARAMS; i++)
    if (paramVisible(i)) Serial.printf(" %s=%g", PARAMS[i].name, *PARAMS[i].value);
  Serial.println();
}

int findExperiment(const char* name) {
  for (int i = 0; i < N_EXPS; i++)
    if (strcasecmp(EXPS[i]->name, name) == 0) return i;
  return -1;
}

// "exp list" / "exp select NAME" (strtok already consumed "exp")
void handleExpCommand() {
  char* sub  = strtok(NULL, " \t");
  char* name = strtok(NULL, " \t");
  if (sub && !strcasecmp(sub, "list")) {
    for (int i = 0; i < N_EXPS; i++) Serial.printf("exp %s | %s\n", EXPS[i]->name, EXPS[i]->title);
    Serial.println("exp end");
  } else if (sub && !strcasecmp(sub, "select") && name) {
    if (!isIdle()) { Serial.printf("err exp busy (%s)\n", stateName()); return; }
    int i = findExperiment(name);
    if (i < 0) { Serial.printf("err exp %s unknown experiment\n", name); return; }
    curExp = EXPS[i];
    Serial.printf("ok exp %s\n", curExp->name);
  } else {
    Serial.println("err exp usage: exp list | exp select NAME");
  }
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "stop")) {
    serialStop = true;
    Serial.println("ok stop");
  } else if (!strcasecmp(cmd, "home") || !strcasecmp(cmd, "start") || !strcasecmp(cmd, "run")) {
    if (!isIdle()) { Serial.printf("err home busy (%s)\n", stateName()); return; }
    serialStart = true;
    Serial.println("ok home");
  } else if (!strcasecmp(cmd, "status")) {
    Serial.printf("status %s %s\n", stateName(), curExp->name);
  } else if (!strcasecmp(cmd, "exp")) {
    handleExpCommand();
  } else if (!strcasecmp(cmd, "params")) {
    printParams();
  } else if (!strcasecmp(cmd, "defaults")) {
    if (!isIdle()) { Serial.printf("err defaults busy (%s)\n", stateName()); return; }
    for (int i = 0; i < N_PARAMS; i++) *PARAMS[i].value = PARAMS[i].def;
    Serial.println("ok defaults");
  } else if (!strcasecmp(cmd, "set")) {
    char* name = strtok(NULL, " \t");
    char* val  = strtok(NULL, " \t");
    if (!name || !val) { Serial.println("err set usage: set NAME VALUE"); return; }
    int i = findParam(name);
    if (i < 0) { Serial.printf("err %s unknown setting\n", name); return; }
    if (!isIdle()) { Serial.printf("err %s busy (%s): change settings while idle\n", PARAMS[i].name, stateName()); return; }
    char* end;
    float f = strtof(val, &end);
    if (end == val || *end) { Serial.printf("err %s not a number\n", PARAMS[i].name); return; }
    if (!(f >= PARAMS[i].lo && f <= PARAMS[i].hi)) {   // also rejects NaN
      Serial.printf("err %s range %g..%g\n", PARAMS[i].name, PARAMS[i].lo, PARAMS[i].hi);
      return;
    }
    *PARAMS[i].value = f;
    Serial.printf("ok %s %g\n", PARAMS[i].name, f);
  } else if (!strcasecmp(cmd, "io")) {
    // Raw encoder pin levels too: if a level toggles while its count stays put,
    // the fault is in the counting; if it never toggles, it's the wiring.
    readEncoders();
    Serial.printf("io cart %ld pend %ld left %d right %d start %d stop %d"
                  "  pins cartA %d cartB %d pendA %d pendB %d\n",
                  (long)cart_position, (long)pend_position, leftLimitHit(), rightLimitHit(),
                  digitalRead(START_BTN) == LOW, digitalRead(STOP_BTN) == HIGH,
                  digitalRead(CART_ENC_A), digitalRead(CART_ENC_B),
                  digitalRead(PEND_ENC_A), digitalRead(PEND_ENC_B));
  } else {
    Serial.printf("err %s unknown command (home, run, stop, status, exp, params, set, defaults, io)\n", cmd);
  }
}

// Non-blocking: call often, including inside homing loops so "stop" works there
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      handleCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// STOP check used inside the homing routine. Homing used to ignore STOP for its
// whole ~10 s, which matters once a website can start the rig.
bool homingStopped() {
  pollSerial();
  if (!stopTriggered()) return false;
  stopMotor();
  Serial.println("STOP during homing — aborted.");
  return true;
}
```

- [ ] **Step 6: Replace the whole of `control.ino`**

```cpp
// =============================================================================
// Run control: state estimate, hanging reference, software safety and the run
// lifecycle around the selected experiment:
//   homing -> runStart() -> PH_HANG (hanging reference) -> beginExperiment()
//   prints "run start ..." -> PH_EXP: curExp->step() + one "D" line per 2 ms
//   -> endRun(reason) prints "run end REASON".
// A software fault (soft limit, overspeed) brakes first (PH_BRAKE).
// Sign conventions (after cartDir / PEND_SIGN):
//   x     > 0 : cart right of centre (m)
//   theta > 0 : top of pendulum leans right; 0 = upright, +/-pi = hanging (rad)
//   u     > 0 : drive right
// =============================================================================
const char* brakeReason = "";   // "run end" reason once the brake has stopped the cart

float wrapPi(float a) {
  a = fmodf(a + PI, 2.0f * PI);
  if (a < 0) a += 2.0f * PI;
  return a - PI;
}

float halfTrackM() { return 0.5f * track_total * CART_M_PER_COUNT; }

// u in [-1, 1] -> PWM RUN_MIN_PWM..maxPwm, starting just under the stall PWM
void driveUCap(float u, int maxPwm) {
  u = constrain(u, -1.0f, 1.0f);
  uOut = u;
  if (fabsf(u) < U_EPS) { stopMotor(); return; }
  int pwm = RUN_MIN_PWM + (int)(fabsf(u) * (maxPwm - RUN_MIN_PWM));
  if (u > 0) driveRight(min(pwm + RUN_RIGHT_EXTRA_PWM, 255));
  else       driveLeft(pwm);
}

// Speed loop: drive the cart at vRef m/s (> 0 = right)
void speedDrive(float vRef) {
  vRef = constrain(vRef, -V_MAX, V_MAX);
  vRefOut = vRef;
  driveUCap(V_FF * vRef + KV_P * (vRef - xDot), SPEED_MAX_PWM);
}

// Motor command in volts (Feedback convention, +-U_FULL_SCALE_V = full scale),
// through the same output mapping and PWM caps as the speed loop
void motorVolts(float u) {
  driveUCap(u / U_FULL_SCALE_V, SPEED_MAX_PWM);
}

// State change line for the bridge and the log: ">> SWING", ">> BALANCE", ...
void announce(const char* state) {
  Serial.printf(">> %s\n", state);
}

// After homing: motor off, measure the hanging reference (PH_HANG)
void runStart() {
  stopMotor();
  readEncoders();
  uOut       = 0;
  vRefOut    = 0;
  haveSample = false;
  phase      = PH_HANG;
  hangMin    = hangMax = pend_position;
  hangSum    = 0;
  hangN      = 0;
  hangStart  = millis();
  Serial.println("Standalone: waiting for the pendulum to hang still...");
}

// Update angle, position and filtered velocities from the latest sample
void updateState() {
  unsigned long now = micros();
  const float k = 2.0f * PI / PEND_COUNTS_PER_REV;
  if (!haveSample) {
    lastCart = cart_position;
    lastPend = pend_position;
    lastSampleUs = now;
    thetaDot = xDot = 0;
    haveSample = true;
  }
  float dt = constrain((now - lastSampleUs) * 1e-6f, 0.0005f, 0.02f);
  stateDt  = dt;
  float a  = dt / (VEL_FILTER_S + dt);
  float thetaRate = PEND_SIGN * (pend_position - lastPend) * k / dt;
  float xRate     = cartDir * (cart_position - lastCart) * CART_M_PER_COUNT / dt;
  thetaDot += a * (thetaRate - thetaDot);
  xDot     += a * (xRate - xDot);
  lastCart = cart_position;
  lastPend = pend_position;
  lastSampleUs = now;

  theta  = wrapPi(PEND_SIGN * (pend_position - pendDownRef) * k - PI) - PEND_TRIM_DEG * DEG_TO_RAD;
  xPos   = cartDir * cart_position * CART_M_PER_COUNT;   // zeroed at centre by homing
  energy = 0.5f * sq(thetaDot / PEND_OMEGA0) + cosf(theta) - 1.0f;
}

// Hanging reference = average over exactly HANG_PERIODS swings. Less biased
// by bearing friction than wherever the pendulum happens to stop.
void stepHang() {
  stopMotor();
  if (pend_position < hangMin) hangMin = pend_position;
  if (pend_position > hangMax) hangMax = pend_position;
  hangSum += pend_position;
  hangN++;
  const unsigned long windowMs =
    (unsigned long)(HANG_PERIODS * 2.0f * PI / PEND_OMEGA0 * 1000.0f);
  if (millis() - hangStart < windowMs) return;

  if (hangMax - hangMin <= HANG_AMP_COUNTS && hangN > 0) {
    pendDownRef = (int32_t)((hangSum + (int64_t)hangN / 2) / (int64_t)hangN);
    Serial.printf("Hanging reference: %ld (swing %ld counts)\n",
                  (long)pendDownRef, (long)(hangMax - hangMin));
    haveSample = false;
    updateState();
    beginExperiment();
  } else {
    hangMin = hangMax = pend_position;
    hangSum = 0;
    hangN   = 0;
    hangStart = millis();
  }
}

// Hanging reference known: open the run and hand over to the experiment
void beginExperiment() {
  phase      = PH_EXP;
  runActive  = true;
  runStartMs = millis();
  expRef     = NAN;
  lastPrint  = 0;
  printRunStart();
  curExp->start();
}

// One line per control step while a run is open. theta here is continuous
// (0 = upright, +pi = hanging at run start), unlike the wrapped control angle,
// so swings through the bottom don't jump between +pi and -pi in the data.
void printDataLine() {
  const float k = 2.0f * PI / PEND_COUNTS_PER_REV;
  float thCont = PEND_SIGN * (pend_position - pendDownRef) * k + PI - PEND_TRIM_DEG * DEG_TO_RAD;
  Serial.printf("D %lu %.4f %.4f %.3f %.4f\n", millis() - runStartMs, xPos, thCont,
                uOut * U_FULL_SCALE_V, expRef);
}

// Software faults: NULL if none, else the "run end" reason
const char* safetyFault() {
  float softLim  = SOFT_LIMIT_FRAC * halfTrackM();
  float stopDist = xDot * xDot / (2.0f * BRAKE_DECEL);
  bool  outward  = xPos * xDot > 0;
  if (fabsf(xDot) > SPEED_TRIP) return "overspeed";
  if (fabsf(xPos) > softLim || (outward && fabsf(xPos) + stopDist > softLim)) return "soft-limit";
  return NULL;
}

// Powered stop through the speed loop (coasting only slows ~1.3 m/s^2)
void startBrake(const char* reason) {
  brakeReason = reason;
  brakeRehome = curExp->autoRehome && strcmp(reason, "soft-limit") == 0
                && autoRehomes < AUTO_REHOME_MAX;
  Serial.printf("%s: cart at %.0f mm, %.2f m/s — braking, then %s\n",
                strcmp(reason, "soft-limit") == 0 ? "SOFT LIMIT" : "OVERSPEED",
                xPos * 1000.0f, xDot,
                brakeRehome ? "re-homing." : "stopping. Press START to re-home.");
  readyLed(false);
  brakeStart = millis();
  phase = PH_BRAKE;
  announce("BRAKE");
}

void stepBrake() {
  // Timeout in case the loop can't stop it
  if (fabsf(xDot) < BRAKE_DONE_V || millis() - brakeStart > BRAKE_MS) {
    stopMotor();
    uOut = vRefOut = 0;
    Serial.printf("Stopped at %.0f mm.\n", xPos * 1000.0f);
    printDataLine();
    endRun(brakeReason);
    if (brakeRehome) {
      autoRehomes++;
      Serial.printf("Auto re-home %d/%d\n", autoRehomes, AUTO_REHOME_MAX);
      delay(500);      // let the cart and pendulum settle a moment
      runHoming();     // re-homes, then runStart() runs the experiment again
    }
  } else {
    speedDrive(0);
  }
}

// Motor off, "run end REASON" (once per run), back to idle
void endRun(const char* reason) {
  stopMotor();
  uOut = vRefOut = 0;
  if (runActive) Serial.printf("run end %s\n", reason);
  runActive     = false;
  phase         = PH_IDLE;
  systemEnabled = false;
  readyLed(false);
}

// One step per 2 ms encoder sample (called from loop() while enabled)
void runStep() {
  if (!newSample) return;
  newSample = false;

  if (phase == PH_HANG) {
    stepHang();
    if (phase == PH_HANG && millis() - lastPrint >= PRINT_MS) {
      lastPrint = millis();
      Serial.printf("HANG  pend %ld  swing %ld\n", (long)pend_position, (long)(hangMax - hangMin));
    }
    return;
  }

  updateState();

  if (phase == PH_EXP && curExp->safety) {
    const char* fault = safetyFault();
    if (fault) startBrake(fault);
  }

  if (phase == PH_BRAKE) {
    stepBrake();
    if (phase == PH_BRAKE) printDataLine();
    return;
  }

  if (phase == PH_EXP) {
    bool more = curExp->step(stateDt);
    printDataLine();
    if (!more) endRun("done");
  }
}
```

- [ ] **Step 7: Add the SwingBal experiment at the end of `exp_swingbal.ino`**

Append (the file already contains `swingV`, `balanceGains`, `balanceStart` and `balanceV` from Task 1):

```cpp

// =============================================================================
// SwingBal experiment: energy swing-up, then pole-placement balance, both on
// the cart speed loop (the controller that has worked since 2026-09-30).
// Re-homes and swings up again after a soft-limit stop (AUTO_REHOME_MAX).
// =============================================================================
bool sbBalancing = false;

void sbStart() {
  lastFlipMs  = millis();
  bobSide     = 0;
  pumpBlocked = false;
  energyEnd   = -2;
  swingDir    = 0;
  // balTrim is NOT reset here: the upright offset is the same after a re-home
  // (5.5-6.4 deg on every run of 2026-10-02), and starting from 0 after an
  // automatic re-home parked the balanced cart at -384 mm, into the soft limit.
  balanceGains();
  sbBalancing = false;
  announce("SWING");
}

bool sbStep(float dt) {
  if (!sbBalancing) {
    if (fabsf(theta) < CATCH_ANGLE_DEG * DEG_TO_RAD && fabsf(thetaDot) < CATCH_RATE) {
      sbBalancing = true;
      announce("BALANCE");
      balanceStart();
      speedDrive(balanceV());
    } else {
      speedDrive(swingV());
    }
  } else if (fabsf(theta) > DROP_ANGLE_DEG * DEG_TO_RAD) {
    sbBalancing = false;
    announce("SWING");
    speedDrive(swingV());
  } else {
    speedDrive(balanceV());
    if (autoRehomes && millis() - balStartMs > AUTO_REHOME_GOOD_MS) {
      autoRehomes = 0;   // balance held: allow the full retry budget again
      Serial.println("Balance holding — auto re-home count reset.");
    }
  }

  // Human-readable lines for the console readouts (balance prints faster)
  unsigned long every = sbBalancing ? STEP_PRINT_MS : PRINT_MS;
  if (millis() - lastPrint >= every) {
    lastPrint = millis();
    if (sbBalancing) {
      Serial.printf("BAL   th %7.1f deg  w %6.2f  x %6.1f mm  v %6.3f  vr %5.2f  trim %5.2f deg  u %5.2f\n",
                    theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, xDot, vRefOut,
                    balTrim * RAD_TO_DEG, uOut);
    } else {
      Serial.printf("SWING th %7.1f deg  w %6.2f  x %6.1f mm  v %6.3f  vr %5.2f  E %5.2f  Eend %5.2f  u %5.2f\n",
                    theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, xDot, vRefOut, energy,
                    energyEnd, uOut);
    }
  }
  return true;   // runs until STOP or a fault
}

const char* sbState() { return sbBalancing ? "BALANCE" : "SWING"; }

Experiment EXP_SWINGBAL = { "SwingBal", "Swing-up & balance", true, true, sbStart, sbStep, sbState };
```

- [ ] **Step 8: Create `exp_checks.ino`**

```cpp
// =============================================================================
// Rig checks, run like any experiment (they replace the SIGN_CHECK and
// STEP_TEST compile switches, so no reflash is needed to run them)
// =============================================================================

// --- SignCheck: motor off; angle and position printed for checking signs by hand
void scStart() {
  stopMotor();
  uOut = vRefOut = 0;
  announce("CHECK");
}

bool scStep(float dt) {
  stopMotor();
  uOut = vRefOut = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.printf("CHECK th %7.1f deg  w %6.2f  x %6.1f mm  v %6.3f  vr %5.2f  E %5.2f  Eend %5.2f  u %5.2f\n",
                  theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, xDot, vRefOut, energy,
                  energyEnd, uOut);
  }
  return true;   // until STOP
}

const char* scState() { return "CHECK"; }

// Motor off and moved by hand: no software soft limit
Experiment EXP_SIGNCHECK = { "SignCheck", "Sign check (motor off)", false, false, scStart, scStep, scState };

// --- StepTest: speed loop +-STEP_V for STEP_MS each, STEP_CYCLES times.
// Also checks the pendulum model: with the pendulum hanging, accelerating the
// cart toward +x must swing theta from 180 toward -179, -178...
void stStart() {
  stepStart = millis();
  announce("STEP");
}

bool stStep(float dt) {
  unsigned long t = millis() - stepStart;
  if (t >= 2UL * STEP_MS * STEP_CYCLES) {
    stopMotor();
    uOut = vRefOut = 0;
    Serial.println("Step test done.");
    return false;
  }
  speedDrive((t / STEP_MS) % 2 == 0 ? STEP_V : -STEP_V);
  if (millis() - lastPrint >= STEP_PRINT_MS) {
    lastPrint = millis();
    Serial.printf("STEP t %4lu  vr %5.2f  v %6.3f  u %5.2f  x %6.1f  th %7.2f  w %6.2f\n",
                  t, vRefOut, xDot, uOut, xPos * 1000.0f, theta * RAD_TO_DEG, thetaDot);
  }
  return true;
}

const char* stState() { return "STEP"; }

Experiment EXP_STEPTEST = { "StepTest", "Speed loop step test", false, true, stStart, stStep, stState };
```

- [ ] **Step 9: Create `zz_registry.ino`**

The name sorts last, so every experiment's `Experiment` instance is already defined above it:

```cpp
// =============================================================================
// Experiments offered by "exp list", in console order. EXPS[0] is selected at
// boot. Each later stage adds one line per new experiment.
// =============================================================================
void registerExperiments() {
  EXPS[N_EXPS++] = &EXP_SWINGBAL;
  EXPS[N_EXPS++] = &EXP_SIGNCHECK;
  EXPS[N_EXPS++] = &EXP_STEPTEST;
  curExp = EXPS[0];
}
```

- [ ] **Step 10: Switch the logger to 921600 baud (`cart/serial_log.py`)**

Replace `BAUD = 115200` with `BAUD = 921600`.

- [ ] **Step 11: Compile**

Run the firmware compile command.
Expected: `Sketch uses ... bytes`, no `error:` lines. A warning such as `unused parameter 'dt'` is fine. Also confirm nothing still refers to removed names:

Run: `cd /c/Users/super/Documents/ED2/ED2-Repo/cart/cart_single && grep -n "BalState\|BAL_NAMES\|balState\|SIGN_CHECK\|STEP_TEST\|standaloneStart\|standaloneStep\|enterState\|printRunParams" *.ino`
Expected: only the comment lines that mention `SIGN_CHECK` / `STEP_TEST` in the change notes. No code uses them.

- [ ] **Step 12: Rig check by the user (ask them; Claude reads the log)**

Ask the user to:
1. Flash `cart/cart_single`. The Arduino Serial Monitor, if used, must be set to **921600**.
2. Run `python cart/serial_log.py` and type, one per line: `exp list`, `status`, `run`.
3. Let it swing up and balance for about 20 s, then press STOP and wait for the `# rest drift` line.
4. Type `exp select StepTest`, then `run`, and let it finish.

Then read the newest `cart/logs/*.log`. Expected:
- `exp SwingBal | Swing-up & balance`, `exp SignCheck | ...`, `exp StepTest | ...`, `exp end`, then `status IDLE SwingBal`.
- `run start SwingBal BAL_MODE=0 ... V_MAX=0.8`, then about 500 `D` lines per second. Check with `awk '/^\S+ D /{print substr($1,1,8)}' LOG | uniq -c | sort -n | head`: every full second shows 490–505.
- `>> SWING`, `>> BALANCE`, `BAL` lines with `th` near 0 (trim now preset), and the cart parked within about ±20 mm early in the balance (no −300 mm run-out).
- `STOP triggered ...` followed by `run end stop`.
- A second run: `run start StepTest KV_P=1.5 V_FF=0.25 V_MAX=0.8`, `>> STEP`, `STEP` lines, `Step test done.`, `run end done`.
- `D` lines are well-formed. Count lines containing `�`: they should be rare (less than 0.1 % of `D` lines). If not, follow the 460800-baud fallback in the spec's risk notes and tell the user before changing anything.

- [ ] **Step 13: Commit (only if approved)**

```bash
git add cart/cart_single cart/serial_log.py
git commit -m "cart_single v11.1: experiments framework, 921600 baud D lines, SignCheck/StepTest experiments, PEND_TRIM_DEG 5.5"
```

---

### Task 3: Bridge run recording and CSV/JSON output (`runs.py`)

**Files:**
- Create: `pendulum-remote/bridge/runs.py`
- Create: `pendulum-remote/bridge/tests/conftest.py`, `pendulum-remote/bridge/tests/test_runs.py`
- Create: `pendulum-remote/bridge/requirements-dev.txt`
- Modify: `pendulum-remote/.gitignore`

**Interfaces:**
- Consumes: `matfile.write_mat` (Task 4). Until Task 4 exists, `save_run`'s `.mat` call fails, so Task 3's `test_save_run_*` tests are written here and pass once Task 4 lands. Do Task 4 immediately after Step 3 of this task, or write both before running the full suite.
- Produces:
  - `parse_d_line(line: str) -> tuple[float, ...] | None`: (t_s, x, theta, u, ref[, r]).
  - `parse_run_start(line: str) -> tuple[str, dict[str, float]] | None`.
  - `class Run(name, params, started, rows, results, reason, dropped)`, with `.columns`, `.column(i)`, `.duration`, `.stem`.
  - `class Recorder`: `on_line(line, now=None) -> list[tuple[str, object]]`, with events `("start", Run)`, `("sample", row)`, `("result", (key, value))`, `("end", Run)`. Also `abort(reason) -> Run | None`.
  - `run_summary(run, files: dict[str, str]) -> dict`.
  - `save_run(run, folder: Path) -> dict`: writes `<stem>.csv`, `<stem>.mat`, `<stem>.json` and returns the summary.

- [ ] **Step 1: Test setup files**

`pendulum-remote/bridge/requirements-dev.txt`:

```
-r requirements.txt
pytest>=8
scipy>=1.11
httpx>=0.27
```

`pendulum-remote/bridge/tests/conftest.py`:

```python
import sys
from pathlib import Path

# The bridge modules live next to bridge.py, not in a package
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
```

Append to `pendulum-remote/.gitignore`:

```
runs/
```

Run: `cd /c/Users/super/Documents/ED2/pendulum-remote && python -m pip install -r bridge/requirements-dev.txt`
Expected: SciPy installs (pytest and httpx already present).

- [ ] **Step 2: Write the failing tests (`bridge/tests/test_runs.py`)**

```python
import json
import math

import pytest

from runs import Recorder, Run, parse_d_line, parse_run_start, save_run


def test_parse_d_line():
    row = parse_d_line("D 1234 0.0123 3.1416 0.250 nan")
    assert row[:4] == (1.234, 0.0123, 3.1416, 0.25)
    assert math.isnan(row[4])
    assert parse_d_line("D 10 0 0 0 nan 0.5")[5] == 0.5          # optional r column


@pytest.mark.parametrize("bad", [
    "D 12 0.1",                       # too short
    "D x 0 0 0 nan",                  # not a number
    "D 12 nan 0 0 nan",               # x must be a number
    "D 1 2 3 4 5 6 7 8",              # too many fields
    "\ufffd\ufffdight 0 start 0",     # USB garbage
])
def test_parse_d_line_rejects(bad):
    assert parse_d_line(bad) is None


def test_parse_run_start():
    assert parse_run_start("run start CartPID P=27.8 I=50 junk D=nan") == ("CartPID", {"P": 27.8, "I": 50.0})
    assert parse_run_start("run end stop") is None


def test_recorder_full_run():
    rec = Recorder()
    ev = rec.on_line("run start SwingBal BAL_POLE=5", now=1000.0)
    assert ev[0][0] == "start" and ev[0][1].params == {"BAL_POLE": 5.0}
    assert rec.on_line("D 0 0 3.1416 0 nan")[0][0] == "sample"
    rec.on_line("D 2 0.001 3.14 0.1 nan")
    assert rec.on_line("result friction_pos_V 0.21") == [("result", ("friction_pos_V", 0.21))]
    ev = rec.on_line("run end stop")
    kind, run = ev[0]
    assert kind == "end" and run.reason == "stop" and len(run.rows) == 2
    assert run.results == {"friction_pos_V": 0.21} and run.started == 1000.0
    assert rec.run is None


def test_garbled_d_lines_are_counted_not_recorded():
    rec = Recorder()
    rec.on_line("run start SwingBal")
    rec.on_line("D 4 0.1 \ufffd\ufffd")
    rec.on_line("D 6 0.1 3.1 0 nan")
    run = rec.on_line("run end done")[0][1]
    assert run.dropped == 1 and len(run.rows) == 1


def test_lines_outside_a_run_are_ignored():
    rec = Recorder()
    assert rec.on_line("D 0 0 3.14 0 nan") == []
    assert rec.on_line("run end done") == []
    assert rec.on_line("result k 1") == []


def test_restart_without_end_closes_previous_as_interrupted():
    rec = Recorder()
    rec.on_line("run start SwingBal")
    rec.on_line("D 0 0 3.14 0 nan")
    ev = rec.on_line("run start StepTest")
    assert [k for k, _ in ev] == ["end", "start"]
    assert ev[0][1].reason == "interrupted" and len(ev[0][1].rows) == 1
    assert rec.run.name == "StepTest"


def test_abort_keeps_partial_run():
    rec = Recorder()
    rec.on_line("run start SwingBal")
    rec.on_line("D 0 0 3.14 0 nan")
    run = rec.abort("disconnected")
    assert run.reason == "disconnected" and len(run.rows) == 1
    assert rec.abort("disconnected") is None


def _sample_run(name="CartPID"):
    return Run(name, {"P": 27.8}, started=1791400000.0,
               rows=[(0.0, 0.0, 3.14, 0.0, math.nan), (0.002, 0.001, 3.13, 0.25, 0.05)],
               results={"k": 1.0}, reason="done")


def test_save_run_writes_csv_mat_and_summary(tmp_path):
    summary = save_run(_sample_run("Cart/PID"), tmp_path)
    csv_path = tmp_path / summary["files"]["csv"]
    assert summary["files"]["csv"].endswith("_Cart_PID.csv")
    lines = csv_path.read_text(encoding="utf-8").splitlines()
    assert "# experiment Cart/PID" in lines and "# end done" in lines and "# param P 27.8" in lines
    assert "t,x,theta,u,ref" in lines
    assert lines[-1] == "0.002,0.001,3.13,0.25,0.05"
    assert lines[-2] == "0.0,0.0,3.14,0.0,nan"
    assert (tmp_path / summary["files"]["mat"]).exists()
    on_disk = json.loads((tmp_path / (csv_path.stem + ".json")).read_text(encoding="utf-8"))
    assert on_disk == summary
    assert summary["samples"] == 2 and summary["duration"] == 0.002 and summary["reason"] == "done"


def test_save_run_never_overwrites(tmp_path):
    a = save_run(_sample_run(), tmp_path)
    b = save_run(_sample_run(), tmp_path)          # same name, same second
    assert a["files"]["csv"] != b["files"]["csv"]
    assert len(list(tmp_path.glob("*.csv"))) == 2
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cd /c/Users/super/Documents/ED2/pendulum-remote && python -m pytest bridge/tests/test_runs.py -q`
Expected: collection error, `ModuleNotFoundError: No module named 'runs'`.

- [ ] **Step 4: Implement `bridge/runs.py`**

```python
"""Run recording for the experiment protocol (cart_single firmware v11.1+).

The firmware brackets every run with "run start NAME K=V ..." and
"run end REASON" and prints one data line per 2 ms control step in between:

    D <t_ms> <x_m> <theta_rad> <u_V> <ref> [r]

Recorder turns those lines into Run objects; save_run() writes a finished run
as CSV and MATLAB .mat files plus a small .json summary for the console.
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path

from matfile import write_mat

BASE_COLUMNS = ("t", "x", "theta", "u", "ref")
EXTRA_COLUMNS = ("r",)               # optional trailing fields, in order
RUN_START = re.compile(r"^run start (\S+)(.*)$")
RUN_END = re.compile(r"^run end (\S+)")
RESULT = re.compile(r"^result (\S+) (\S+)")


def parse_d_line(line: str) -> tuple[float, ...] | None:
    """Data line -> (t_s, x, theta, u, ref, *extra), or None if malformed."""
    parts = line.split()
    if parts[:1] != ["D"] or not 6 <= len(parts) <= 6 + len(EXTRA_COLUMNS):
        return None
    try:
        vals = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in vals[:4]):    # t, x, theta, u; ref may be nan
        return None
    vals[0] /= 1000.0                                    # ms -> s
    return tuple(vals)


def parse_run_start(line: str) -> tuple[str, dict[str, float]] | None:
    m = RUN_START.match(line)
    if not m:
        return None
    params: dict[str, float] = {}
    for tok in m.group(2).split():
        key, sep, val = tok.partition("=")
        if not sep:
            continue
        try:
            f = float(val)
        except ValueError:
            continue
        if math.isfinite(f):
            params[key] = f
    return m.group(1), params


@dataclass
class Run:
    name: str
    params: dict[str, float]
    started: float                       # wall clock, time.time()
    rows: list[tuple[float, ...]] = field(default_factory=list)
    results: dict[str, float] = field(default_factory=dict)
    reason: str = ""
    dropped: int = 0                     # malformed "D" lines

    @property
    def columns(self) -> list[str]:
        width = max((len(r) for r in self.rows), default=len(BASE_COLUMNS))
        return list(BASE_COLUMNS) + list(EXTRA_COLUMNS[: width - len(BASE_COLUMNS)])

    def column(self, i: int) -> list[float]:
        return [r[i] if i < len(r) else math.nan for r in self.rows]

    @property
    def duration(self) -> float:
        return self.rows[-1][0] if self.rows else 0.0

    @property
    def stem(self) -> str:
        stamp = time.strftime("%Y-%m-%d_%H%M%S", time.localtime(self.started))
        return f"{stamp}_{re.sub(r'[^A-Za-z0-9_-]', '_', self.name)}"


class Recorder:
    """Feed every serial line to on_line(); it returns a list of events:
    ("start", Run), ("sample", row), ("result", (key, value)), ("end", Run)."""

    def __init__(self):
        self.run: Run | None = None

    def on_line(self, line: str, now: float | None = None) -> list[tuple[str, object]]:
        if line.startswith("D "):
            if self.run is None:
                return []
            row = parse_d_line(line)
            if row is None:
                self.run.dropped += 1
                return []
            self.run.rows.append(row)
            return [("sample", row)]
        start = parse_run_start(line)
        if start:
            events: list[tuple[str, object]] = []
            if self.run is not None:                     # no "run end": board reset mid-run
                events.append(("end", self._close("interrupted")))
            self.run = Run(start[0], start[1], time.time() if now is None else now)
            events.append(("start", self.run))
            return events
        m = RUN_END.match(line)
        if m:
            return [("end", self._close(m.group(1)))] if self.run is not None else []
        m = RESULT.match(line)
        if m and self.run is not None:
            try:
                val = float(m.group(2))
            except ValueError:
                return []
            if not math.isfinite(val):
                return []
            self.run.results[m.group(1)] = val
            return [("result", (m.group(1), val))]
        return []

    def abort(self, reason: str) -> Run | None:
        """Close an open run that will never get its "run end" (serial lost)."""
        return self._close(reason) if self.run is not None else None

    def _close(self, reason: str) -> Run:
        run, self.run = self.run, None
        run.reason = reason
        return run


def run_summary(run: Run, files: dict[str, str]) -> dict:
    return {"name": run.name, "started": run.started, "duration": run.duration,
            "reason": run.reason, "samples": len(run.rows), "dropped": run.dropped,
            "params": run.params, "results": run.results, "files": files}


def _fmt(v: float) -> str:
    return "nan" if math.isnan(v) else repr(float(v))


def save_run(run: Run, folder: Path) -> dict:
    """Write <stem>.csv, <stem>.mat and <stem>.json; return the summary."""
    folder.mkdir(parents=True, exist_ok=True)
    stem, n = run.stem, 2
    while (folder / f"{stem}.csv").exists():               # never overwrite a run
        stem, n = f"{run.stem}_{n}", n + 1
    cols = run.columns

    csv_path = folder / f"{stem}.csv"
    with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"# experiment {run.name}\n")
        f.write(f"# started {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(run.started))}\n")
        f.write(f"# end {run.reason}\n")
        if run.dropped:
            f.write(f"# dropped_lines {run.dropped}\n")
        for k, v in run.params.items():
            f.write(f"# param {k} {v:g}\n")
        for k, v in run.results.items():
            f.write(f"# result {k} {v:g}\n")
        f.write(",".join(cols) + "\n")
        for r in run.rows:
            f.write(",".join(_fmt(r[i]) if i < len(r) else "nan" for i in range(len(cols))) + "\n")

    data = {c: run.column(i) for i, c in enumerate(cols)}
    mat_path = folder / f"{stem}.mat"
    write_mat(mat_path,
              columns=data,
              matrices={"simout": [data["x"], data["theta"], data["u"], data["t"], data["ref"]]},
              structs={"params": run.params, "results": run.results},
              strings={"experiment": run.name, "reason": run.reason})

    summary = run_summary(run, {"csv": csv_path.name, "mat": mat_path.name})
    (folder / f"{stem}.json").write_text(json.dumps(summary), encoding="utf-8")
    return summary
```

- [ ] **Step 5: Run the tests**

Run: `python -m pytest bridge/tests/test_runs.py -q`
Expected: every test except the two `test_save_run_*` passes. Those two fail with `ModuleNotFoundError: No module named 'matfile'` until Task 4 exists.

- [ ] **Step 6: Commit (only if approved), after Task 4 makes the suite green**

```bash
cd /c/Users/super/Documents/ED2/pendulum-remote
git add .gitignore bridge/runs.py bridge/requirements-dev.txt bridge/tests/conftest.py bridge/tests/test_runs.py
git commit -m "bridge: record runs from run start/D/run end lines, save CSV + JSON summary"
```

---

### Task 4: MATLAB `.mat` writer (`matfile.py`)

**Files:**
- Create: `pendulum-remote/bridge/matfile.py`
- Create: `pendulum-remote/bridge/tests/test_matfile.py`

**Interfaces:**
- Produces: `write_mat(path, columns=None, matrices=None, structs=None, strings=None) -> None`:
  - `columns`: `dict[str, list[float]]`, each saved as an n×1 double.
  - `matrices`: `dict[str, list[list[float]]]`, a list of equal-length columns saved as n×k.
  - `structs`: `dict[str, dict[str, float]]`, each a 1×1 struct of doubles.
  - `strings`: `dict[str, str]`, each a 1×n char.
- Also produces `matlab_name(name: str) -> str`.

- [ ] **Step 1: Write the failing tests (`bridge/tests/test_matfile.py`)**

```python
import math

import pytest

sio = pytest.importorskip("scipy.io")

from matfile import matlab_name, write_mat


def test_columns_matrix_struct_and_strings(tmp_path):
    p = tmp_path / "a.mat"
    write_mat(p,
              columns={"t": [0.0, 0.002, 0.004], "ref": [math.nan] * 3},
              matrices={"simout": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]},
              structs={"params": {"P": 27.84, "BAL_MODE": 0.0}},
              strings={"experiment": "CartPID"})
    m = sio.loadmat(p)
    assert m["t"].shape == (3, 1) and m["t"][1, 0] == 0.002
    assert math.isnan(m["ref"][0, 0])
    assert m["simout"].shape == (3, 2) and m["simout"][2, 1] == 6.0 and m["simout"][0, 1] == 4.0
    params = m["params"][0, 0]
    assert params["P"][0, 0] == 27.84 and params["BAL_MODE"][0, 0] == 0.0
    assert m["experiment"][0] == "CartPID"


def test_empty_run_and_odd_names(tmp_path):
    p = tmp_path / "b.mat"
    write_mat(p,
              columns={"t": []},
              matrices={"simout": [[], [], [], [], []]},
              structs={"params": {"2X": 1.0, "A-B": 2.0, "L" * 40: 3.0}, "results": {}},
              strings={"reason": ""})
    m = sio.loadmat(p)
    assert m["t"].shape == (0, 1)
    assert m["simout"].shape == (0, 5)
    assert set(m["params"].dtype.names) == {"x2X", "A_B", "L" * 31}
    assert "results" in m and "reason" in m


def test_matlab_name():
    assert matlab_name("BAL_POLE") == "BAL_POLE"
    assert matlab_name("2X") == "x2X"
    assert matlab_name("") == "x"
    assert len(matlab_name("a" * 50)) == 31
```

- [ ] **Step 2: Run them to verify they fail**

Run: `python -m pytest bridge/tests/test_matfile.py -q`
Expected: `ModuleNotFoundError: No module named 'matfile'`.

- [ ] **Step 3: Implement `bridge/matfile.py`**

```python
"""Minimal MATLAB Level 5 MAT-file writer, so the bridge needs no SciPy.

Writes double column vectors and matrices, 1x1 structs of doubles and char
row vectors: enough for run files. MATLAB's load() and scipy.io.loadmat read
them. Format: MathWorks "MAT-File Format" (Level 5), little-endian.
"""
from __future__ import annotations

import re
import struct
import time
from pathlib import Path

MI_INT8, MI_UINT16, MI_INT32, MI_UINT32, MI_DOUBLE, MI_MATRIX = 1, 4, 5, 6, 9, 14
MX_STRUCT, MX_CHAR, MX_DOUBLE = 2, 4, 6
FIELD_NAME_LEN = 32                  # 31 characters + NUL


def _element(mtype: int, data: bytes) -> bytes:
    """Data element; small-element format for <= 4 bytes (as MATLAB writes it)."""
    if len(data) <= 4:
        return struct.pack("<HH", mtype, len(data)) + data.ljust(4, b"\0")
    return struct.pack("<II", mtype, len(data)) + data + b"\0" * (-len(data) % 8)


def _matrix(name: str, cls: int, dims: tuple[int, ...], body: bytes) -> bytes:
    inner = (_element(MI_UINT32, struct.pack("<II", cls, 0))
             + _element(MI_INT32, struct.pack(f"<{len(dims)}i", *dims))
             + _element(MI_INT8, name.encode("ascii"))
             + body)
    return struct.pack("<II", MI_MATRIX, len(inner)) + inner


def _doubles(name: str, dims: tuple[int, int], values: list[float]) -> bytes:
    return _matrix(name, MX_DOUBLE, dims,
                   _element(MI_DOUBLE, struct.pack(f"<{len(values)}d", *values)))


def _chars(name: str, text: str) -> bytes:
    codes = [ord(c) if ord(c) < 0x10000 else ord("?") for c in text]
    return _matrix(name, MX_CHAR, (1, len(codes)),
                   _element(MI_UINT16, struct.pack(f"<{len(codes)}H", *codes)))


def matlab_name(name: str) -> str:
    """A valid MATLAB identifier: letters, digits, _; starts with a letter; <= 31 chars."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not s or not s[0].isalpha():
        s = "x" + s
    return s[:31]


def _struct(name: str, fields: dict[str, float]) -> bytes:
    names: list[str] = []
    for key in fields:
        n = matlab_name(key)
        while n in names:                                  # keep field names unique
            n = (n[:27] + f"_{len(names)}")[:31]
        names.append(n)
    body = _element(MI_INT32, struct.pack("<i", FIELD_NAME_LEN))
    body += _element(MI_INT8, b"".join(n.encode("ascii").ljust(FIELD_NAME_LEN, b"\0") for n in names))
    for value in fields.values():
        body += _doubles("", (1, 1), [float(value)])
    return _matrix(matlab_name(name), MX_STRUCT, (1, 1), body)


def write_mat(path: Path,
              columns: dict[str, list[float]] | None = None,
              matrices: dict[str, list[list[float]]] | None = None,
              structs: dict[str, dict[str, float]] | None = None,
              strings: dict[str, str] | None = None) -> None:
    text = f"MATLAB 5.0 MAT-file, Platform: pendulum-remote, Created on: {time.asctime()}"
    out = [text.encode("ascii").ljust(116, b" ")[:116], b"\0" * 8, struct.pack("<H", 0x0100), b"IM"]
    for name, col in (columns or {}).items():
        out.append(_doubles(matlab_name(name), (len(col), 1), [float(v) for v in col]))
    for name, cols in (matrices or {}).items():
        rows = len(cols[0]) if cols else 0
        flat = [float(v) for c in cols for v in c]          # column-major
        out.append(_doubles(matlab_name(name), (rows, len(cols)), flat))
    for name, fields in (structs or {}).items():
        out.append(_struct(name, fields))
    for name, text_value in (strings or {}).items():
        out.append(_chars(matlab_name(name), text_value))
    Path(path).write_bytes(b"".join(out))
```

- [ ] **Step 4: Run all bridge tests so far**

Run: `python -m pytest bridge/tests -q`
Expected: all tests in `test_matfile.py` and `test_runs.py` pass (13 + 3 parametrised cases).

If `test_empty_run_and_odd_names` fails inside SciPy on the empty `results` struct, change `_struct` to write the field-name element with `_element(MI_INT8, b"")` only when `names` is non-empty, and rerun. (MATLAB writes a zero-length name element; SciPy versions differ in what they accept.)

- [ ] **Step 5: Commit (only if approved)**

```bash
git add bridge/matfile.py bridge/tests/test_matfile.py
git commit -m "bridge: minimal MAT 5 writer for run files"
```

(Then make Task 3's commit, if it was waiting.)

---

### Task 5: Bridge integration: 921600 baud, experiments, recording, `/runs`, mock

**Files:**
- Replace whole content: `pendulum-remote/bridge/bridge.py`
- Create: `pendulum-remote/bridge/tests/test_bridge.py`

**Interfaces:**
- Consumes: `runs.Recorder`, `runs.save_run`, `runs.run_summary` (Task 3).
- Produces, for the browser (Task 6), WebSocket messages:
  - `hello`: `{state, params, control, clients, experiments: [{name, title}], exp}`
  - `experiments`: `{experiments}`
  - `exp`: `{name}`
  - `run_start`: `{name, params, t}`
  - `run_end`: `{name, started, duration, reason, samples, dropped, params, results, files: {csv, mat}}`
  - `telemetry` with `mode: "D"`: `{t, run_t, x_mm, th_deg (wrapped, from upright), u, ref_mm (or null)}`
  - plus the existing `telemetry` SWING/BAL/HANG, `params`, `state`, `reply`, `event`, `log`, `control`.
- Commands accepted: `{cmd: "select", name}`, `{cmd: "start"}` (sends `run`), plus the existing ones.
- HTTP:
  - `GET /runs[?token=]` returns the summaries, newest first.
  - `GET /runs/<file>[?token=]` downloads one CSV or `.mat`.
- Methods:
  - `Bridge.check_heartbeat(now: float)`: async.
  - `Bridge.serial_lost()`.
  - `Bridge(runs_dir: Path = RUNS_DIR)`.

- [ ] **Step 1: Write the failing tests (`bridge/tests/test_bridge.py`)**

```python
import asyncio
import json

from fastapi.testclient import TestClient

import bridge as bridge_mod
from bridge import Bridge, make_app


class FakeWS:
    def __init__(self):
        self.sent = []

    async def send_text(self, text):
        self.sent.append(json.loads(text))


class FakeBridge(Bridge):
    def __init__(self, runs_dir):
        super().__init__(runs_dir)
        self.lines, self.pushed = [], []

    def send_line(self, line):
        self.lines.append(line)

    def push(self, msg):
        self.pushed.append(msg)

    def start(self):
        pass


def feed_run(b, n=25, end="run end stop"):
    b.handle_line("run start SwingBal BAL_POLE=5")
    for i in range(n):
        b.handle_line(f"D {2 * i} 0.01 3.1416 0.1 nan")
    b.handle_line(end)


def test_select_and_start_need_control(tmp_path):
    b, ws = FakeBridge(tmp_path), FakeWS()

    async def go():
        b.clients.add(ws)
        await b.command(ws, {"cmd": "select", "name": "CartPID"})
        assert b.lines == [] and ws.sent[-1]["text"] == "take control first"
        await b.command(ws, {"cmd": "claim"})
        await b.command(ws, {"cmd": "select", "name": "CartPID"})
        await b.command(ws, {"cmd": "select", "name": "x; stop"})      # rejected
        await b.command(ws, {"cmd": "start"})

    asyncio.run(go())
    assert b.lines == ["exp select CartPID", "run"]


def test_heartbeat_loss_during_run_sends_stop(tmp_path):
    b, ws = FakeBridge(tmp_path), FakeWS()

    async def go():
        b.clients.add(ws)
        await b.command(ws, {"cmd": "claim"})
        b.state = "BALANCE"
        await b.check_heartbeat(b.last_beat + 1.0)       # still within the timeout
        assert "stop" not in b.lines
        await b.check_heartbeat(b.last_beat + 10.0)

    asyncio.run(go())
    assert b.lines.count("stop") == 1 and b.controller is None


def test_run_lines_are_recorded_saved_and_decimated(tmp_path):
    b = FakeBridge(tmp_path)
    feed_run(b)
    kinds = [m["kind"] for m in b.pushed]
    assert kinds.count("telemetry") == 2                 # every 10th of 25 samples
    assert not any(m["kind"] == "log" and m["text"].startswith("D ") for m in b.pushed)
    tel = next(m for m in b.pushed if m["kind"] == "telemetry")
    assert tel["ref_mm"] is None and abs(abs(tel["th_deg"]) - 180) < 0.01 and tel["x_mm"] == 10.0
    end = next(m for m in b.pushed if m["kind"] == "run_end")
    assert end["samples"] == 25 and end["reason"] == "stop"
    assert (tmp_path / end["files"]["csv"]).exists() and (tmp_path / end["files"]["mat"]).exists()
    assert "status" in b.lines                           # asks for the new state after run end


def test_status_and_exp_lines(tmp_path):
    b = FakeBridge(tmp_path)
    b.handle_line("exp SwingBal | Swing-up & balance")
    b.handle_line("exp StepTest | Speed loop step test")
    b.handle_line("exp end")
    assert b.experiments == [{"name": "SwingBal", "title": "Swing-up & balance"},
                             {"name": "StepTest", "title": "Speed loop step test"}]
    b.handle_line("status BALANCE SwingBal")
    assert b.state == "BALANCE" and b.exp == "SwingBal"
    b.handle_line("ok exp StepTest")
    assert b.exp == "StepTest" and "params" in b.lines
    assert {"kind": "exp", "name": "StepTest"} in b.pushed


def test_serial_lost_saves_partial_run(tmp_path):
    b = FakeBridge(tmp_path)
    b.handle_line("run start SwingBal")
    b.handle_line("D 0 0 3.14 0 nan")
    b.serial_lost()
    end = next(m for m in b.pushed if m["kind"] == "run_end")
    assert end["reason"] == "disconnected" and end["samples"] == 1
    assert b.state == "OFFLINE"


def test_runs_endpoints(tmp_path, monkeypatch):
    b = FakeBridge(tmp_path)
    feed_run(b)
    client = TestClient(make_app(b))                     # no "with": lifespan (serial) not started
    r = client.get("/runs")
    assert r.status_code == 200 and r.json()[0]["reason"] == "stop"
    csv_name = r.json()[0]["files"]["csv"]
    assert client.get(f"/runs/{csv_name}").status_code == 200
    assert client.get("/runs/..%2Fbridge.py").status_code == 404
    assert client.get("/runs/missing.csv").status_code == 404
    assert client.get("/runs/" + csv_name.replace(".csv", ".json")).status_code == 404
    monkeypatch.setattr(bridge_mod, "TOKEN", "s3cret")
    assert client.get("/runs").status_code == 401
    assert client.get("/runs?token=s3cret").status_code == 200
    assert client.get(f"/runs/{csv_name}").status_code == 401
    assert client.get(f"/runs/{csv_name}?token=s3cret").status_code == 200
```

- [ ] **Step 2: Run them to verify they fail**

Run: `python -m pytest bridge/tests/test_bridge.py -q`
Expected: failures such as `TypeError: Bridge.__init__() takes 1 positional argument` or `AttributeError: 'FakeBridge' object has no attribute 'check_heartbeat'`.

- [ ] **Step 3: Replace `bridge/bridge.py` with the new version**

```python
"""Pendulum remote bridge: ESP32 (USB serial) <-> browsers (WebSocket).

    python bridge/bridge.py                 # auto-detect the rig's serial port
    python bridge/bridge.py --port COM3     # or name it
    python bridge/bridge.py --mock          # simulated rig, no hardware needed

Then open http://localhost:8000

- Talks to the single-board firmware (cart_single v11.1+) at 921600 baud:
  experiments, settings, run control and telemetry.
- Records every run (the firmware's "run start" / "D" / "run end" lines) and
  saves it to runs/ as CSV + MATLAB .mat, with a .json summary the console
  lists. Downloads: GET /runs and /runs/<file>.
- One controller at a time ("take control"). Anyone connected can press STOP.
- If the controlling browser disconnects or stops sending heartbeats while the
  rig is running, the bridge sends "stop".
- Optional password: set BRIDGE_TOKEN=... and open http://host:8000/?token=...

Close the Arduino serial monitor first: only one program can open the port.
"""

import argparse
import asyncio
import json
import math
import os
import random
import re
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import serial
import serial.tools.list_ports
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from runs import Recorder, run_summary, save_run

FRONTEND = Path(__file__).resolve().parent.parent / "frontend"
RUNS_DIR = Path(__file__).resolve().parent.parent / "runs"
TOKEN = os.environ.get("BRIDGE_TOKEN", "")
BAUD = 921600
HEARTBEAT_TIMEOUT_S = 3.0
STATUS_POLL_S = 2.0
DECIMATE = 10                    # D lines arrive at 500 Hz; browsers get every 10th (50 Hz)
RUN_FILE = re.compile(r"[A-Za-z0-9_-]+\.(csv|mat)")
EXP_NAME = re.compile(r"[A-Za-z0-9_]{1,32}")

# ----------------------------------------------------------------------------
# Parsing the firmware's human-readable serial lines
# ----------------------------------------------------------------------------
NUM = r"(-?[\d.]+)"
TELEMETRY = {
    "SWING": re.compile(rf"^SWING th\s+{NUM} deg\s+w\s+{NUM}\s+x\s+{NUM} mm\s+v\s+{NUM}\s+vr\s+{NUM}\s+E\s+{NUM}\s+Eend\s+{NUM}\s+u\s+{NUM}"),
    "BAL":   re.compile(rf"^BAL\s+th\s+{NUM} deg\s+w\s+{NUM}\s+x\s+{NUM} mm\s+v\s+{NUM}\s+vr\s+{NUM}\s+trim\s+{NUM} deg\s+u\s+{NUM}"),
}
FIELDS = {
    "SWING": ["th_deg", "w", "x_mm", "v", "vr", "E", "Eend", "u"],
    "BAL":   ["th_deg", "w", "x_mm", "v", "vr", "trim_deg", "u"],
}
HANG = re.compile(rf"^HANG\s+pend\s+{NUM}\s+swing\s+{NUM}")
PARAM = re.compile(r"^param (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) \| (.*)$")
EVENT_PREFIXES = ("SOFT LIMIT", "OVERSPEED", "LIMIT HIT", "STOP", "Stopped at", "Homing complete",
                  "ERROR", "Auto re-home", "Balance gains", "Hanging reference", "Balance holding",
                  "START from", "Homing: driving left", "=== ESP32", "Ready.", "WARNING",
                  "Step test done")
STATUS_REFRESH = ("STOP triggered", "Stopped at", "ERROR", "LIMIT HIT", "run end", "Ready.")
STATE_NAMES = {"STEP TEST": "STEP"}


def wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


class Bridge:
    """Shared state + the browser side. Subclasses provide send_line()."""

    def __init__(self, runs_dir: Path = RUNS_DIR):
        self.clients: set[WebSocket] = set()
        self.controller: WebSocket | None = None
        self.last_beat = 0.0
        self.state = "OFFLINE"
        self.params: list[dict] = []
        self._param_buf: list[dict] = []
        self.experiments: list[dict] = []
        self._exp_buf: list[dict] = []
        self.exp = ""
        self.runs_dir = runs_dir
        self.recorder = Recorder()
        self._n_samples = 0
        self.loop: asyncio.AbstractEventLoop | None = None

    # --- to browsers ---
    async def broadcast(self, msg: dict):
        data = json.dumps(msg)
        dead = []
        for ws in self.clients:
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.drop(ws)

    def push(self, msg: dict):
        """Thread-safe broadcast (from the serial reader thread)."""
        if self.loop:
            self.loop.call_soon_threadsafe(asyncio.ensure_future, self.broadcast(msg))

    def snapshot(self, ws: WebSocket) -> dict:
        return {"kind": "hello", "state": self.state, "params": self.params,
                "control": self._role(ws), "clients": len(self.clients),
                "experiments": self.experiments, "exp": self.exp}

    def _role(self, ws: WebSocket) -> str:
        if self.controller is ws:
            return "you"
        return "other" if self.controller else "free"

    async def announce_control(self):
        for ws in list(self.clients):
            try:
                await ws.send_text(json.dumps({"kind": "control", "control": self._role(ws),
                                               "clients": len(self.clients)}))
            except Exception:
                pass

    # --- from the board ---
    def handle_line(self, line: str):
        line = line.strip()
        if not line:
            return
        if line.startswith(("D ", "run start", "run end", "result ")):
            for kind, payload in self.recorder.on_line(line):
                self._on_run_event(kind, payload)
            if line.startswith("D "):
                return                          # 500 Hz: recorded, not logged as text
        for kind, rx in TELEMETRY.items():
            m = rx.match(line)
            if m:
                vals = dict(zip(FIELDS[kind], (float(g) for g in m.groups())))
                self.push({"kind": "telemetry", "mode": kind, "t": time.time(), **vals})
                self._set_state("BALANCE" if kind == "BAL" else "SWING")
                return
        m = HANG.match(line)
        if m:
            self.push({"kind": "telemetry", "mode": "HANG", "t": time.time(),
                       "pend": float(m.group(1)), "swing": float(m.group(2))})
            self._set_state("HANG")       # the firmware enters HANG without a ">>" line
            return
        if line.startswith("Homing: driving left"):
            self._set_state("HOMING")     # no ">>" line when homing starts either
        if line.startswith(">> "):
            self._set_state(line[3:].strip())
        elif line.startswith("status "):
            parts = line[7:].split()
            if parts:
                self._set_state(parts[0])
                if len(parts) > 1:
                    self._set_exp(parts[1])
        elif line.startswith("exp ") and " | " in line:
            name, _, title = line[4:].partition(" | ")
            self._exp_buf.append({"name": name.strip(), "title": title.strip()})
        elif line == "exp end":
            self.experiments, self._exp_buf = self._exp_buf, []
            self.push({"kind": "experiments", "experiments": self.experiments})
        elif line.startswith("param "):
            m = PARAM.match(line)
            if m:
                name, val, lo, hi, de, unit, desc = m.groups()
                self._param_buf.append({"name": name, "value": float(val), "min": float(lo),
                                        "max": float(hi), "default": float(de),
                                        "unit": "" if unit == "-" else unit, "desc": desc})
        elif line == "params end":
            self.params, self._param_buf = self._param_buf, []
            self.push({"kind": "params", "params": self.params})
        elif line.startswith("ok ") or line.startswith("err "):
            ok = line.startswith("ok ")
            self.push({"kind": "reply", "ok": ok, "text": line[3 if ok else 4:]})
            words = line.split()
            if ok and len(words) > 2 and words[1] == "exp":
                self._set_exp(words[2])
            if ok and len(words) > 1 and words[1] not in ("stop", "home"):
                self.send_line("params")     # a setting or the experiment changed
        else:
            if line.startswith(EVENT_PREFIXES):
                self.push({"kind": "event", "text": line, "t": time.time()})
            if line.startswith(STATUS_REFRESH) or "aborted" in line:
                self.send_line("status")
        self.push({"kind": "log", "text": line})

    def _on_run_event(self, kind: str, payload):
        if kind == "start":
            self._n_samples = 0
            self.push({"kind": "run_start", "name": payload.name, "params": payload.params,
                       "t": payload.started})
        elif kind == "sample":
            self._n_samples += 1
            if self._n_samples % DECIMATE == 0:
                t, x, th, u, ref = payload[:5]
                self.push({"kind": "telemetry", "mode": "D", "t": time.time(), "run_t": t,
                           "x_mm": x * 1000.0, "th_deg": wrap_deg(math.degrees(th)), "u": u,
                           "ref_mm": None if math.isnan(ref) else ref * 1000.0})
        elif kind == "result":
            key, value = payload
            self.push({"kind": "event", "text": f"Result: {key} = {value:g}", "t": time.time()})
        elif kind == "end":
            try:
                summary = save_run(payload, self.runs_dir)
            except OSError as e:
                print(f"[bridge] could not save the run: {e}")
                summary = run_summary(payload, {})
            self.push({"kind": "run_end", **summary})

    def serial_lost(self):
        """The board is gone: keep whatever the open run recorded."""
        run = self.recorder.abort("disconnected")
        if run:
            self._on_run_event("end", run)
        self._set_state("OFFLINE")

    def _set_state(self, s: str):
        s = STATE_NAMES.get(s, s)
        if s != self.state:
            self.state = s
            self.push({"kind": "state", "state": s})

    def _set_exp(self, name: str):
        if name != self.exp:
            self.exp = name
            self.push({"kind": "exp", "name": name})

    # --- commands from a browser ---
    async def command(self, ws: WebSocket, msg: dict):
        cmd = msg.get("cmd")
        if cmd == "ping":
            if ws is self.controller:
                self.last_beat = time.time()
            return
        if cmd == "stop":                       # anyone may stop
            self.send_line("stop")
            return
        if cmd == "claim":
            if self.controller is None:
                self.controller, self.last_beat = ws, time.time()
            await self.announce_control()
            return
        if cmd == "release":
            if ws is self.controller:
                await self.release(stop_if_running=True)
            return
        if cmd == "params":
            self.send_line("params")
            return
        if ws is not self.controller:
            await ws.send_text(json.dumps({"kind": "reply", "ok": False,
                                           "text": "take control first"}))
            return
        if cmd == "start":
            self.send_line("run")
        elif cmd == "select":
            name = str(msg.get("name", ""))
            if not EXP_NAME.fullmatch(name):
                await ws.send_text(json.dumps({"kind": "reply", "ok": False, "text": "bad experiment"}))
                return
            self.send_line(f"exp select {name}")
        elif cmd == "defaults":
            self.send_line("defaults")
        elif cmd == "set":
            name, value = str(msg.get("name", "")), msg.get("value")
            if not re.fullmatch(r"[A-Z_]+", name) or not isinstance(value, (int, float)) \
               or not math.isfinite(value):
                await ws.send_text(json.dumps({"kind": "reply", "ok": False, "text": "bad set"}))
                return
            self.send_line(f"set {name} {value:g}")   # the board checks the limits

    async def release(self, stop_if_running: bool):
        if stop_if_running and self.state not in ("IDLE", "OFFLINE"):
            self.send_line("stop")
        self.controller = None
        await self.announce_control()

    async def drop(self, ws: WebSocket):
        self.clients.discard(ws)
        if ws is self.controller:
            await self.release(stop_if_running=True)
        else:
            await self.announce_control()

    async def check_heartbeat(self, now: float):
        """Stop the rig if the controller stopped sending heartbeats."""
        if self.controller and now - self.last_beat > HEARTBEAT_TIMEOUT_S:
            self.push({"kind": "event", "text": "Controller heartbeat lost — stopping.",
                       "t": time.time()})
            await self.release(stop_if_running=True)

    async def watchdog(self):
        last_poll = 0.0
        while True:
            await asyncio.sleep(0.5)
            await self.check_heartbeat(time.time())
            if time.time() - last_poll > STATUS_POLL_S:
                last_poll = time.time()
                self.send_line("status")

    def start(self):
        raise NotImplementedError

    def send_line(self, line: str):
        raise NotImplementedError


# ----------------------------------------------------------------------------
# Real rig over USB serial
# ----------------------------------------------------------------------------
class SerialBridge(Bridge):
    def __init__(self, port: str | None, runs_dir: Path = RUNS_DIR):
        super().__init__(runs_dir)
        self.port = port
        self.ser: serial.Serial | None = None
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._reader, daemon=True).start()

    def _open(self) -> serial.Serial | None:
        ports = [self.port] if self.port else [
            p.device for p in serial.tools.list_ports.comports()
            if any(h in (p.description or "").upper() for h in ("CP210", "CH340", "CH910", "USB", "UART"))]
        for dev in ports:
            try:
                s = serial.Serial()
                s.port, s.baudrate, s.timeout = dev, BAUD, 0.2
                s.dtr = s.rts = False             # don't reset the ESP32
                s.open()
            except serial.SerialException:
                continue
            if self.port:
                return s
            # Auto-detect: the rig answers "status"
            s.write(b"status\n")
            deadline = time.time() + 1.5
            buf = b""
            while time.time() < deadline:
                buf += s.read(512)
                if b"status " in buf or b"SWING" in buf or b"BAL " in buf:
                    return s
            s.close()
        return None

    def _reader(self):
        while True:
            s = self._open()
            if not s:
                self._set_state("OFFLINE")
                time.sleep(2)
                continue
            self.ser = s
            print(f"[bridge] connected to {s.port}")
            self.send_line("status")
            self.send_line("exp list")
            self.send_line("params")
            buf = b""
            try:
                while True:
                    buf += s.read(4096)
                    *lines, buf = buf.split(b"\n")
                    for raw in lines:
                        self.handle_line(raw.decode("utf-8", errors="replace"))
            except serial.SerialException as e:
                print(f"[bridge] serial lost: {e}")
                self.ser = None
                self.serial_lost()
                time.sleep(1)

    def send_line(self, line: str):
        with self.lock:
            if self.ser:
                try:
                    self.ser.write((line + "\n").encode())
                except serial.SerialException:
                    pass


# ----------------------------------------------------------------------------
# Simulated rig (same lines as the firmware) for building the frontend
# ----------------------------------------------------------------------------
MOCK_EXPERIMENTS = [("SwingBal", "Swing-up & balance"),
                    ("SignCheck", "Sign check (motor off)"),
                    ("StepTest", "Speed loop step test")]
# Mirrors PARAMS in cart_single.ino v11.1: (experiment or None = shared, name, default, lo, hi, unit, desc)
MOCK_PARAMS = [
    ("SwingBal", "BAL_MODE", 0, 0, 1, "-", "0 = gains from BAL_POLE, 1 = manual gains"),
    ("SwingBal", "BAL_POLE", 5, 3, 7, "rad/s", "balance stiffness (all four gains computed from it)"),
    ("SwingBal", "BAL_K_TH", 62.8, 0, 200, "m/s2/rad", "manual gain: pendulum angle"),
    ("SwingBal", "BAL_K_THD", 11.2, 0, 40, "m/s2/(rad/s)", "manual gain: pendulum angular speed"),
    ("SwingBal", "BAL_K_X", 19.9, -50, 100, "m/s2/m", "manual gain: cart position"),
    ("SwingBal", "BAL_K_V", 15.9, -50, 80, "m/s2/(m/s)", "manual gain: cart speed"),
    ("SwingBal", "CATCH_ANGLE_DEG", 14.3, 5, 20, "deg", "start balancing inside this angle from upright"),
    ("SwingBal", "CATCH_RATE", 2, 1, 3, "rad/s", "...and only if turning slower than this"),
    ("SwingBal", "DROP_ANGLE_DEG", 34.4, 20, 45, "deg", "give up balancing past this angle"),
    ("SwingBal", "SWING_V", 0.4, 0.1, 0.6, "m/s", "swing-up cart speed"),
    ("SwingBal", "SWING_E_TARGET", 0.02, -0.1, 0.2, "-", "energy target (0 = just reaches upright)"),
    ("SwingBal", "SWING_E_SLOW", 0.6, 0.2, 1.5, "-", "how gradually the push eases off near the top"),
    ("SwingBal", "SWING_PHASE_DEG", 15, 0, 90, "deg", "push timing lead (0 = at the bottom, 90 = at swing ends)"),
    ("SwingBal", "K_X_SWING", 1, 0, 3, "1/s", "pull back toward centre while swinging"),
    (None, "KV_P", 1.5, 0.5, 3, "1/(m/s)", "speed loop gain"),
    (None, "V_FF", 0.25, 0.1, 0.5, "1/(m/s)", "speed loop feed-forward"),
    (None, "V_MAX", 0.8, 0.2, 1.0, "m/s", "top cart speed"),
]
MOCK_RUN_STATE = {"SwingBal": "SWING", "SignCheck": "CHECK", "StepTest": "STEP"}


class MockBridge(Bridge):
    def __init__(self, runs_dir: Path = RUNS_DIR):
        super().__init__(runs_dir)
        self.vals = {p[1]: float(p[2]) for p in MOCK_PARAMS}
        self.sim_exp = MOCK_EXPERIMENTS[0][0]
        self.sim_state = "IDLE"
        self.t0 = 0.0          # when sim_state last changed
        self.run_t0 = 0.0      # D line time 0
        self.next_d = 0.0      # time of the next D line
        self.inbox: list[str] = []

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def send_line(self, line: str):
        self.inbox.append(line)

    def _emit(self, line: str):
        self.handle_line(line)

    def _visible(self):
        return [p for p in MOCK_PARAMS if p[0] in (None, self.sim_exp)]

    def _goto(self, state: str, now: float):
        self.sim_state, self.t0 = state, now

    def _cmd(self, line: str):
        parts = line.split()
        if not parts:
            return
        cmd = parts[0].lower()
        idle = self.sim_state == "IDLE"
        if cmd == "status":
            self._emit(f"status {self.sim_state} {self.sim_exp}")
        elif cmd == "exp":
            if len(parts) == 2 and parts[1] == "list":
                for name, title in MOCK_EXPERIMENTS:
                    self._emit(f"exp {name} | {title}")
                self._emit("exp end")
            elif len(parts) == 3 and parts[1] == "select":
                match = next((n for n, _ in MOCK_EXPERIMENTS if n.lower() == parts[2].lower()), None)
                if not idle:
                    self._emit(f"err exp busy ({self.sim_state})")
                elif match is None:
                    self._emit(f"err exp {parts[2]} unknown experiment")
                else:
                    self.sim_exp = match
                    self._emit(f"ok exp {match}")
            else:
                self._emit("err exp usage: exp list | exp select NAME")
        elif cmd == "params":
            for _exp, n, d, lo, hi, u, desc in self._visible():
                self._emit(f"param {n} {self.vals[n]:g} {lo:g} {hi:g} {d:g} {u} | {desc}")
            self._emit("params end")
        elif cmd == "stop":
            self._emit("ok stop")
            if not idle:
                self._emit("STOP triggered — motor stopped. Press START to re-home.")
                if self.sim_state not in ("HOMING", "HANG"):
                    self._emit("run end stop")
                self._goto("IDLE", time.time())
        elif cmd in ("home", "start", "run"):
            if not idle:
                self._emit(f"err home busy ({self.sim_state})")
                return
            self._emit("ok home")
            self._emit("Homing: driving left...")
            self._goto("HOMING", time.time())
        elif cmd == "defaults":
            if not idle:
                self._emit(f"err defaults busy ({self.sim_state})")
                return
            self.vals = {p[1]: float(p[2]) for p in MOCK_PARAMS}
            self._emit("ok defaults")
        elif cmd == "set" and len(parts) == 3:
            name, v = parts[1].upper(), parts[2]
            spec = next((p for p in self._visible() if p[1] == name), None)
            if not spec:
                self._emit(f"err {name} unknown setting")
            elif not idle:
                self._emit(f"err {name} busy ({self.sim_state}): change settings while idle")
            else:
                try:
                    f = float(v)
                except ValueError:
                    self._emit(f"err {name} not a number")
                    return
                if not (spec[3] <= f <= spec[4]):
                    self._emit(f"err {name} range {spec[3]:g}..{spec[4]:g}")
                else:
                    self.vals[name] = f
                    self._emit(f"ok {name} {f:g}")
        else:
            self._emit(f"err {cmd} unknown command")

    def _sample(self, t: float) -> tuple[float, float, float]:
        """x (m), continuous theta (rad, pi = hanging), u (V) at t s into the current state."""
        s = self.sim_state
        if s == "SWING":
            amp = math.radians(min(170, 20 + 22 * t))
            return 0.18 * math.sin(5.6 * t), math.pi + amp * math.sin(5.6 * t), 0.75 * math.cos(5.6 * t)
        if s == "BALANCE":
            th = math.radians(3 * math.exp(-t) * math.cos(6 * t) + random.gauss(0, 0.15))
            return 0.12 * math.exp(-t / 3) * math.cos(1.5 * t), th, random.gauss(0, 0.1)
        if s == "STEP":
            u = 0.6 if int(t / 0.6) % 2 == 0 else -0.6
            return 0.09 * math.sin(math.pi * t / 0.6), math.pi + 0.05 * math.sin(5.6 * t), u
        return 0.0, math.pi + 0.1 * math.sin(5.6 * t), 0.0        # CHECK: motor off

    def _begin_run(self, now: float):
        params = " ".join(f"{p[1]}={self.vals[p[1]]:g}" for p in self._visible())
        self._emit(f"run start {self.sim_exp} {params}")
        self.run_t0 = self.next_d = now
        state = MOCK_RUN_STATE[self.sim_exp]
        self._emit(f">> {state}")
        self._goto(state, now)

    def _run(self):
        self._emit("=== ESP32 Single-Board Pendulum Controller v11.1-single (MOCK) ===")
        self._emit('Ready. Waiting for START (button or "home") to home...')
        for cmd in ("status", "exp list", "params"):   # what SerialBridge asks on connect
            self._cmd(cmd)
        last = 0.0
        while True:
            while self.inbox:
                self._cmd(self.inbox.pop(0))
            now = time.time()
            t = now - self.t0
            s = self.sim_state
            if s == "HOMING" and t > 4:
                self._emit("Homing complete!")
                self._goto("HANG", now)
            elif s == "HANG" and t > 2.2:
                self._emit("Hanging reference: 12 (swing 170 counts)")
                self._begin_run(now)
            elif s == "SWING" and t > 7:
                self._emit(">> BALANCE")
                self._goto("BALANCE", now)
            elif s == "STEP" and t > 3.6:
                self._emit("Step test done.")
                self._emit("run end done")
                self._goto("IDLE", now)
            if self.sim_state in ("SWING", "BALANCE", "STEP", "CHECK"):
                while self.next_d <= now:            # 500 Hz, like the firmware
                    x, th, u = self._sample(self.next_d - self.t0)
                    self._emit(f"D {int((self.next_d - self.run_t0) * 1000)} {x:.4f} {th:.4f} {u:.3f} nan")
                    self.next_d += 0.002
            period = 0.02 if self.sim_state == "BALANCE" else 0.1
            if now - last >= period:
                last = now
                t = now - self.t0
                if self.sim_state == "HANG":
                    self._emit(f"HANG  pend {int(85 * math.sin(5.6 * t))}  swing 170")
                elif self.sim_state == "SWING":
                    amp = min(170, 20 + 22 * t)
                    th = ((180 + amp * math.sin(5.6 * t)) + 180) % 360 - 180
                    w = amp / 57.3 * 5.6 * math.cos(5.6 * t)
                    x = 180 * math.sin(5.6 * t)
                    E = -2 + 2 * min(1, t / 7)
                    self._emit(f"SWING th {th:7.1f} deg  w {w:6.2f}  x {x:6.1f} mm  v {0.4 * math.cos(5.6 * t):6.3f}  "
                               f"vr {0.4 * math.copysign(1, math.cos(5.6 * t)):5.2f}  E {E:5.2f}  Eend {E:5.2f}  u {0.3:5.2f}")
                elif self.sim_state == "BALANCE":
                    th = 3 * math.exp(-t) * math.cos(6 * t) + random.gauss(0, 0.15)
                    x = 120 * math.exp(-t / 3) * math.cos(1.5 * t) + random.gauss(0, 1)
                    trim = 0.3 * (1 - math.exp(-max(0, t - 2) / 8))
                    self._emit(f"BAL   th {th:7.1f} deg  w {random.gauss(0, 0.1):6.2f}  x {x:6.1f} mm  v {random.gauss(0, 0.02):6.3f}  "
                               f"vr {0.0:5.2f}  trim {trim:5.2f} deg  u {random.gauss(0, 0.05):5.2f}")
            time.sleep(0.005)


# ----------------------------------------------------------------------------
# Web server
# ----------------------------------------------------------------------------
def _authorised(request: Request) -> bool:
    return not TOKEN or request.query_params.get("token") == TOKEN


def make_app(bridge: Bridge) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_app):
        bridge.loop = asyncio.get_running_loop()
        bridge.start()
        task = asyncio.ensure_future(bridge.watchdog())
        yield
        task.cancel()

    app = FastAPI(lifespan=lifespan)

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        if TOKEN and ws.query_params.get("token") != TOKEN:
            await ws.close(code=4401)
            return
        await ws.accept()
        bridge.clients.add(ws)
        await ws.send_text(json.dumps(bridge.snapshot(ws)))
        await bridge.announce_control()
        try:
            while True:
                try:
                    msg = json.loads(await ws.receive_text())
                except ValueError:
                    continue
                if isinstance(msg, dict):
                    await bridge.command(ws, msg)
        except WebSocketDisconnect:
            pass
        finally:
            await bridge.drop(ws)

    @app.get("/runs")
    async def list_runs(request: Request):
        if not _authorised(request):
            return JSONResponse({"error": "token required"}, status_code=401)
        out = []
        for p in sorted(bridge.runs_dir.glob("*.json"), reverse=True)[:200]:
            try:
                out.append(json.loads(p.read_text(encoding="utf-8")))
            except (OSError, ValueError):
                continue
        return out

    @app.get("/runs/{file}")
    async def get_run(file: str, request: Request):
        if not _authorised(request):
            return JSONResponse({"error": "token required"}, status_code=401)
        if not RUN_FILE.fullmatch(file):
            return JSONResponse({"error": "not found"}, status_code=404)
        path = (bridge.runs_dir / file).resolve()
        if path.parent != bridge.runs_dir.resolve() or not path.is_file():
            return JSONResponse({"error": "not found"}, status_code=404)
        return FileResponse(path, filename=file)

    @app.get("/")
    async def index():
        return FileResponse(FRONTEND / "index.html")

    app.mount("/", StaticFiles(directory=FRONTEND), name="static")
    return app


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the rig's ESP32 (default: auto-detect)")
    ap.add_argument("--mock", action="store_true", help="simulate the rig")
    ap.add_argument("--host", default="127.0.0.1", help="listen address (127.0.0.1 = this PC only)")
    ap.add_argument("--http-port", type=int, default=8000)
    a = ap.parse_args()
    bridge = MockBridge() if a.mock else SerialBridge(a.port)
    print(f"[bridge] {'MOCK rig' if a.mock else 'serial rig'} — open http://{a.host}:{a.http_port}")
    uvicorn.run(make_app(bridge), host=a.host, port=a.http_port, log_level="warning")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the whole bridge test suite**

Run: `python -m pytest bridge/tests -q`
Expected: all tests pass.

- [ ] **Step 5: Smoke-test mock mode**

Run: `python bridge/bridge.py --mock` (leave it running), then in another shell:
`python -c "import urllib.request,json;print(json.loads(urllib.request.urlopen('http://127.0.0.1:8000/runs').read()))"`
Expected: `[]` (no runs yet). Stop the bridge with Ctrl+C.

- [ ] **Step 6: Commit (only if approved)**

```bash
git add bridge/bridge.py bridge/tests/test_bridge.py
git commit -m "bridge: 921600 baud, experiments, run recording and /runs downloads, mock runs"
```

---

### Task 6: Console: experiment picker, motor-command chart, runs panel

**Files:**
- Replace whole content: `pendulum-remote/frontend/app.js`, `pendulum-remote/frontend/index.html`
- Modify: `pendulum-remote/frontend/styles.css`

**Interfaces:**
- Consumes (Task 5): the WebSocket messages and commands listed in Task 5's Interfaces; `GET /runs`; `/runs/<file>`.
- Produces: UI only.

- [ ] **Step 1: Replace `frontend/index.html`**

```html
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Pendulum Lab · Remote Control</title>
  <meta name="color-scheme" content="light dark">
  <!-- Geist + Geist Mono (self-hosted builds via Fontsource), Phosphor icons -->
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/@fontsource/geist-sans@5/latin.css">
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/@fontsource/geist-mono@5/latin.css">
  <script src="https://unpkg.com/@phosphor-icons/web@2.1.1"></script>
  <link rel="stylesheet" href="styles.css">
</head>
<body>
  <header class="topbar">
    <div class="brand">
      <i class="ph ph-wave-sine" aria-hidden="true"></i>
      <span>Pendulum Lab</span>
    </div>
    <div class="topbar-status">
      <span class="conn" id="conn" data-state="connecting">
        <i class="ph ph-plugs" aria-hidden="true"></i><span id="connText">Connecting</span>
      </span>
      <span class="viewers" id="viewers" title="People connected">
        <i class="ph ph-users" aria-hidden="true"></i><span id="viewerCount">0</span>
      </span>
    </div>
    <div class="control-owner">
      <span id="ownerText" class="owner-text">Viewing</span>
      <button class="btn btn-quiet" id="controlBtn" type="button">Take control</button>
    </div>
  </header>

  <main class="layout">
    <!-- Rig: state, live view, start / stop -->
    <section class="panel rig" aria-labelledby="stateLabel">
      <div class="rig-head">
        <div>
          <h1 class="state" id="stateLabel" data-state="OFFLINE">Rig offline</h1>
          <p class="state-help" id="stateHelp">Waiting for the bridge to find the rig.</p>
        </div>
        <div class="rig-actions">
          <button class="btn btn-primary" id="startBtn" type="button" disabled>
            <i class="ph ph-play" aria-hidden="true"></i>Start
          </button>
          <button class="btn btn-stop" id="stopBtn" type="button" disabled>
            <i class="ph ph-stop" aria-hidden="true"></i>Stop
          </button>
        </div>
      </div>

      <div class="viz-wrap">
        <canvas id="viz" aria-label="Live view of the cart and pendulum" role="img"></canvas>
        <div class="viz-empty" id="vizEmpty">
          <i class="ph ph-cell-signal-slash" aria-hidden="true"></i>
          <p>No live data yet. Start a run to see the cart and pendulum move.</p>
        </div>
      </div>

      <dl class="readouts">
        <div><dt>Angle from upright</dt><dd><span id="roAngle">-</span><small>deg</small></dd></div>
        <div><dt>Cart position</dt><dd><span id="roX">-</span><small>mm</small></dd></div>
        <div><dt id="roExtraLabel">Motor command</dt><dd><span id="roExtra">-</span><small id="roExtraUnit">V</small></dd></div>
      </dl>
    </section>

    <!-- Experiment + parameters -->
    <section class="panel params" aria-labelledby="paramsTitle">
      <div class="exp-picker">
        <label for="expSelect">Experiment</label>
        <select id="expSelect" disabled><option>Waiting for the rig</option></select>
        <p class="exp-desc" id="expDesc"></p>
      </div>

      <div class="params-head">
        <h2 id="paramsTitle">Parameters</h2>
        <p class="params-note" id="paramsNote">Take control to change settings. Changes apply while the rig is idle.</p>
      </div>

      <div id="paramsBody" class="params-body" aria-busy="true">
        <div class="skeleton"></div><div class="skeleton"></div><div class="skeleton"></div>
        <div class="skeleton"></div><div class="skeleton short"></div>
      </div>

      <div class="params-actions">
        <button class="btn btn-primary" id="applyBtn" type="button" disabled>Apply changes</button>
        <button class="btn btn-quiet" id="defaultsBtn" type="button" disabled>Restore defaults</button>
      </div>
    </section>

    <!-- Live charts -->
    <section class="panel charts" aria-label="Live charts">
      <div class="chart">
        <div class="chart-head"><h3>Pendulum angle</h3><span class="chart-range">last 20 s, deg from upright</span></div>
        <canvas id="chartAngle"></canvas>
      </div>
      <div class="chart">
        <div class="chart-head"><h3>Cart position</h3><span class="chart-range">last 20 s, mm from centre, dashed = setpoint</span></div>
        <canvas id="chartX"></canvas>
      </div>
      <div class="chart">
        <div class="chart-head"><h3>Motor command</h3><span class="chart-range">last 20 s, volts (2.5 = full power)</span></div>
        <canvas id="chartU"></canvas>
      </div>
    </section>

    <!-- Event log -->
    <section class="panel events" aria-labelledby="eventsTitle">
      <h2 id="eventsTitle">Activity</h2>
      <ol class="event-list" id="eventList" aria-live="polite">
        <li class="event-empty">Nothing yet. Events from the rig show up here.</li>
      </ol>
    </section>

    <!-- Saved runs -->
    <section class="panel runs" aria-labelledby="runsTitle">
      <div class="runs-head">
        <h2 id="runsTitle">Runs</h2>
        <p class="params-note">Every finished run is saved on the bridge PC. In MATLAB, <code>load</code> the .mat file: it holds t, x, theta, u, ref and simout.</p>
      </div>
      <ol class="run-list" id="runList">
        <li class="run-empty">No runs yet.</li>
      </ol>
    </section>
  </main>

  <dialog id="startDialog" class="dialog">
    <form method="dialog">
      <h2>Start a run?</h2>
      <p id="startText">The cart will home to both ends, then run the selected experiment. Make sure the track is clear.</p>
      <div class="dialog-actions">
        <button class="btn btn-quiet" value="cancel" type="submit">Cancel</button>
        <button class="btn btn-primary" value="start" type="submit" id="confirmStart">Start run</button>
      </div>
    </form>
  </dialog>

  <dialog id="defaultsDialog" class="dialog">
    <form method="dialog">
      <h2>Restore defaults?</h2>
      <p>Every parameter goes back to the tested values in the firmware.</p>
      <div class="dialog-actions">
        <button class="btn btn-quiet" value="cancel" type="submit">Cancel</button>
        <button class="btn btn-primary" value="ok" type="submit">Restore</button>
      </div>
    </form>
  </dialog>

  <div class="toasts" id="toasts" aria-live="assertive"></div>

  <script src="app.js"></script>
</body>
</html>
```

- [ ] **Step 2: Update `frontend/styles.css`**

In the `.layout` rule, replace the `grid-template-areas` with:

```css
  grid-template-areas:
    "rig params"
    "charts params"
    "events runs";
```

In the `@media (max-width: 1023px)` block, replace `grid-template-areas: "rig" "params" "charts" "events";` with:

```css
    grid-template-areas: "rig" "params" "runs" "charts" "events";
```

Append to the end of the file:

```css
/* ---------- experiment picker ---------- */
.exp-picker {
  display: grid; gap: 6px;
  margin-bottom: 18px; padding-bottom: 18px;
  border-bottom: 1px solid var(--line);
}
.exp-picker label { font-size: 13px; font-weight: 600; color: var(--text-2); }
.exp-picker select {
  font: inherit; font-size: 15px; font-weight: 600;
  height: 40px; padding: 0 12px;
  border-radius: var(--r-ctl); border: 1px solid var(--line-strong);
  background: var(--surface-2); color: var(--text);
}
.exp-picker select:focus-visible { outline: 2px solid var(--accent); outline-offset: 1px; }
.exp-picker select:disabled { opacity: .65; cursor: not-allowed; }
.exp-desc { font-size: 13px; color: var(--text-3); }

/* ---------- runs ---------- */
.runs { grid-area: runs; }
.runs-head { display: grid; gap: 4px; margin-bottom: 12px; }
.runs-head code { font-family: var(--mono); font-size: 12px; }
.run-list { list-style: none; margin: 0; padding: 0; display: grid; }
.run-list li {
  display: grid; grid-template-columns: 1fr auto; gap: 2px 12px; align-items: center;
  padding: 10px 0; border-top: 1px solid var(--line); font-size: 13px;
}
.run-list li:first-child { border-top: 0; }
.run-title { font-weight: 600; }
.run-meta { grid-column: 1; color: var(--text-3); font-family: var(--mono); font-size: 12px; }
.run-meta[data-tone="bad"] { color: var(--stop); }
.run-files { grid-column: 2; grid-row: 1 / span 2; display: flex; gap: 6px; }
.run-files a {
  font-size: 12px; font-weight: 600; text-decoration: none;
  padding: 4px 10px; border-radius: 999px;
  border: 1px solid var(--line-strong); color: var(--text);
}
.run-files a:hover { border-color: var(--accent); color: var(--accent); }
.run-files a:focus-visible { outline: 2px solid var(--accent); outline-offset: 1px; }
.run-empty { color: var(--text-3); display: block !important; border-top: 0 !important; }
```

- [ ] **Step 3: Replace `frontend/app.js`**

```js
// Pendulum Lab console. Talks to bridge.py over a WebSocket.
"use strict";

// ---------------------------------------------------------------- constants
const TRACK_HALF_MM = 470;          // limit switch to centre (homing: ~12,500 counts)
const SOFT_LIMIT_MM = 0.85 * TRACK_HALF_MM;
const CHART_SECONDS = 20;

const STATES = {
  OFFLINE: ["Rig offline", "Waiting for the bridge to find the rig."],
  IDLE:    ["Ready", "Pick an experiment, then press Start. Every run homes the cart first."],
  HOMING:  ["Homing", "Finding both ends of the track, then centring the cart."],
  HANG:    ["Settling", "Measuring where the pendulum hangs at rest before the run starts."],
  SWING:   ["Swinging up", "Pumping energy into the pendulum to bring it to the top."],
  BALANCE: ["Balancing", "Holding the pendulum upright and the cart near the centre."],
  BRAKE:   ["Braking", "The cart reached the software limit and is stopping."],
  STEP:    ["Step test", "Moving the cart back and forth to test the speed loop."],
  CHECK:   ["Sign check", "Motor off. Move the cart and pendulum by hand and check the readings."],
  DONE:    ["Test finished", "Motor off."],
  DAC:     ["Pi control", "The Raspberry Pi is driving the motor."],
};

// Picker groups, in the order of the Feedback manual. Names must match the
// firmware's experiment names; anything else lands in "Other".
const EXP_GROUPS = [
  { title: "Swing-up & balance", names: ["SwingBal"] },
  { title: "Identification", names: ["FreeSwing", "Friction", "CartIdent", "CraneIdent", "InvPendIdent"] },
  { title: "Control", names: ["CartPID", "SwingUp", "PendStab", "CraneCtl"] },
  { title: "Combined", names: ["SwingHold", "UpDown"] },
  { title: "Rig checks", names: ["SignCheck", "StepTest"] },
];
// [description under the picker, what will move (start dialog)]
const EXP_INFO = {
  SwingBal: ["Swings the pendulum up from hanging and balances it upright.",
             "The cart will home to both ends, then swing the pendulum up and balance it without further input. Make sure the track is clear."],
  SignCheck: ["Motor off after homing. Shows angle and position so the sign conventions can be checked by hand.",
              "The cart will home to both ends, then the motor turns off. Move the cart and pendulum by hand and check the readings."],
  StepTest: ["Moves the cart back and forth at a fixed speed to test the speed loop.",
             "The cart will home to both ends, then move about 18 cm each way, three times, and stop."],
};
const DEFAULT_START_TEXT = "The cart will home to both ends, then run the selected experiment. Make sure the track is clear.";

const GROUPS = [
  { title: "Balance", names: ["BAL_MODE", "BAL_POLE", "BAL_K_TH", "BAL_K_THD", "BAL_K_X", "BAL_K_V",
                              "CATCH_ANGLE_DEG", "CATCH_RATE", "DROP_ANGLE_DEG"] },
  { title: "Swing-up", names: ["SWING_V", "SWING_E_TARGET", "SWING_E_SLOW", "SWING_PHASE_DEG", "K_X_SWING"] },
  { title: "Cart speed loop", names: ["KV_P", "V_FF", "V_MAX"] },
];
const LABELS = {
  BAL_MODE: "Gain mode",
  BAL_POLE: "Stiffness",
  BAL_K_TH: "Angle gain",
  BAL_K_THD: "Angular-rate gain",
  BAL_K_X: "Cart position gain",
  BAL_K_V: "Cart speed gain",
  CATCH_ANGLE_DEG: "Catch angle",
  CATCH_RATE: "Catch speed limit",
  DROP_ANGLE_DEG: "Drop angle",
  SWING_V: "Swing speed",
  SWING_E_TARGET: "Energy target",
  SWING_E_SLOW: "Energy ease-off",
  SWING_PHASE_DEG: "Push timing lead",
  K_X_SWING: "Centring pull",
  KV_P: "Speed loop gain",
  V_FF: "Feed-forward",
  V_MAX: "Top speed",
};
// Help text shown under each field (the board's own descriptions are terser)
const HELP = {
  BAL_POLE: "Higher is stiffer. The four gains are computed from this.",
  BAL_K_TH: "Cart acceleration per radian of lean.",
  BAL_K_THD: "Per rad/s of rotation. Damps the motion.",
  BAL_K_X: "Per metre from centre. Brings the cart home.",
  BAL_K_V: "Per m/s of cart speed.",
  CATCH_ANGLE_DEG: "Switch to balancing inside this angle from upright.",
  CATCH_RATE: "Only catch when the pendulum turns slower than this.",
  DROP_ANGLE_DEG: "Give up and swing up again past this angle.",
  SWING_V: "How fast the cart moves while pumping energy in.",
  SWING_E_TARGET: "0 just reaches upright. Higher arrives faster.",
  SWING_E_SLOW: "Larger values ease off the push earlier near the top.",
  SWING_PHASE_DEG: "0 pushes as the pendulum passes the bottom, 90 at the swing ends.",
  K_X_SWING: "Pulls the cart back toward the centre while swinging.",
  KV_P: "How hard the motor corrects speed errors.",
  V_FF: "Motor command per m/s of target speed.",
  V_MAX: "Speed limit for the cart.",
};
const MANUAL_ONLY = new Set(["BAL_K_TH", "BAL_K_THD", "BAL_K_X", "BAL_K_V"]);
const POLE_ONLY = new Set(["BAL_POLE"]);

// ---------------------------------------------------------------- state
const app = {
  ws: null,
  connected: false,
  role: "free",          // "you" | "other" | "free"
  state: "OFFLINE",
  params: [],            // from the board (shared + the selected experiment's)
  pending: {},           // name -> value typed but not applied
  errors: {},            // name -> message
  experiments: [],       // [{name, title}] from the board
  exp: "",               // selected experiment
  runs: [],              // run summaries, newest first
  samples: [],           // {t, th, x, u, ref} from the 50 Hz D telemetry
  pose: { th: 180, x: 0, has: false },
};

const $ = (id) => document.getElementById(id);

function tokenQuery() {
  const token = new URLSearchParams(location.search).get("token");
  return token ? `?token=${encodeURIComponent(token)}` : "";
}

// ---------------------------------------------------------------- connection
let retryMs = 500;
function connect() {
  const url = `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws` + tokenQuery();
  setConn("connecting", "Connecting");
  const ws = new WebSocket(url);
  app.ws = ws;
  ws.onopen = () => { app.connected = true; retryMs = 500; setConn("ok", "Connected"); loadRuns(); render(); };
  ws.onmessage = (e) => { try { onMessage(JSON.parse(e.data)); } catch (err) { console.error(err); } };
  ws.onclose = (e) => {
    app.connected = false;
    app.role = "free";
    setConn("down", e.code === 4401 ? "Wrong or missing token" : "Disconnected");
    render();
    if (e.code !== 4401) {
      setTimeout(connect, retryMs);
      retryMs = Math.min(retryMs * 2, 8000);
    }
  };
}
function send(msg) {
  if (app.ws && app.ws.readyState === WebSocket.OPEN) app.ws.send(JSON.stringify(msg));
}
function setConn(state, text) {
  $("conn").dataset.state = state;
  $("connText").textContent = text;
}
setInterval(() => { if (app.role === "you") send({ cmd: "ping" }); }, 1000);

// ---------------------------------------------------------------- messages
function onMessage(m) {
  switch (m.kind) {
    case "hello":
      app.role = m.control;
      setState(m.state, true);
      app.experiments = m.experiments || [];
      app.exp = m.exp || "";
      renderPicker();
      if (m.params && m.params.length) setParams(m.params);
      else send({ cmd: "params" });             // bridge hasn't got them yet: ask
      $("viewerCount").textContent = m.clients;
      break;
    case "control":
      app.role = m.control;
      $("viewerCount").textContent = m.clients;
      break;
    case "state":
      setState(m.state);
      break;
    case "params":
      setParams(m.params);
      break;
    case "experiments":
      app.experiments = m.experiments;
      renderPicker();
      break;
    case "exp":
      app.exp = m.name;
      app.pending = {};
      app.errors = {};
      renderPicker();
      break;
    case "run_start":
      app.samples = [];
      addEvent(`Run started: ${titleOf(m.name)}`);
      break;
    case "run_end":
      addRun(m);
      addEvent(`Run ended (${m.reason}): ${titleOf(m.name)}`, ["done", "stop"].includes(m.reason) ? "" : "bad");
      break;
    case "telemetry":
      onTelemetry(m);
      return;                       // high rate: no full re-render
    case "reply":
      onReply(m);
      break;
    case "event":
      addEvent(m.text, toneOf(m.text));
      break;
  }
  render();
}

function setState(s, quiet) {
  if (!STATES[s]) s = s === "STEP TEST" ? "STEP" : "IDLE";
  if (s === app.state) return;
  app.state = s;
  if (!quiet && s !== "OFFLINE") addEvent(STATES[s][0], s === "BALANCE" ? "good" : s === "BRAKE" ? "bad" : "");
  if (s === "HOMING") { app.samples = []; }
}

function setParams(list) {
  app.params = list;
  // Drop pending edits that now match the board
  for (const p of list) {
    if (p.name in app.pending && Math.abs(app.pending[p.name] - p.value) < 1e-9) delete app.pending[p.name];
  }
  renderParams();
}

function onReply(m) {
  const [name, ...rest] = m.text.split(" ");
  const isParam = app.params.some((p) => p.name === name);
  if (isParam) {
    if (m.ok) {
      delete app.errors[name];
    } else {
      app.errors[name] = rest.join(" ").replace(/^range /, "Allowed range ");
    }
    renderParams();
    return;
  }
  if (!m.ok) toast(clean(m.text), "bad");
  else if (name === "defaults") toast("Defaults restored");
}

function setExtra(label, value, unit) {
  $("roExtraLabel").textContent = label;
  $("roExtra").textContent = value;
  $("roExtraUnit").textContent = unit;
}

function onTelemetry(m) {
  const now = performance.now() / 1000;
  if (m.mode === "D") {
    app.pose = { th: m.th_deg, x: m.x_mm, has: true };
    app.samples.push({ t: now, th: m.th_deg, x: m.x_mm, u: m.u, ref: m.ref_mm });
    $("roAngle").textContent = m.th_deg.toFixed(1);
    $("roX").textContent = m.x_mm.toFixed(0);
    if (app.exp !== "SwingBal") setExtra("Motor command", m.u.toFixed(2), "V");
  } else if (m.mode === "SWING") {
    setExtra("Swing energy", m.Eend.toFixed(2), "of 0");
  } else if (m.mode === "BAL") {
    setExtra("Upright trim", m.trim_deg.toFixed(2), "deg");
  } else if (m.mode === "HANG") {
    app.pose = { th: 180, x: 0, has: true };
  }
  const cutoff = now - CHART_SECONDS;
  while (app.samples.length && app.samples[0].t < cutoff) app.samples.shift();
  $("vizEmpty").hidden = app.pose.has;
}

// ---------------------------------------------------------------- render
function canEdit() { return app.connected && app.role === "you" && app.state === "IDLE"; }

function render() {
  const [label, help] = STATES[app.state] || STATES.IDLE;
  const st = $("stateLabel");
  st.textContent = app.connected ? label : "Not connected";
  st.dataset.state = app.connected ? app.state : "OFFLINE";
  $("stateHelp").textContent = !app.connected ? "Trying to reach the bridge on this computer."
    : app.state === "IDLE" && app.role !== "you" ? "Take control to pick an experiment and start a run." : help;

  const online = app.connected && app.state !== "OFFLINE";
  $("startBtn").disabled = !(online && app.role === "you" && app.state === "IDLE" && app.exp);
  $("stopBtn").disabled = !online;          // anyone connected can stop

  const cb = $("controlBtn");
  if (app.role === "you") { cb.textContent = "Release control"; $("ownerText").textContent = "You are in control"; }
  else if (app.role === "other") { cb.textContent = "Take control"; $("ownerText").textContent = "Someone else is in control"; }
  else { cb.textContent = "Take control"; $("ownerText").textContent = "Viewing"; }
  cb.disabled = !app.connected || app.role === "other";

  $("expSelect").disabled = !canEdit() || !app.experiments.length;
  $("paramsNote").textContent =
    app.role !== "you" ? "Take control to change settings. Changes apply while the rig is idle."
    : app.state !== "IDLE" ? "Settings are locked while the rig is running. Stop it to make changes."
    : "Edit values, then apply. They take effect on the next run.";
  $("applyBtn").disabled = !canEdit() || Object.keys(app.pending).length === 0;
  $("defaultsBtn").disabled = !canEdit();
  document.querySelectorAll("#paramsBody input, #paramsBody .segmented button")
    .forEach((el) => { el.disabled = !canEdit(); });
}

function titleOf(name) {
  const e = app.experiments.find((x) => x.name === name);
  return e ? e.title : name;
}

function renderPicker() {
  const sel = $("expSelect");
  sel.innerHTML = "";
  if (!app.experiments.length) {
    const o = document.createElement("option");
    o.textContent = "Waiting for the rig";
    sel.appendChild(o);
    $("expDesc").textContent = "";
    return;
  }
  const placed = new Set();
  const addGroup = (title, list) => {
    if (!list.length) return;
    const og = document.createElement("optgroup");
    og.label = title;
    for (const e of list) {
      const o = document.createElement("option");
      o.value = e.name;
      o.textContent = e.title;
      og.appendChild(o);
      placed.add(e.name);
    }
    sel.appendChild(og);
  };
  for (const g of EXP_GROUPS) addGroup(g.title, app.experiments.filter((e) => g.names.includes(e.name)));
  addGroup("Other", app.experiments.filter((e) => !placed.has(e.name)));
  sel.value = app.exp;
  $("expDesc").textContent = (EXP_INFO[app.exp] || [""])[0];
}

function valueOf(name) {
  if (name in app.pending) return app.pending[name];
  const p = app.params.find((q) => q.name === name);
  return p ? p.value : undefined;
}

function fmt(v) {
  if (v === undefined || v === null || Number.isNaN(v)) return "";
  return String(Math.round(v * 1000) / 1000);
}

function renderParams() {
  const body = $("paramsBody");
  if (!app.params.length) return;               // keep the skeleton until data arrives
  body.removeAttribute("aria-busy");
  const manual = valueOf("BAL_MODE") >= 0.5;
  const byName = Object.fromEntries(app.params.map((p) => [p.name, p]));
  const known = new Set(GROUPS.flatMap((g) => g.names));
  // Settings the console has no group for (new experiments) come first
  const groups = [{ title: "Experiment settings", names: app.params.map((p) => p.name).filter((n) => !known.has(n)) },
                  ...GROUPS];
  const focusedId = document.activeElement && document.activeElement.id;
  body.innerHTML = "";

  for (const g of groups) {
    const items = g.names.map((n) => byName[n]).filter((p) => p
      && !(manual && POLE_ONLY.has(p.name)) && !(!manual && MANUAL_ONLY.has(p.name)));
    if (!items.length) continue;
    const sec = document.createElement("div");
    sec.className = "group";
    sec.innerHTML = `<h3>${g.title}</h3><div class="group-fields"></div>`;
    const fields = sec.querySelector(".group-fields");
    for (const p of items) fields.appendChild(p.name === "BAL_MODE" ? modeField(p) : numberField(p));
    body.appendChild(sec);
  }
  if (focusedId) { const el = $(focusedId); if (el) el.focus(); }
  render();
}

function modeField(p) {
  const wrap = document.createElement("div");
  wrap.className = "field";
  const manual = valueOf(p.name) >= 0.5;
  wrap.innerHTML = `
    <label>${LABELS[p.name]}</label>
    <div class="segmented" role="group" aria-label="${LABELS[p.name]}">
      <button type="button" data-v="0" aria-pressed="${!manual}">Pole placement</button>
      <button type="button" data-v="1" aria-pressed="${manual}">Manual gains</button>
    </div>
    <span class="help">${manual ? "Type the four feedback gains directly." : "All four gains are computed from the stiffness."}</span>`;
  wrap.querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
    setPending(p, Number(b.dataset.v));
    renderParams();
  }));
  if (p.name in app.pending) wrap.classList.add("dirty");
  return wrap;
}

function numberField(p) {
  const wrap = document.createElement("div");
  wrap.className = "field" + (p.name in app.pending ? " dirty" : "");
  const id = `f_${p.name}`;
  wrap.innerHTML = `
    <label for="${id}">${LABELS[p.name] || p.name}<span class="range">${fmt(p.min)} to ${fmt(p.max)}</span></label>
    <div class="input-row">
      <input type="number" id="${id}" step="any" inputmode="decimal" value="${fmt(valueOf(p.name))}"
             min="${p.min}" max="${p.max}" aria-describedby="${id}_h ${id}_e">
      <span class="unit">${p.unit || ""}</span>
    </div>
    <span class="help" id="${id}_h">${HELP[p.name] || capitalise(p.desc)}${Math.abs(p.value - p.default) > 1e-9 ? ` (default ${fmt(p.default)})` : ""}</span>
    <span class="error" id="${id}_e">${app.errors[p.name] || ""}</span>`;
  const input = wrap.querySelector("input");
  input.addEventListener("input", () => {
    const v = input.value === "" ? NaN : Number(input.value);
    const err = wrap.querySelector(".error");
    if (Number.isNaN(v)) { err.textContent = "Enter a number"; return; }
    if (v < p.min || v > p.max) { err.textContent = `Allowed range ${fmt(p.min)} to ${fmt(p.max)}`; }
    else { err.textContent = ""; delete app.errors[p.name]; }
    setPending(p, v);
    wrap.classList.toggle("dirty", p.name in app.pending);
    render();
  });
  return wrap;
}

function setPending(p, v) {
  if (Math.abs(v - p.value) < 1e-9) delete app.pending[p.name];
  else app.pending[p.name] = v;
}

function capitalise(s) { return s ? s.charAt(0).toUpperCase() + s.slice(1) : ""; }
function clean(s) { return s.replace(/[—–]/g, "-"); }   // no em/en dashes in the UI

// ---------------------------------------------------------------- runs
async function loadRuns() {
  try {
    const res = await fetch("/runs" + tokenQuery());
    if (!res.ok) return;
    app.runs = await res.json();
    renderRuns();
  } catch (e) {
    // Bridge not reachable yet: the next WebSocket reconnect loads them again
  }
}

function addRun(r) {
  app.runs.unshift(r);
  renderRuns();
}

function fileUrl(f) { return `/runs/${encodeURIComponent(f)}` + tokenQuery(); }

function renderRuns() {
  const list = $("runList");
  list.innerHTML = "";
  if (!app.runs.length) {
    list.innerHTML = '<li class="run-empty">No runs yet.</li>';
    return;
  }
  for (const r of app.runs.slice(0, 50)) {
    const li = document.createElement("li");
    li.innerHTML = '<span class="run-title"></span><span class="run-files"></span><span class="run-meta"></span>';
    li.querySelector(".run-title").textContent = titleOf(r.name);
    const when = new Date(r.started * 1000).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit", second: "2-digit" });
    const results = Object.entries(r.results || {}).map(([k, v]) => `${k} ${fmt(v)}`).join(", ");
    const meta = li.querySelector(".run-meta");
    meta.textContent = `${when} · ${r.duration.toFixed(1)} s · ${r.reason}${results ? " · " + results : ""}`;
    if (!["done", "stop"].includes(r.reason)) meta.dataset.tone = "bad";
    const files = li.querySelector(".run-files");
    for (const [kind, f] of Object.entries(r.files || {})) {
      const a = document.createElement("a");
      a.href = fileUrl(f);
      a.download = f;
      a.textContent = kind === "mat" ? ".mat" : "CSV";
      a.setAttribute("aria-label", `Download ${titleOf(r.name)} run as ${a.textContent}`);
      files.appendChild(a);
    }
    list.appendChild(li);
  }
}

// ---------------------------------------------------------------- actions
$("controlBtn").addEventListener("click", () => send({ cmd: app.role === "you" ? "release" : "claim" }));
$("startBtn").addEventListener("click", () => {
  $("startText").textContent = (EXP_INFO[app.exp] || [])[1] || DEFAULT_START_TEXT;
  $("startDialog").showModal();
});
$("startDialog").addEventListener("close", () => {
  if ($("startDialog").returnValue === "start") send({ cmd: "start" });
});
$("stopBtn").addEventListener("click", () => send({ cmd: "stop" }));
$("expSelect").addEventListener("change", (e) => {
  const name = e.target.value;
  e.target.value = app.exp;                     // stay on the board's choice until it confirms
  if (name && name !== app.exp) send({ cmd: "select", name });
});
$("defaultsBtn").addEventListener("click", () => $("defaultsDialog").showModal());
$("defaultsDialog").addEventListener("close", () => {
  if ($("defaultsDialog").returnValue === "ok") { app.pending = {}; app.errors = {}; send({ cmd: "defaults" }); }
});
$("applyBtn").addEventListener("click", () => {
  const entries = Object.entries(app.pending);
  let blocked = false;
  for (const [name, v] of entries) {
    const p = app.params.find((q) => q.name === name);
    if (!p || Number.isNaN(v) || v < p.min || v > p.max) { blocked = true; continue; }
    send({ cmd: "set", name, value: v });       // the board re-checks every limit
  }
  if (blocked) toast("Some values are out of range. Fix the highlighted fields.", "bad");
});
document.addEventListener("keydown", (e) => {   // Esc = stop, anywhere on the page
  if (e.key === "Escape" && !$("stopBtn").disabled && !document.querySelector("dialog[open]")) send({ cmd: "stop" });
});

// ---------------------------------------------------------------- events & toasts
function toneOf(t) {
  return /SOFT LIMIT|OVERSPEED|LIMIT HIT|ERROR|STOP|heartbeat|aborted/i.test(t) ? "bad" : "";
}
function addEvent(text, tone = "") {
  const list = $("eventList");
  const empty = list.querySelector(".event-empty");
  if (empty) empty.remove();
  const li = document.createElement("li");
  if (tone) li.dataset.tone = tone;
  const time = new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
  li.innerHTML = `<time>${time}</time><span></span>`;
  li.querySelector("span").textContent = clean(text);
  list.prepend(li);
  while (list.children.length > 60) list.lastChild.remove();
}
function toast(text, tone = "") {
  const el = document.createElement("div");
  el.className = "toast";
  if (tone) el.dataset.tone = tone;
  el.textContent = clean(text);
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), 4000);
}

// ---------------------------------------------------------------- drawing
let colors = {};
function readColors() {
  const cs = getComputedStyle(document.documentElement);
  for (const k of ["text", "text-2", "text-3", "line", "line-strong", "accent", "stop", "surface", "surface-2"]) {
    colors[k] = cs.getPropertyValue(`--${k}`).trim();
  }
}
readColors();
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", readColors);

function fitCanvas(c) {
  const r = c.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: r.width, h: r.height };
}

const smooth = { th: 180, x: 0 };
function drawViz() {
  const { ctx, w, h } = fitCanvas($("viz"));
  ctx.clearRect(0, 0, w, h);
  const margin = 28;
  const railY = h * 0.68;
  const sx = (mm) => w / 2 + (mm / TRACK_HALF_MM) * (w / 2 - margin);

  // ease toward the latest sample (purely visual, data is ~50 Hz)
  let dth = ((app.pose.th - smooth.th + 540) % 360) - 180;
  smooth.th += dth * 0.5;
  smooth.x += (app.pose.x - smooth.x) * 0.5;

  // rail, limit stops, soft-limit marks
  ctx.strokeStyle = colors["line-strong"]; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(sx(-TRACK_HALF_MM), railY); ctx.lineTo(sx(TRACK_HALF_MM), railY); ctx.stroke();
  ctx.fillStyle = colors["text-3"];
  for (const s of [-1, 1]) ctx.fillRect(sx(s * TRACK_HALF_MM) - 2, railY - 10, 4, 20);
  ctx.setLineDash([3, 4]); ctx.strokeStyle = colors["stop"]; ctx.globalAlpha = 0.5; ctx.lineWidth = 1;
  for (const s of [-1, 1]) { ctx.beginPath(); ctx.moveTo(sx(s * SOFT_LIMIT_MM), railY - 14); ctx.lineTo(sx(s * SOFT_LIMIT_MM), railY + 14); ctx.stroke(); }
  ctx.setLineDash([]); ctx.globalAlpha = 1;
  ctx.strokeStyle = colors["line"];
  ctx.beginPath(); ctx.moveTo(sx(0), railY + 8); ctx.lineTo(sx(0), railY + 16); ctx.stroke();

  // cart
  const cx = sx(smooth.x), cw = Math.max(44, w * 0.07), ch = 18;
  ctx.fillStyle = colors["text"];
  roundRect(ctx, cx - cw / 2, railY - ch / 2, cw, ch, 5); ctx.fill();

  // pendulum: theta 0 = up, positive = top leans right (+x)
  const L = Math.min(h * 0.55, w * 0.22);
  const rad = smooth.th * Math.PI / 180;
  const px = cx, py = railY - ch / 2;
  const bx = px + L * Math.sin(rad), by = py - L * Math.cos(rad);
  ctx.strokeStyle = app.state === "BALANCE" ? colors["accent"] : colors["text-2"];
  ctx.lineWidth = 5; ctx.lineCap = "round";
  ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(bx, by); ctx.stroke();
  ctx.fillStyle = ctx.strokeStyle;
  ctx.beginPath(); ctx.arc(bx, by, 8, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = colors["surface"];
  ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
}
function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}

function niceScale(maxAbs, steps) { return steps.find((s) => maxAbs <= s) || steps[steps.length - 1]; }

// Plots app.samples[key] over the last CHART_SECONDS; refKey (optional) is drawn dashed
function drawChart(canvas, key, steps, color, refKey) {
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  const left = 40, right = 6, top = 6, bottom = 6;
  const pw = w - left - right, ph = h - top - bottom;
  const data = app.samples;
  const val = (s, k) => (typeof s[k] === "number" ? s[k] : NaN);
  let maxAbs = 0;
  for (const s of data) {
    for (const k of refKey ? [key, refKey] : [key]) {
      const v = val(s, k);
      if (!Number.isNaN(v)) maxAbs = Math.max(maxAbs, Math.abs(v));
    }
  }
  const range = niceScale(maxAbs, steps);
  const y = (v) => top + ph / 2 - (v / range) * (ph / 2);

  ctx.font = "11px " + getComputedStyle(document.body).getPropertyValue("--mono");
  ctx.fillStyle = colors["text-3"]; ctx.textAlign = "right"; ctx.textBaseline = "middle";
  ctx.strokeStyle = colors["line"]; ctx.lineWidth = 1;
  for (const v of [range, 0, -range]) {
    ctx.beginPath(); ctx.moveTo(left, y(v)); ctx.lineTo(w - right, y(v)); ctx.stroke();
    ctx.fillText(String(v), left - 6, y(v));
  }
  if (data.length < 2) {
    ctx.textAlign = "center"; ctx.fillText("Waiting for data", left + pw / 2, top + ph / 2 - 12);
    return;
  }
  const tEnd = performance.now() / 1000;
  const xAt = (t) => left + pw * (1 - (tEnd - t) / CHART_SECONDS);
  const line = (k, stroke, dash) => {
    ctx.strokeStyle = stroke; ctx.lineWidth = 1.75; ctx.lineJoin = "round"; ctx.setLineDash(dash);
    ctx.beginPath();
    let pen = false;
    data.forEach((s, i) => {
      const v = val(s, k);
      if (Number.isNaN(v)) { pen = false; return; }
      const vx = xAt(s.t), vy = y(Math.max(-range, Math.min(range, v)));
      const jump = k === "th" && i > 0 && Math.abs(v - val(data[i - 1], k)) > 180;   // wrap at +-180
      if (!pen || jump) ctx.moveTo(vx, vy); else ctx.lineTo(vx, vy);
      pen = true;
    });
    ctx.stroke();
    ctx.setLineDash([]);
  };
  line(key, color, []);
  if (refKey) line(refKey, colors["text-3"], [4, 4]);
}

function frame() {
  drawViz();
  drawChart($("chartAngle"), "th", [10, 30, 90, 180], colors["accent"]);
  drawChart($("chartX"), "x", [50, 100, 250, 500], colors["text"], "ref");
  drawChart($("chartU"), "u", [0.5, 1, 2.5], colors["text-2"]);
  requestAnimationFrame(frame);
}

// ---------------------------------------------------------------- go
render();
renderPicker();
connect();
requestAnimationFrame(frame);
```

- [ ] **Step 4: Check the console against the mock**

Run: `cd /c/Users/super/Documents/ED2/pendulum-remote && python bridge/bridge.py --mock`, then open http://localhost:8000. Expected:
1. The picker shows "Swing-up & balance" (group "Swing-up & balance"), plus "Sign check (motor off)" and "Speed loop step test" (group "Rig checks"). It is disabled until **Take control**.
2. After taking control, selecting "Speed loop step test" changes the parameters panel to only the three "Cart speed loop" fields. Selecting "Swing-up & balance" brings the Balance and Swing-up groups back.
3. **Start** shows the start dialog with the experiment's own text. After about 6 s a run starts:
   - the angle, cart and motor-command charts move;
   - the motor-command readout shows volts for the step test;
   - "Swing energy" / "Upright trim" show for swing-up & balance.
4. **Stop** (or the step test finishing) adds a row to **Runs** with time, duration, reason, and **CSV** / **.mat** buttons. Both download. The CSV opens with the `# experiment ...` header and the `t,x,theta,u,ref` columns.
5. Reloading the page keeps the runs list (loaded from `/runs`).
6. At 375 px width (DevTools device mode) the layout stacks rig → params → runs → charts → events, with no horizontal scroll.

- [ ] **Step 5: Commit (only if approved)**

```bash
git add frontend/app.js frontend/index.html frontend/styles.css
git commit -m "console: experiment picker, motor command chart, runs panel with CSV/.mat downloads"
```

---

### Task 7: End-to-end on the rig, and docs

**Files:**
- Modify: `pendulum-remote/README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: the stage 0–1 "done when" from the spec: *a swing-up & balance run downloads and loads in MATLAB*.

- [ ] **Step 1: Update `pendulum-remote/README.md`**

Replace the `## Requirements` list's first bullet with:

```markdown
- Rig ESP32 flashed with **cart_single v11.1** or later (`ED2-Repo/cart/cart_single`):
  experiments, 921600 baud, run data lines.
```

Replace the bullets under `## What the page does` with:

```markdown
- **Experiment** picker: choose what the next run does (swing-up & balance,
  rig checks; the Feedback lab experiments arrive in later stages).
- **Start** homes the cart, measures the hanging pendulum, then runs the experiment.
- **Stop** (or the **Esc** key) stops the rig. Anyone connected can press it, even without control.
- **Parameters** for the selected experiment can be changed while the rig is idle.
  The ESP32 enforces every limit itself, so the page can't push it outside a safe range even if bypassed.
- Live view of the cart and pendulum, and angle, cart-position and motor-command charts.
- **Runs**: every finished run is saved in `runs/` on this PC as CSV and MATLAB
  `.mat` (`t x theta u ref`, plus `simout = [x theta u t ref]`, `params`, `results`).
  Download them from the page.
```

Replace the whole `## Serial protocol (ESP32 #2, v10.1)` section with:

```markdown
## Serial protocol (cart_single v11.1, 921600 baud)

One command per line. Replies start with `ok `, `err `, `param `, `params end`,
`status `, `exp ` or `io `.

| Command | Effect |
|---|---|
| `exp list` | `exp NAME \| title` per experiment, then `exp end` |
| `exp select NAME` | Experiment for the next run (idle only) |
| `run` (or `home`) | Home, measure the hanging reference, run the selected experiment |
| `stop` | Stop now, also during homing |
| `status` | `status STATE EXPERIMENT` |
| `params` | Shared + the selected experiment's settings: `param NAME VALUE MIN MAX DEFAULT UNIT \| description`, then `params end` |
| `set NAME VALUE` | Change a setting (idle only, within limits) |
| `defaults` | Restore the firmware defaults |

During a run the board prints `run start NAME K=V ...`, then one
`D <t_ms> <x_m> <theta_rad> <u_V> <ref>` line per 2 ms step (theta continuous:
0 = upright, π = hanging; `ref` is `nan` without a setpoint), then
`run end REASON` (`done`, `stop`, `soft-limit`, `overspeed`, `limit-switch`).
Human-readable `SWING` / `BAL` / `HANG` lines and `>> STATE` lines are
documented in `ED2-Repo/docs/esp32.md`.

## Tests

    python -m pip install -r bridge/requirements-dev.txt
    python -m pytest bridge/tests -q
```

Replace the `## Files` bullets with:

```markdown
- `bridge/bridge.py`: serial ⇄ WebSocket bridge, control lock, heartbeat watchdog, run downloads, mock rig
- `bridge/runs.py`: turns `run start` / `D` / `run end` lines into saved runs (CSV, .mat, .json)
- `bridge/matfile.py`: small MATLAB Level 5 .mat writer (no SciPy needed to run the bridge)
- `bridge/tests/`: pytest suite (needs `requirements-dev.txt`)
- `frontend/`: plain HTML/CSS/JS console (no build step)
- `runs/`: saved runs (gitignored)
```

- [ ] **Step 2: Rig run through the console (user; Claude checks the files)**

Ask the user to:
1. Close the Arduino Serial Monitor and the logger, and start `python bridge/bridge.py`.
2. Open http://localhost:8000, take control, keep **Swing-up & balance**, press Start, let it balance for about 20 s, then press Stop.
3. Select **Speed loop step test** and run it to the end.

Then check the files in `pendulum-remote/runs/`:

Run: `cd /c/Users/super/Documents/ED2/pendulum-remote && ls -t runs | head -6 && python -c "import scipy.io,glob;f=sorted(glob.glob('runs/*SwingBal*.mat'))[-1];m=scipy.io.loadmat(f);print(f,m['t'].shape,m['simout'].shape,m['experiment'][0],m['reason'][0],float(m['t'][-1,0]))"`

Expected:
- A `SwingBal` CSV/.mat/.json triple and a `StepTest` triple.
- The `.mat` loads with `t` and `simout` row counts equal and about 500 × the run length in seconds.
- `experiment` = `SwingBal`, `reason` = `stop`.
- In the CSV, `theta` starts near 3.14 and ends near 0.0 (balanced upright), and `u` stays within ±2.5.

- [ ] **Step 3: MATLAB check (user)**

Ask the user to run in MATLAB, on the downloaded SwingBal `.mat`:

```matlab
load('<file>.mat');
plot(t, theta, t, u); legend('theta (rad)', 'u (V)'); xlabel('t (s)');
```

Expected: theta falls from about π to 0 during the swing-up, then stays near 0 while balancing; u is bounded by ±2.5.

- [ ] **Step 4: Commit (only if approved)**

```bash
git add README.md
git commit -m "README: experiments, run downloads, v11.1 serial protocol, tests"
```

---

## Self-review notes (for the reviewer)

**Spec coverage (stages 0–1):**

| Spec item | Task |
|---|---|
| Tab split | 1 |
| Experiment framework, `exp`/`run`/`status` protocol, `D`/`run start`/`run end` lines, continuous theta, `u` in volts, safety outside experiments, auto re-home limited to SwingBal, `PEND_TRIM_DEG`, 921600 baud | 2 |
| Recorder, CSV | 3 |
| `.mat` | 4 |
| Bridge parsing, decimation, `/runs`, mock, control lock unchanged | 5 |
| Picker, parameters from `params`, `u` chart + `ref` overlay, start dialog text, runs panel | 6 |
| README; MATLAB load check (the stage 1 "done when") | 7 |

**Deferred to the stage 2+ plans, by design:**
- the friction offsets in flash;
- every experiment beyond SwingBal/SignCheck/StepTest;
- the `result` lines' producers (the bridge already handles them);
- the `r` column producer (experiment 5);
- the student guides and the `docs/esp32.md` single-board section (stage 8).
