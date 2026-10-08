// =============================================================================
// Single-board cart + pendulum controller (one ESP32)
// Version: 11.0
// Description: One ESP32 reads both quadrature encoders (PCNT hardware),
//              drives the BTS7960 IBT-2 H-bridge, homes the cart, then swings
//              the pendulum up and balances it. Replaces the two-board setup
//              (cart/esp32_encoder_v9 + cart/cart_control), so there is no
//              UART link between boards and no Pi / DAC / ADS1115 path.
//              Control and telemetry over USB serial (115200), same protocol
//              as cart_control v10.1, so pendulum-remote and serial_log.py work.
//
// Changes from cart_control v10.1:
//   - Encoders counted here (code from esp32_encoder_v9 v9.2): 32-bit counts,
//     sampled every 2 ms (500 Hz control step, as before).
//   - Homing zeroes the cart count in software at centre (was the AA 55 A5 5A
//     message to ESP32 #1).
//   - Removed: DAC-follow mode, ADS1115, Pi GPIO start/stop/homing-done lines.
//   - Status LED (GPIO 2) on while homed and running; a limit-switch hit now
//     turns it off too (E2 in the handoff report).
//   - The learned upright trim is kept across re-homes (reset only at boot):
//     on 2026-10-02 it settled at 5.5-6.4 deg every run, and starting from 0
//     after an automatic re-home parked the cart at -384 mm (soft limit).
//   - New serial command "io": counts and switch states, for bring-up.
//
// Pin Assignments:
//   GPIO 39 : Cart encoder A       (input-only; 4.7k pull-up to 5V + 10k/22k divider)
//   GPIO 36 : Cart encoder B       (same)
//   GPIO 35 : Pendulum encoder A   (same)
//   GPIO 34 : Pendulum encoder B   (same)
//   GPIO 25 : RPWM  (IBT-2)
//   GPIO 26 : LPWM  (IBT-2)
//   GPIO 27 : START button         (normally open to GND, pull-up, active LOW)
//   GPIO 14 : STOP button          (normally CLOSED to GND, open = stop)
//   GPIO 32 : Left limit switch    (to GND, pull-up, active LOW)
//   GPIO 33 : Right limit switch   (to GND, pull-up, active LOW)
//   GPIO  2 : Status LED           (on-board LED on most DevKits)
//   Free for a later Pi link: 4, 13, 16, 17, 18, 19, 21, 22, 23
// =============================================================================

#include "driver/pulse_cnt.h"
#include <WiFi.h>

#define FW_VERSION "11.0-single"

// --- Encoder pins (input-only GPIOs, no internal pull-ups) ---
#define CART_ENC_A    39
#define CART_ENC_B    36
#define PEND_ENC_A    35
#define PEND_ENC_B    34

// --- Motor pins ---
#define RPWM_PIN      25
#define LPWM_PIN      26

// --- Button pins ---
#define START_BTN     27
#define STOP_BTN      14

// --- Limit switch pins ---
#define LEFT_LIMIT    32
#define RIGHT_LIMIT   33

// --- Status LED: on while homed and running ---
#define STATUS_LED     2

// --- Encoder sampling: one control step per sample (500 Hz) ---
#define SAMPLE_US     2000

pcnt_unit_handle_t cart_pcnt = NULL;
pcnt_unit_handle_t pend_pcnt = NULL;

// --- Control constants ---
// Homing speed per direction. On the rig, right at 45 crawled ~50 counts per
// breakaway kick, and left at 45 still stalled every ~200–1700 counts.
// Tune each: lowest value that moves steadily with no "Stall" lines in the log.
#define HOMING_PWM_LEFT   50
#define HOMING_PWM_RIGHT  60
// Slowest centering speed near the target (left; right is offset by the
// right/left homing difference). Must stay above the stall threshold — the cart
// stalls moving left at PWM 33 and below, so 20 made it crawl on kicks.
#define CENTER_MIN_PWM    40
#define CENTER_TOLERANCE  20     // counts — within this = at center (~1.5mm)
// Start slowing down this far from center. At homing speed the cart moves
// ~7000 counts/s; with a 500-count ramp it reached center at full speed and
// coasted ~1400 counts past it after the motor stopped.
#define CENTER_SLOW_ZONE  3000
#define CENTER_ACCEPT     50     // settled within this of center = done (~4mm)
#define CENTER_ATTEMPTS   3      // approach + settle attempts before accepting
#define MIN_TRACK_COUNTS  100    // sanity check — reject if track shorter than this

// --- Stall handling during homing ---
// If the encoder moves less than STALL_COUNTS in STALL_MS, the cart is stuck:
// drive at BREAKAWAY_PWM until it moves KICK_MOVE_COUNTS (or BREAKAWAY_MS
// passes), then drop back to the normal homing speed. Tune BREAKAWAY_PWM on the
// rig: just above the lowest PWM that starts the cart moving right from rest.
#define BREAKAWAY_PWM     70
#define BREAKAWAY_MS      300
#define KICK_MOVE_COUNTS  5
#define STALL_COUNTS      5
#define STALL_MS          400
#define MAX_KICKS         5      // consecutive kicks without progress before giving up
#define KICK_PROGRESS_COUNTS 200 // travel after a kick needed to count as "moving again"
#define PHASE_TIMEOUT_MS  30000  // max time for any single homing phase
#define LIMIT_RELEASE_MS  50     // switch must read released this long (debounce)
#define LIMIT_CONFIRM_MS  5      // run mode: switch must read pressed this long

// --- Settle after centering (before zeroing the cart count) ---
#define SETTLE_COUNTS     5      // moving less than this counts as stopped
#define SETTLE_MS         200    // must stay stopped this long
#define SETTLE_TIMEOUT_MS 2000

// =============================================================================
// Swing-up + balance
// =============================================================================
// 1 = motor stays OFF after homing; prints angle and position so the signs
// can be checked by hand. Set to 0 only after the check passes (docs/esp32.md).
// Passed on the single-board build 2026-10-07 (PEND_SIGN -1 still correct).
#define SIGN_CHECK         0

// --- Geometry / scaling ---
// Pendulum counts per revolution. Measured with the logger (cart/serial_log.py),
// turning to a mark: 4 turns back = 1967/turn, 5 turns forward = 1969/turn,
// back through the start mark within 9 counts. 2000 (the 500-line guess) put
// upright 2.9 deg off and added 5.9 deg per full rotation during swing-up —
// the balanced cart ran ~270 mm to one side into the soft limit.
#define PEND_COUNTS_PER_REV 1968
// +1 or -1: chosen so theta is POSITIVE when the top of the upright pendulum
// leans toward the RIGHT limit switch. Checked with SIGN_CHECK.
// -1 on this rig. Confirmed from swing-up logs, not by eye: when the cart
// accelerates, a hanging bob must swing the opposite way, and with +1 it
// read as swinging the same way every time (and in the first CHECK log).
#define PEND_SIGN           -1
// Upright offset (degrees). If the balanced cart creeps steadily one way,
// nudge this by 0.2-0.5 deg until it holds still.
#define PEND_TRIM_DEG       0.0f
#define CART_M_PER_COUNT    (0.156f / 2048.0f)   // Feedback model "Counts->Meters"
// Small-swing natural frequency, rad/s = 2*pi / period. Time 10 small swings.
// Measured: 10 swings in 11.20 s -> period 1.12 s.
#define PEND_OMEGA0         5.61f
#define VEL_FILTER_S        0.008f   // low-pass on velocity estimates

// --- Output mapping: u in [-1, 1] -> PWM ---
// Starts at RUN_MIN_PWM (just under the stall PWM) so small u still moves the
// cart — replaces the Feedback model's friction compensation.
#define RUN_MIN_PWM         30
#define RUN_RIGHT_EXTRA_PWM 10       // right needs more drive (homing 60 vs 50)
#define U_EPS               0.02f

// --- Balance (on the speed loop) ---
// Commands a cart acceleration a = KA_TH*th + KA_THD*th' + KA_X*x + KA_V*v
// (m/s^2), integrated into the speed-loop command. Pendulum model, exact for
// any mass distribution: th'' = w0^2*th - (w0^2/g)*a. The four gains put all
// closed-loop poles at s = -BAL_POLE and are computed from it and PEND_OMEGA0
// (see balanceGains()). All positive with the sign conventions: theta>0 = top
// leans right, x>0 = cart right of centre, a>0 = accelerate right.
// The Feedback model's PID gains (P7 D4 / P25 D1.5) do NOT transfer: its cart
// signal is scaled by a negative factor and it drove a force-like amplifier.
// On this rig they caught the pendulum and then ran the cart away from it.
// Higher BAL_POLE = stiffer, needs more motor; 5 rad/s settles in ~0.2 s.
float BAL_POLE        = 5.0f;     // rad/s   [online]
// Manual gains (BAL_MODE 1) instead of pole placement, so a user can type the
// four gains in directly. Defaults are what BAL_POLE 5 computes. Bad gains just
// make the pendulum fall; the soft limit and re-home handle that.
float BAL_MODE        = 0.0f;     // 0 = gains from BAL_POLE, 1 = manual   [online]
float BAL_K_TH        = 62.8f;    // m/s^2 per rad     [online]
float BAL_K_THD       = 11.2f;    // m/s^2 per rad/s   [online]
float BAL_K_X         = 19.9f;    // m/s^2 per m       [online]
float BAL_K_V         = 15.9f;    // m/s^2 per m/s     [online]
// Self-trim of the upright angle while balancing. An angle offset d makes the
// balanced cart park at x = -d * KA_TH / KA_X (~3.2 m per rad): first
// successful balance held 18 s parked at -302 mm reading +5.5 deg. So adjust
// the trim at -BAL_TRIM_RATE * x (rad/s per m) until the cart parks at centre.
// Simulated (the model reproduced that run: parks at -303 mm): the cart first
// moves the wrong way when the trim changes, so a fast trim fights the balance
// — 0.05+ from the catch went unstable. Starting after BAL_TRIM_DELAY_MS, 0.04
// brings a 5.5 deg offset to within ~6 mm of centre in ~30 s.
#define BAL_TRIM_RATE       0.04f
#define BAL_TRIM_DELAY_MS   2000     // let the catch settle first
#define BAL_TRIM_MAX        0.17f    // rad (~10 deg)
float CATCH_ANGLE_DEG = 14.3f;    // deg — switch to balance inside this   [online]
// rad/s — too fast to catch above this. Stopping rotation w takes a cart speed
// change of ~w * g/w0^2 = 0.31*w m/s: a catch at 2.37 rad/s saturated the
// speed command (0.8 m/s) and fell. Faster passes just keep swinging.
float CATCH_RATE      = 2.0f;     // [online]
float DROP_ANGLE_DEG  = 34.4f;    // deg — lost it, back to swing-up   [online]

// --- Cart speed loop (swing-up and step test) ---
// u = V_FF*vRef + KV_P*(vRef - v), mapped to RUN_MIN_PWM..SPEED_MAX_PWM.
// Commanding PWM directly, the cart took ~half a swing to reverse, and every
// swing-up rule built on that fought the lag (overshot the track, or braked
// at the wrong moment and lost energy). The loop makes reversals fast and
// fixes the cart's travel by the commanded speed.
#define SPEED_MAX_PWM       180
float V_FF            = 0.25f;    // u per m/s (homing: PWM 50-60 ran ~0.5 m/s)   [online]
float KV_P            = 1.5f;     // u per m/s of speed error   [online]
float V_MAX           = 0.8f;     // m/s: speed command cap   [online]
#define SPEED_TRIP          1.8f     // m/s: faster than this = fault stop

// --- Swing-up (energy pumping on the speed loop) ---
// Energy E = 0.5*(th'/w0)^2 + cos(th) - 1: -2 hanging still, 0 upright still.
// Cart speed command = SWING_V opposite to the bob's side, reversing as the bob
// passes the bottom (when it is fastest), minus a pull back to centre. Travel
// per half swing ~ SWING_V * 0.56 s (0.22 m at 0.4 m/s).
float SWING_V         = 0.4f;     // m/s   [online]
// Energy control: push = SWING_V * clamp((target - E) / SWING_E_SLOW, -1, 1).
// Proportional, both ways: a small shortfall gets a small push (each reversal
// adds roughly 0.6 of energy per m/s of speed change), and too much energy
// pushes the other way and takes energy out. E is the height at the last swing
// end (cos(th_end) - 1, exact), or the running E within SWING_TOP_ZONE of
// upright, which also covers full rotations (no swing end). The running E
// reads ~0.25 high at the bottom where the reversal happens, so it's not used
// there.
// History: a 50% minimum push arrived at 2.4 rad/s (not catchable) and then,
// with no swing end to update E, pumped a spinning pendulum up to 16 rad/s.
// Target +0.02 arrives at the top at ~1 rad/s, inside CATCH_RATE.
float SWING_E_TARGET  = 0.02f;    // [online]
float SWING_E_SLOW    = 0.6f;     // [online]
#define SWING_TOP_ZONE      0.5f     // rad (~30 deg): running E trusted inside this
#define SWING_END_W         0.3f     // rad/s: |th'| above this sets the swing direction
// (Every swing-up run before the PEND_SIGN fix had the bob's side mirrored, so
// the observations below were made with the wrong sign.)
// When the reversal happens, as a phase lead on the bob's position (degrees):
// 0 = as the bob passes the bottom, 90 = at the swing ends. Set it to the
// cart's remaining reversal lag. Without the loop, the cart lagged ~90 deg, so
// 90 was the only value that pumped. Step test with the loop: +0.3 -> -0.3 m/s
// passed the target within ~180 ms, midpoint ~50-70 ms = ~15-20 deg.
float SWING_PHASE_DEG = 15.0f;    // [online]
#define SWING_HYST          0.03f    // ~2 deg of bob swing: stops the push chattering
// Reversals happen when the bob CROSSES from one side to the other. With no
// crossing for SWING_KICK_MS (pendulum at rest) reverse on the timer instead
// (the pendulum's half swing is 0.56 s). Crossing-triggered, so a steady offset
// can't undo the kick. At 600 it fired just before a natural crossing (small
// swings cross every ~565 ms), so keep it well above half a swing.
#define SWING_KICK_MS       800
// Kick only while nearly at rest (swing under ~25 deg). Big swings slow down
// near the top, so bottom crossings can be > 0.8 s apart: at ~130 deg the
// timer fired every swing ~70 deg before the bottom and the swing stalled at
// ~50 deg from upright.
#define SWING_KICK_E        -1.9f
float K_X_SWING       = 1.0f;     // m/s per m: pull back toward centre   [online]
// No outward speed beyond this fraction of half-track, until back inside 70%
// of it (no chatter at the edge)
#define SWING_X_FRAC        0.6f
#define SOFT_LIMIT_FRAC     0.85f    // fault stop beyond this fraction of half-track
// Soft limit trips early enough to stop in time: when |x| + v^2/(2*BRAKE_DECEL)
// passes it while moving outward. On a fault the speed loop brakes to a stop
// (up to BRAKE_MS) instead of coasting: coasting only slows ~1.3 m/s^2, so
// tripping AT the limit at 0.8 m/s slammed the cart into the end rail.
#define BRAKE_DECEL         3.0f     // m/s^2, conservative for powered braking
#define BRAKE_MS            600
#define BRAKE_DONE_V        0.05f    // m/s: stopped
// After a SOFT LIMIT brake, re-home and swing up again instead of stopping
// (e.g. the pendulum was knocked over and the cart ran out). Overspeed, limit
// switches and STOP still stop for good. Capped so a fault that keeps
// repeating can't loop forever; the count resets after a balance holds for
// AUTO_REHOME_GOOD_MS, or on a START press.
#define AUTO_REHOME_MAX     3
#define AUTO_REHOME_GOOD_MS 5000

// --- Speed loop step test ---
// 1 = after HANG, instead of swinging up: +STEP_V for STEP_MS, -STEP_V for
// STEP_MS, STEP_CYCLES times, then stop. Prints speed every STEP_PRINT_MS.
// Also checks the pendulum model: with the pendulum hanging, each cart
// acceleration a should kick it at th'' = +3.21*a (th moving from 180 toward
// -179, -178... when accelerating toward +x).
#define STEP_TEST           0
#define STEP_V              0.3f     // m/s (travel ~STEP_V * STEP_MS = 0.18 m)
// (the pendulum must be hanging still for the model check)
#define STEP_MS             600
#define STEP_CYCLES         3
#define STEP_PRINT_MS       20

// --- Finding "hanging down" after homing ---
// Average the count over exactly HANG_PERIODS swings: whole swings average to
// the centre wherever the window starts. The pendulum has so little friction
// that waiting for it to stop takes minutes. (The middle of min and max over
// 1.5 s was off by ~3.4 deg on the rig — enough to look like the bob sitting
// on one side forever.)
#define HANG_PERIODS        2
#define HANG_AMP_COUNTS     600      // ~±54 deg; larger = window restarts (period
                                     // grows with amplitude, so keep it moderate)
#define PRINT_MS            100

// --- State ---
bool systemEnabled  = false;
bool homingComplete = false;

// --- Cart and pendulum position (encoder counts, read by readEncoders()) ---
int32_t cart_position = 0;      // relative to cartZero (0 = centre after homing)
int32_t cartZero      = 0;      // raw count at the centre, set by homing
int32_t track_total   = 0;
int32_t track_center  = 0;
int     cartDir       = 1;      // +1 if driving right increases the count (set by homing)
int32_t pend_position = 0;      // pendulum count (32-bit, keeps counting past +/-32767)
bool    newSample     = false;  // a 2 ms sample was taken since the last control step
unsigned long lastSampleTickUs = 0;

enum Dir { DIR_LEFT, DIR_RIGHT };

// Standalone controller state (declared up here for the Arduino prototype
// generator, like StallGuard below)
enum BalState { BAL_HANG, BAL_CHECK, BAL_SWING, BAL_BALANCE, BAL_STEP, BAL_DONE, BAL_BRAKE };
BalState balState = BAL_HANG;

// Declared up here (not next to stallCheck) because the Arduino IDE inserts
// auto-generated function prototypes before the first function in the file.
struct StallGuard {
  int32_t       ref_pos;
  unsigned long ref_time;
  int           kicks;
  int32_t       kick_pos;   // position right after the last kick
};

const char* BAL_NAMES[] = { "HANG", "CHECK", "SWING", "BALANCE", "STEP TEST", "DONE", "BRAKE" };

// =============================================================================
// Online parameters: changeable over USB serial (bridge / website) while idle.
// The limits are enforced HERE, so the website can't push the rig outside them
// even if someone bypasses the frontend. Defaults are the values the variables
// start with (captured in setup()). Safety limits and calibration (soft limit,
// overspeed, PWM cap, counts/rev, PEND_SIGN, PEND_OMEGA0) are deliberately not
// in this table.
// =============================================================================
struct Param {
  const char* name;
  float*      value;
  float       lo, hi;
  const char* unit;
  const char* desc;
  float       def;      // filled in by captureParamDefaults()
};
Param PARAMS[] = {
  // Balance
  { "BAL_MODE",        &BAL_MODE,        0.0f,   1.0f,  "-",       "0 = gains from BAL_POLE, 1 = manual gains" },
  { "BAL_POLE",        &BAL_POLE,        3.0f,   7.0f,  "rad/s",   "balance stiffness (all four gains computed from it)" },
  { "BAL_K_TH",        &BAL_K_TH,        0.0f, 200.0f,  "m/s2/rad",  "manual gain: pendulum angle" },
  { "BAL_K_THD",       &BAL_K_THD,       0.0f,  40.0f,  "m/s2/(rad/s)", "manual gain: pendulum angular speed" },
  { "BAL_K_X",         &BAL_K_X,       -50.0f, 100.0f,  "m/s2/m",  "manual gain: cart position" },
  { "BAL_K_V",         &BAL_K_V,       -50.0f,  80.0f,  "m/s2/(m/s)", "manual gain: cart speed" },
  { "CATCH_ANGLE_DEG", &CATCH_ANGLE_DEG, 5.0f,  20.0f,  "deg",     "start balancing inside this angle from upright" },
  { "CATCH_RATE",      &CATCH_RATE,      1.0f,   3.0f,  "rad/s",   "...and only if turning slower than this" },
  { "DROP_ANGLE_DEG",  &DROP_ANGLE_DEG, 20.0f,  45.0f,  "deg",     "give up balancing past this angle" },
  // Swing-up
  { "SWING_V",         &SWING_V,         0.1f,   0.6f,  "m/s",     "swing-up cart speed" },
  { "SWING_E_TARGET",  &SWING_E_TARGET, -0.1f,   0.2f,  "-",       "energy target (0 = just reaches upright)" },
  { "SWING_E_SLOW",    &SWING_E_SLOW,    0.2f,   1.5f,  "-",       "how gradually the push eases off near the top" },
  { "SWING_PHASE_DEG", &SWING_PHASE_DEG, 0.0f,  90.0f,  "deg",     "push timing lead (0 = at the bottom, 90 = at swing ends)" },
  { "K_X_SWING",       &K_X_SWING,       0.0f,   3.0f,  "1/s",     "pull back toward centre while swinging" },
  // Cart speed loop
  { "KV_P",            &KV_P,            0.5f,   3.0f,  "1/(m/s)", "speed loop gain" },
  { "V_FF",            &V_FF,            0.1f,   0.5f,  "1/(m/s)", "speed loop feed-forward" },
  { "V_MAX",           &V_MAX,           0.2f,   1.0f,  "m/s",     "top cart speed" },
};
const int N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// --- USB serial commands ---
bool serialStart  = false;   // "home" received: acts like the START button
bool serialStop   = false;   // "stop" received: acts like the STOP button
bool homingActive = false;   // inside runHoming() (settings locked)
char cmdBuf[80];
int  cmdLen = 0;

// =============================================================================
// Motor helpers
// =============================================================================
void stopMotor() {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
}

void driveLeft(int pwm) {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, pwm);
}

void driveRight(int pwm) {
  ledcWrite(RPWM_PIN, pwm);
  ledcWrite(LPWM_PIN, 0);
}

void drive(Dir d, int pwm) {
  if (d == DIR_LEFT) driveLeft(pwm);
  else               driveRight(pwm);
}

const char* dirName(Dir d) { return d == DIR_LEFT ? "left" : "right"; }

// Status LED: on while homed and running (replaces the homing-done line to the Pi)
void readyLed(bool on) {
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
}

bool leftLimitHit()  { return digitalRead(LEFT_LIMIT)  == LOW; }
bool rightLimitHit() { return digitalRead(RIGHT_LIMIT) == LOW; }
bool anyLimitHit()   { return leftLimitHit() || rightLimitHit(); }

// Limit in the direction of travel, and the one we are moving away from
int homingPwm(Dir d) { return d == DIR_LEFT ? HOMING_PWM_LEFT : HOMING_PWM_RIGHT; }

bool limitAhead(Dir d)  { return d == DIR_LEFT ? leftLimitHit()  : rightLimitHit(); }
bool limitBehind(Dir d) { return d == DIR_LEFT ? rightLimitHit() : leftLimitHit(); }

// =============================================================================
// Trigger helpers — unify button and USB serial inputs
// =============================================================================
bool startTriggered() {
  return (digitalRead(START_BTN) == LOW) || serialStart;
}

bool stopTriggered() {
  // STOP_BTN is normally closed wired to GND:
  //   resting = closed = LOW = not stopped
  //   pressed = open   = HIGH (pulled up) = stopped
  return (digitalRead(STOP_BTN) == HIGH) || serialStop;
}

// =============================================================================
// USB serial command interface (for the bridge / website)
// One command per line. Every reply starts with "ok ", "err ", "param ",
// "params end" or "status ", so it can be told apart from the telemetry lines.
//   home | start        start homing + swing-up (only while idle)
//   stop                stop now (always accepted, also during homing)
//   status              -> status IDLE | HOMING | HANG | SWING | BALANCE | ...
//   params              -> param NAME VALUE MIN MAX DEFAULT UNIT | description
//   set NAME VALUE      change a setting (only while idle, within its limits)
//   defaults            restore every setting to its default (only while idle)
//   io                  -> io cart N pend N left 0|1 right 0|1 start 0|1 stop 0|1
//                          pins cartA cartB pendA pendB (encoder pin levels)
//                          (counts, switch states and encoder levels, for bring-up)
// =============================================================================
bool isIdle() { return !systemEnabled && !homingActive; }

const char* stateName() {
  if (homingActive)   return "HOMING";
  if (!systemEnabled) return "IDLE";
  return BAL_NAMES[balState];
}

void readEncoders();   // defined with the encoder code below

void captureParamDefaults() {
  for (int i = 0; i < N_PARAMS; i++) PARAMS[i].def = *PARAMS[i].value;
}

int findParam(const char* name) {
  for (int i = 0; i < N_PARAMS; i++)
    if (strcasecmp(PARAMS[i].name, name) == 0) return i;
  return -1;
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    const Param& p = PARAMS[i];
    Serial.printf("param %s %g %g %g %g %s | %s\n",
                  p.name, *p.value, p.lo, p.hi, p.def, p.unit, p.desc);
  }
  Serial.println("params end");
}

// One line with every setting, printed when a run starts so the log records it
void printRunParams() {
  Serial.print("run params:");
  for (int i = 0; i < N_PARAMS; i++)
    Serial.printf(" %s=%g", PARAMS[i].name, *PARAMS[i].value);
  Serial.println();
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "stop")) {
    serialStop = true;
    Serial.println("ok stop");
  } else if (!strcasecmp(cmd, "home") || !strcasecmp(cmd, "start")) {
    if (!isIdle()) { Serial.printf("err home busy (%s)\n", stateName()); return; }
    serialStart = true;
    Serial.println("ok home");
  } else if (!strcasecmp(cmd, "status")) {
    Serial.printf("status %s\n", stateName());
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
    readEncoders();
    // Raw encoder pin levels too: if a level toggles while its count stays put,
    // the fault is in the counting; if it never toggles, it's the wiring.
    Serial.printf("io cart %ld pend %ld left %d right %d start %d stop %d"
                  "  pins cartA %d cartB %d pendA %d pendB %d\n",
                  (long)cart_position, (long)pend_position, leftLimitHit(), rightLimitHit(),
                  digitalRead(START_BTN) == LOW, digitalRead(STOP_BTN) == HIGH,
                  digitalRead(CART_ENC_A), digitalRead(CART_ENC_B),
                  digitalRead(PEND_ENC_A), digitalRead(PEND_ENC_B));
  } else {
    Serial.printf("err %s unknown command (home, stop, status, params, set, defaults, io)\n", cmd);
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

// =============================================================================
// Encoders — PCNT quadrature decoders (from esp32_encoder_v9 v9.2)
// =============================================================================
// accum_count + watch points on both limits: the driver adds the limit to a
// software total each time the hardware counter overflows and resets, so
// pcnt_unit_get_count() keeps counting past +/-32767.
bool setupPcnt(pcnt_unit_handle_t* unit, int pin_a, int pin_b) {
  pcnt_unit_config_t unit_config = {
    .low_limit  = -32768,
    .high_limit =  32767,
  };
  unit_config.flags.accum_count = 1;
  if (pcnt_new_unit(&unit_config, unit) != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(*unit, unit_config.low_limit)  != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(*unit, unit_config.high_limit) != ESP_OK) return false;

  pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 5000 };
  if (pcnt_unit_set_glitch_filter(*unit, &filter_config) != ESP_OK) return false;

  pcnt_chan_config_t chan_a_config = {
    .edge_gpio_num  = pin_a,
    .level_gpio_num = pin_b,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  if (pcnt_new_channel(*unit, &chan_a_config, &pcnt_chan_a) != ESP_OK) return false;
  pcnt_channel_set_edge_action(pcnt_chan_a,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_a,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_chan_config_t chan_b_config = {
    .edge_gpio_num  = pin_b,
    .level_gpio_num = pin_a,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  if (pcnt_new_channel(*unit, &chan_b_config, &pcnt_chan_b) != ESP_OK) return false;
  pcnt_channel_set_edge_action(pcnt_chan_b,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_b,
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

  if (pcnt_unit_enable(*unit)      != ESP_OK) return false;
  if (pcnt_unit_clear_count(*unit) != ESP_OK) return false;
  if (pcnt_unit_start(*unit)       != ESP_OK) return false;
  return true;
}

// Read both counters (cheap — call as often as you like). Every SAMPLE_US it
// also flags a new sample, which runs one control step (500 Hz, the rate the
// UART packets from ESP32 #1 used to arrive at).
void readEncoders() {
  int c = 0, p = 0;
  pcnt_unit_get_count(cart_pcnt, &c);
  pcnt_unit_get_count(pend_pcnt, &p);
  cart_position = (int32_t)c - cartZero;
  pend_position = (int32_t)p;

  unsigned long now = micros();
  if (now - lastSampleTickUs >= SAMPLE_US) {
    lastSampleTickUs += SAMPLE_US;
    if (now - lastSampleTickUs >= SAMPLE_US) lastSampleTickUs = now;   // fell behind: resync
    newSample = true;
  }
}

// =============================================================================
// Stall detection + breakaway kick
// =============================================================================
void stallReset(StallGuard &g) {
  g.ref_pos  = cart_position;
  g.ref_time = millis();
  g.kicks    = 0;
  g.kick_pos = cart_position;
}

// Drive hard until the cart starts moving, then resume at resume_pwm
void kick(Dir d, int resume_pwm) {
  int32_t start = cart_position;
  unsigned long t0 = millis();
  drive(d, BREAKAWAY_PWM);
  while (millis() - t0 < BREAKAWAY_MS) {
    readEncoders();
    pollSerial();
    if (stopTriggered()) break;   // the caller's loop aborts on the same check
    if (abs(cart_position - start) >= KICK_MOVE_COUNTS) break;
    if (limitAhead(d)) break;
    delay(2);
  }
  drive(d, resume_pwm);
}

// Call every loop iteration while moving. Returns false if the cart is stuck
// and kicking did not free it (motor is stopped in that case).
bool stallCheck(StallGuard &g, Dir d, int pwm) {
  if (abs(cart_position - g.ref_pos) >= STALL_COUNTS) {
    g.ref_pos  = cart_position;
    g.ref_time = millis();
    // Coasting a few dozen counts after a kick is not recovery — only reset
    // the kick count once the cart has clearly kept moving on its own.
    if (abs(cart_position - g.kick_pos) >= KICK_PROGRESS_COUNTS) g.kicks = 0;
    return true;
  }
  if (millis() - g.ref_time < STALL_MS) return true;

  if (g.kicks >= MAX_KICKS) {
    stopMotor();
    Serial.printf("ERROR: Cart stalled driving %s at PWM %d after %d kicks. "
                  "Check mechanics or raise HOMING_PWM_%s/BREAKAWAY_PWM.\n",
                  dirName(d), pwm, MAX_KICKS, d == DIR_LEFT ? "LEFT" : "RIGHT");
    return false;
  }

  g.kicks++;
  Serial.printf("Stall driving %s at PWM %d (pos %ld) — breakaway kick %d/%d\n",
                dirName(d), pwm, (long)cart_position, g.kicks, MAX_KICKS);
  kick(d, pwm);
  // Movement produced by the kick itself does not count as progress
  g.ref_pos  = cart_position;
  g.ref_time = millis();
  g.kick_pos = cart_position;
  return true;
}

// =============================================================================
// Homing routine
// Steps:
//   1. Drive left to left limit
//   2. Drive right to right limit (ignore left limit until it releases)
//   3. Calculate center from total track counts
//   4. Drive to center (proportional speed, ignore limits until released)
//   5. Zero the cart count where the cart came to rest
// Returns true on success, false on any error (motor stopped).
// =============================================================================

// Drive in direction d until the limit ahead is hit.
bool homeToLimit(Dir d) {
  Serial.printf("Homing: driving %s...\n", dirName(d));

  StallGuard g;
  stallReset(g);
  unsigned long t0 = millis();
  bool behindReleased = false;
  unsigned long behindClearSince = millis();

  drive(d, homingPwm(d));
  while (!limitAhead(d)) {
    readEncoders();
    if (homingStopped()) return false;

    if (millis() - t0 > PHASE_TIMEOUT_MS) {
      stopMotor();
      Serial.printf("ERROR: Timed out driving %s\n", dirName(d));
      return false;
    }

    // Starting on the opposite limit is fine — wait for it to release (and
    // stay released) before treating it as an error.
    if (!behindReleased) {
      if (limitBehind(d)) {
        behindClearSince = millis();
      } else if (millis() - behindClearSince >= LIMIT_RELEASE_MS) {
        behindReleased = true;
      }
    } else if (limitBehind(d)) {
      stopMotor();
      Serial.printf("ERROR: Wrong limit hit while driving %s\n", dirName(d));
      return false;
    }

    if (!stallCheck(g, d, homingPwm(d))) return false;
    delay(5);
  }
  stopMotor();
  delay(200);
  readEncoders();
  return true;
}

// Proportional move until within CENTER_TOLERANCE of track_center (motor then
// stopped — the cart may still coast).
bool approachCenter() {
  StallGuard g;
  stallReset(g);
  unsigned long t0 = millis();
  Dir lastDir = DIR_LEFT;
  int bestError = 32767;
  bool limitsArmed = false;
  unsigned long limitsClearSince = millis();

  while (true) {
    readEncoders();
    if (homingStopped()) return false;

    if (millis() - t0 > PHASE_TIMEOUT_MS) {
      stopMotor();
      Serial.println("ERROR: Timed out centering");
      return false;
    }

    int error = cart_position - track_center;
    if (abs(error) < CENTER_TOLERANCE) {
      stopMotor();
      return true;
    }

    // Proportional speed — slow down as we approach center. Moving right needs
    // more drive, so shift the whole range up by the right/left difference.
    // Count above centre = cart right of centre only if driving right raises
    // the count (cartDir +1); with the encoder the other way round it is left.
    Dir d = error * cartDir > 0 ? DIR_LEFT : DIR_RIGHT;
    int maxPwm = homingPwm(d);
    int minPwm = CENTER_MIN_PWM + (maxPwm - HOMING_PWM_LEFT);
    int pwm = constrain(
      map(abs(error), CENTER_TOLERANCE, CENTER_SLOW_ZONE, minPwm, maxPwm),
      minPwm, maxPwm
    );
    if (d != lastDir) {        // reversed (overshoot) — restart stall timing
      stallReset(g);
      lastDir = d;
    }
    drive(d, pwm);

    // We start on the right limit — ignore limits until both read released
    if (!limitsArmed) {
      if (anyLimitHit()) {
        limitsClearSince = millis();
      } else if (millis() - limitsClearSince >= LIMIT_RELEASE_MS) {
        limitsArmed = true;
        Serial.println("Limits released, centering...");
      }
    } else if (anyLimitHit()) {
      stopMotor();
      Serial.println("ERROR: Limit hit during centering");
      return false;
    }

    // Getting closer to center counts as progress even if it took kicks
    if (abs(error) < bestError - STALL_COUNTS) {
      bestError = abs(error);
      g.kicks = 0;
    }

    if (!stallCheck(g, d, pwm)) return false;
    delay(5);
  }
}

// Wait for the cart to stop coasting after centering.
void waitForSettle() {
  int32_t ref = cart_position;
  unsigned long refTime = millis();
  unsigned long t0 = millis();
  while (millis() - t0 < SETTLE_TIMEOUT_MS) {
    readEncoders();
    pollSerial();
    if (stopTriggered()) return;   // centerCart() aborts on the same check
    if (abs(cart_position - ref) > SETTLE_COUNTS) {
      ref = cart_position;
      refTime = millis();
    } else if (millis() - refTime >= SETTLE_MS) {
      break;
    }
    delay(5);
  }
  Serial.printf("Settled at count %ld (center %ld, off by %ld)\n",
                (long)cart_position, (long)track_center, (long)(cart_position - track_center));
}

// Approach center, let the cart stop, and re-approach if it coasted too far.
bool centerCart() {
  Serial.println("Homing: driving to center...");
  for (int attempt = 1; attempt <= CENTER_ATTEMPTS; attempt++) {
    if (!approachCenter()) return false;
    waitForSettle();
    if (homingStopped()) return false;
    int off = cart_position - track_center;
    if (abs(off) <= CENTER_ACCEPT) return true;
    if (attempt < CENTER_ATTEMPTS) {
      Serial.printf("Overshot center by %d — correcting (attempt %d/%d)\n",
                    off, attempt + 1, CENTER_ATTEMPTS);
    }
  }
  Serial.printf("WARNING: still %ld counts from center after %d attempts — accepting\n",
                (long)(cart_position - track_center), CENTER_ATTEMPTS);
  return true;
}

bool doHoming() {
  if (!homeToLimit(DIR_LEFT)) return false;
  int32_t left_pos = cart_position;
  Serial.print("Left limit at count: "); Serial.println(left_pos);

  if (!homeToLimit(DIR_RIGHT)) return false;
  int32_t right_pos = cart_position;
  Serial.print("Right limit at count: "); Serial.println(right_pos);

  track_total  = abs(right_pos - left_pos);
  track_center = (left_pos + right_pos) / 2;
  cartDir      = right_pos > left_pos ? 1 : -1;
  Serial.printf("Driving right %s the count\n", cartDir > 0 ? "increases" : "decreases");
  Serial.print("Track total counts: "); Serial.println(track_total);
  Serial.print("Center count: ");       Serial.println(track_center);

  if (track_total < MIN_TRACK_COUNTS) {
    Serial.println("ERROR: Track too short — check encoder wiring");
    return false;
  }

  if (!centerCart()) return false;

  Serial.println("Homing complete!");

  // Count 0 = where the cart rests now (within CENTER_ACCEPT of the centre).
  // Done in software: the two-board build did this with a UART message that
  // motor noise could fake.
  readEncoders();
  cartZero += cart_position;
  readEncoders();

  return true;
}

// Full homing sequence plus enabling run mode on success. Settings are locked
// (homingActive) while it runs; a START or STOP that arrived before it is
// cleared so it can't act twice.
void runHoming() {
  homingActive = true;
  serialStart  = false;
  serialStop   = false;
  runHomingSteps();
  homingActive = false;
}

void runHomingSteps() {
  readyLed(false);
  systemEnabled = false;

  homingComplete = doHoming();
  if (!homingComplete) {
    stopMotor();
    Serial.println("ERROR: Homing failed. Press START to retry.");
    return;
  }

  // Cart is centered, so no limit should be pressed. Give switches a moment
  // to settle; if one still reads pressed, something is wrong.
  unsigned long t0 = millis();
  while (anyLimitHit() && millis() - t0 < 2000) { readEncoders(); pollSerial(); delay(10); }
  if (homingStopped()) { homingComplete = false; return; }
  if (anyLimitHit()) {
    homingComplete = false;
    Serial.println("ERROR: Limit switch still pressed after centering. Press START to retry.");
    return;
  }
  delay(300);  // additional debounce settle time

  readyLed(true);
  systemEnabled = true;
  standaloneStart();
}

// =============================================================================
// Standalone swing-up + balance
// Sign conventions (after cartDir / PEND_SIGN):
//   x     > 0 : cart right of centre (m)
//   theta > 0 : top of pendulum leans right; 0 = upright, +/-pi = hanging (rad)
//   u     > 0 : drive right
// =============================================================================
int32_t       pendDownRef = 0;
float         theta = 0, thetaDot = 0, xPos = 0, xDot = 0, energy = -2, uOut = 0, vRefOut = 0;
float         stateDt = 0.002f;     // seconds between the last two samples
float         energyEnd = -2;       // energy at the last swing end (cos(th) - 1)
int           swingDir  = 0;        // sign of th' (with SWING_END_W hysteresis)
float         balVRef = 0;          // balance: integrated speed command
float         balTrim = 0;          // balance: learned upright offset (rad)
unsigned long balStartMs = 0;       // when the current balance began
float         kaTh = 0, kaThd = 0, kaX = 0, kaV = 0;   // balance gains (balanceGains())
int32_t       lastCart = 0;
int32_t       lastPend = 0;
unsigned long lastSampleUs = 0;
bool          haveSample = false;
int           pumpDir = 1;
int           bobSide = 0;           // side of the (phase-led) bob: -1, +1, 0 = not yet known
unsigned long lastFlipMs = 0;
bool          pumpBlocked = false;   // cart past SWING_X_FRAC: no outward pushes
int32_t       hangMin = 0, hangMax = 0;
int64_t       hangSum = 0;
uint32_t      hangN   = 0;
unsigned long hangStart = 0;
unsigned long stepStart = 0;
unsigned long brakeStart = 0;
bool          brakeRehome = false;   // re-home after this brake (soft limit)
int           autoRehomes = 0;       // automatic re-homes since the last good balance
unsigned long lastPrint = 0;

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

void standaloneStart() {
  stopMotor();
  readEncoders();
  uOut       = 0;
  vRefOut    = 0;
  haveSample = false;
  balState   = BAL_HANG;
  hangMin    = hangMax = pend_position;
  hangSum    = 0;
  hangN      = 0;
  hangStart  = millis();
  lastFlipMs = millis();
  bobSide    = 0;
  pumpBlocked = false;
  energyEnd  = -2;
  swingDir   = 0;
  // balTrim is NOT reset here: the upright offset is the same after a re-home
  // (5.5-6.4 deg on every run of 2026-10-02), and starting from 0 after an
  // automatic re-home parked the balanced cart at -384 mm, into the soft limit.
  balanceGains();
  printRunParams();
  Serial.println("Standalone: waiting for the pendulum to hang still...");
}

void enterState(BalState s) {
  balState = s;
  const char** names = BAL_NAMES;
  Serial.printf(">> %s\n", names[s]);
}

// Update angle, position and filtered velocities from the latest packet
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
    stepStart  = millis();
    lastFlipMs = millis();
    enterState(SIGN_CHECK ? BAL_CHECK : STEP_TEST ? BAL_STEP : BAL_SWING);
  } else {
    hangMin = hangMax = pend_position;
    hangSum = 0;
    hangN   = 0;
    hangStart = millis();
  }
}

// Cart speed command for swing-up
float swingV() {
  // Move the cart opposite to the bob's sideways position, advanced by
  // SWING_PHASE_DEG to make up for the time the cart takes to reverse.
  // Bob x = l*sin(th); its velocity / w0 = cos(th)*th'/w0 has the same scale.
  const float ph = SWING_PHASE_DEG * DEG_TO_RAD;
  float bobPos = sinf(theta);
  float bobVel = cosf(theta) * thetaDot / PEND_OMEGA0;
  float lead   = bobPos * cosf(ph) + bobVel * sinf(ph);
  // Swing energy from height at each swing end (th' changes sign): th' = 0
  // there, so cos(th) - 1 is the exact energy, and it is known half a swing
  // before the next reversal at the bottom. (Recording the peak at the next
  // bottom crossing instead ran half a swing late: after a big energy change
  // it kept pushing weakly on a small swing, or hard enough to spin it over.)
  float height = cosf(theta) - 1.0f;
  int dir = swingDir;
  if      (thetaDot >  SWING_END_W) dir =  1;
  else if (thetaDot < -SWING_END_W) dir = -1;
  if (dir != swingDir) {
    if (swingDir != 0) energyEnd = height;
    swingDir = dir;
  }
  if (fabsf(theta) < SWING_TOP_ZONE) energyEnd = energy;

  int side = bobSide;
  if      (lead >  SWING_HYST) side =  1;
  else if (lead < -SWING_HYST) side = -1;
  if (side != bobSide) {
    // Bob crossed (or first reading): move opposite to its new side
    bobSide = side;
    pumpDir = -side;
    lastFlipMs = millis();
  } else if (energy < SWING_KICK_E && millis() - lastFlipMs > SWING_KICK_MS) {
    // No crossing for a while and barely swinging — reverse on the timer
    pumpDir = -pumpDir;
    lastFlipMs = millis();
  }

  // The current height is also a lower bound on energy (covers going over the top)
  float eNow = max(energyEnd, height);

  // Below target: pump. Above target: the negative scale pushes the other way
  // and takes energy out.
  float need = SWING_E_TARGET - eNow;
  float pump = pumpDir * SWING_V * constrain(need / SWING_E_SLOW, -1.0f, 1.0f);

  // Too far out: no outward speed until well back inside
  float lim = SWING_X_FRAC * halfTrackM();
  if      (fabsf(xPos) > lim)         pumpBlocked = true;
  else if (fabsf(xPos) < 0.7f * lim)  pumpBlocked = false;
  if (pumpBlocked && pump * xPos > 0) pump = 0;

  return pump - K_X_SWING * xPos;
}

// Balance gains for all four closed-loop poles at -p. With l = g/w0^2 and
// a = kaTh*th + kaThd*th' + kaX*x + kaV*v, the characteristic polynomial is
//   s^4 + (kaThd/l - kaV) s^3 + (kaTh/l - kaX - w0^2) s^2 + w0^2 kaV s + w0^2 kaX
// Matching (s + p)^4 = s^4 + 4p s^3 + 6p^2 s^2 + 4p^3 s + p^4 gives:
void balanceGains() {
  if (BAL_MODE >= 0.5f) {   // manual gains typed in by the user
    kaTh = BAL_K_TH;  kaThd = BAL_K_THD;  kaX = BAL_K_X;  kaV = BAL_K_V;
    Serial.printf("Balance gains (manual): th %.1f  th' %.1f  x %.1f  v %.1f\n",
                  kaTh, kaThd, kaX, kaV);
    return;
  }
  const float p  = BAL_POLE;
  const float w2 = PEND_OMEGA0 * PEND_OMEGA0;
  const float l  = 9.81f / w2;
  kaX   = p * p * p * p / w2;
  kaV   = 4.0f * p * p * p / w2;
  kaTh  = l * (6.0f * p * p + kaX + w2);
  kaThd = l * (4.0f * p + kaV);
  Serial.printf("Balance gains (pole %.1f): th %.1f  th' %.1f  x %.1f  v %.1f\n",
                p, kaTh, kaThd, kaX, kaV);
}

// Called on the swing -> balance switch: start the speed command from the
// cart's current speed so the handover doesn't jerk the cart
void balanceStart() {
  balVRef = xDot;
  balStartMs = millis();   // balTrim itself is kept across catches
}

// Cart speed command while balancing
float balanceV() {
  // Slow self-trim: a cart parked off-centre means the angle reads off; shift
  // the trim until the cart parks at centre (see BAL_TRIM_RATE)
  if (millis() - balStartMs > BAL_TRIM_DELAY_MS)
    balTrim = constrain(balTrim - BAL_TRIM_RATE * xPos * stateDt, -BAL_TRIM_MAX, BAL_TRIM_MAX);
  float a = kaTh * (theta - balTrim) + kaThd * thetaDot + kaX * xPos + kaV * xDot;
  balVRef = constrain(balVRef + a * stateDt, -V_MAX, V_MAX);
  return balVRef;
}

void standaloneStep() {
  if (!newSample) return;
  newSample = false;

  if (balState == BAL_HANG) {
    stepHang();
  } else {
    updateState();

    bool checkFaults = balState != BAL_CHECK && balState != BAL_DONE && balState != BAL_BRAKE;
    const char* fault = NULL;
    if (checkFaults) {
      float softLim  = SOFT_LIMIT_FRAC * halfTrackM();
      float stopDist = xDot * xDot / (2.0f * BRAKE_DECEL);
      bool  outward  = xPos * xDot > 0;
      if (fabsf(xPos) > softLim || (outward && fabsf(xPos) + stopDist > softLim))
        fault = "SOFT LIMIT";
      if (fabsf(xDot) > SPEED_TRIP) fault = "OVERSPEED";
    }
    if (fault) {
      brakeRehome = (strcmp(fault, "SOFT LIMIT") == 0) && autoRehomes < AUTO_REHOME_MAX;
      Serial.printf("%s: cart at %.0f mm, %.2f m/s — braking, then %s\n",
                    fault, xPos * 1000.0f, xDot,
                    brakeRehome ? "re-homing." : "stopping. Press START to re-home.");
      readyLed(false);
      brakeStart = millis();
      enterState(BAL_BRAKE);
    }

    switch (balState) {
      case BAL_BRAKE:
        // Powered stop, then disable. Timeout in case the loop can't stop it.
        if (fabsf(xDot) < BRAKE_DONE_V || millis() - brakeStart > BRAKE_MS) {
          stopMotor();
          uOut = vRefOut = 0;
          Serial.printf("Stopped at %.0f mm.\n", xPos * 1000.0f);
          if (brakeRehome) {
            autoRehomes++;
            Serial.printf("Auto re-home %d/%d\n", autoRehomes, AUTO_REHOME_MAX);
            delay(500);      // let the cart and pendulum settle a moment
            runHoming();     // re-homes, then standaloneStart() swings up again
          } else {
            systemEnabled = false;
          }
        } else {
          speedDrive(0);
        }
        return;
      case BAL_CHECK:
      case BAL_DONE:
        stopMotor();
        uOut = vRefOut = 0;
        break;
      case BAL_STEP: {
        unsigned long t = millis() - stepStart;
        if (t >= 2UL * STEP_MS * STEP_CYCLES) {
          stopMotor();
          uOut = vRefOut = 0;
          enterState(BAL_DONE);
          Serial.println("Step test done. Set STEP_TEST 0 for swing-up.");
        } else {
          speedDrive((t / STEP_MS) % 2 == 0 ? STEP_V : -STEP_V);
        }
        break;
      }
      case BAL_SWING:
        if (fabsf(theta) < CATCH_ANGLE_DEG * DEG_TO_RAD && fabsf(thetaDot) < CATCH_RATE) {
          enterState(BAL_BALANCE);
          balanceStart();
          speedDrive(balanceV());
        } else {
          speedDrive(swingV());
        }
        break;
      case BAL_BALANCE:
        if (fabsf(theta) > DROP_ANGLE_DEG * DEG_TO_RAD) {
          enterState(BAL_SWING);
          speedDrive(swingV());
        } else {
          speedDrive(balanceV());
          if (autoRehomes && millis() - balStartMs > AUTO_REHOME_GOOD_MS) {
            autoRehomes = 0;   // balance held: allow the full retry budget again
            Serial.println("Balance holding — auto re-home count reset.");
          }
        }
        break;
      default:
        break;
    }
  }

  // Step test and balance print fast so the cart and pendulum response is visible
  unsigned long printEvery =
    (balState == BAL_STEP || balState == BAL_BALANCE) ? STEP_PRINT_MS : PRINT_MS;
  if (millis() - lastPrint >= printEvery) {
    lastPrint = millis();
    if (balState == BAL_HANG) {
      Serial.printf("HANG  pend %ld  swing %ld\n", (long)pend_position, (long)(hangMax - hangMin));
    } else if (balState == BAL_STEP) {
      Serial.printf("STEP t %4lu  vr %5.2f  v %6.3f  u %5.2f  x %6.1f  th %7.2f  w %6.2f\n",
                    millis() - stepStart, vRefOut, xDot, uOut, xPos * 1000.0f,
                    theta * RAD_TO_DEG, thetaDot);
    } else if (balState == BAL_BALANCE) {
      Serial.printf("BAL   th %7.1f deg  w %6.2f  x %6.1f mm  v %6.3f  vr %5.2f  trim %5.2f deg  u %5.2f\n",
                    theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, xDot, vRefOut,
                    balTrim * RAD_TO_DEG, uOut);
    } else if (balState != BAL_DONE) {
      Serial.printf("%s th %7.1f deg  w %6.2f  x %6.1f mm  v %6.3f  vr %5.2f  E %5.2f  Eend %5.2f  u %5.2f\n",
                    balState == BAL_CHECK ? "CHECK" : balState == BAL_SWING ? "SWING" : "BAL  ",
                    theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, xDot, vRefOut, energy,
                    energyEnd, uOut);
    }
  }
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  // --- Hold motor outputs LOW immediately — prevents runaway on boot ---
  // Do this before ledcAttach so pins are driven low, not floating
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);

  Serial.begin(115200);
  Serial.println();
  Serial.println("=== ESP32 Single-Board Pendulum Controller v" FW_VERSION " ===");
  captureParamDefaults();   // the starting values become the "defaults"

  pinMode(STATUS_LED, OUTPUT);
  readyLed(false);

  // Now attach LEDC PWM (25kHz, 8-bit, above audible range)
  ledcAttach(RPWM_PIN, 25000, 8);
  ledcAttach(LPWM_PIN, 25000, 8);
  stopMotor();   // redundant but explicit

  // --- GPIO setup ---
  // (Buttons and limits also have external 10k pull-ups + 100 nF on the
  // breadboard; the internal pull-ups are a backup.)
  pinMode(START_BTN,    INPUT_PULLUP);
  pinMode(STOP_BTN,     INPUT_PULLUP);
  pinMode(LEFT_LIMIT,   INPUT_PULLUP);
  pinMode(RIGHT_LIMIT,  INPUT_PULLUP);

  WiFi.mode(WIFI_OFF);

  // --- Encoders ---
  if (!setupPcnt(&cart_pcnt, CART_ENC_A, CART_ENC_B) ||
      !setupPcnt(&pend_pcnt, PEND_ENC_A, PEND_ENC_B)) {
    Serial.println("ERROR: encoder (PCNT) setup failed. Halting with the motor off.");
    while (1) { stopMotor(); delay(100); }
  }
  lastSampleTickUs = micros();
  readEncoders();

  Serial.println("Ready. Waiting for START (button or \"home\") to home...");

  // --- Wait for start trigger (button or serial) ---
  while (!startTriggered()) {
    readEncoders();
    pollSerial();
    delay(10);
  }
  Serial.println(serialStart ? "START from serial" : "START from button");
  delay(200);  // debounce

  runHoming();
}

// =============================================================================
// Loop — safety checks, then one swing-up/balance step per 2 ms sample
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
    readyLed(false);
    systemEnabled = false;
    Serial.printf("LIMIT HIT (%s) — motor stopped. Press START to re-home.\n",
                  leftLimitHit() ? "left" : "right");
  }

  // --- Stop trigger (button or serial) ---
  if (stopTriggered() && systemEnabled) {
    stopMotor();
    readyLed(false);
    systemEnabled = false;
    serialStop = false;
    Serial.println("STOP triggered — motor stopped. Press START to re-home.");
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

  // --- Swing-up + balance, one control step per 2 ms sample ---
  standaloneStep();
}
