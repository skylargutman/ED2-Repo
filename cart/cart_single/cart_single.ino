// =============================================================================
// Single-board cart + pendulum controller (one ESP32)
// Version: 11.1
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
// Changes in 11.1 (experiments framework; design in
// docs/superpowers/specs/2026-10-07-esp32-experiments-design.md):
//   - Code split into tabs: hardware.ino, protocol.ino, control.ino,
//     exp_*.ino (one per experiment), zz_registry.ino (experiment list).
//   - Experiments chosen at runtime: "exp list", "exp select NAME", "run".
//     SwingBal (swing-up + balance), SignCheck and StepTest; the last two
//     replace the SIGN_CHECK / STEP_TEST compile switches.
//   - USB serial at 921600 baud. Each run prints "run start NAME K=V ...",
//     one "D t_ms x theta u ref" line per 2 ms step, then "run end REASON".
//   - PEND_TRIM_DEG stays 0: a 5.5 preset broke the swing-up (2026-10-08 log);
//     the balance self-trim handles the upright offset.
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
#include <Preferences.h>

#define FW_VERSION "11.1-single"

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
// Sign check = the "SignCheck" experiment: motor off after homing, angle and
// position printed so the signs can be checked by hand (docs/esp32.md).
// Passed on the single-board build 2026-10-07 (PEND_SIGN -1 still correct).

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
// Upright offset (degrees), subtracted from the angle EVERYWHERE, including at
// the bottom. Keep it 0: on 2026-10-08 a 5.5 preset made a hanging pendulum read
// 174.5 deg, so the swing-up saw the bob 5.5 deg to one side (more than
// SWING_HYST) and took 22 s instead of ~6. The upright offset is also not
// constant (it moves with counts lost in the ~124 deg encoder dead zone: 7 counts
// on 2026-10-07, ~34 on 2026-10-08), so the balance self-trim (balTrim, kept
// across re-homes) handles it instead.
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
// Motor command in volts, Feedback convention: +-2.5 V = full scale (u = 1)
#define U_FULL_SCALE_V      2.5f

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

// --- Friction compensation for motorVolts() (measured by the Friction experiment) ---
// Breakaway motor command per direction, in volts on the motorRawVolts() scale
// (2.5 V = SPEED_MAX_PWM). Defaults equal the offsets the speed loop has always
// used (RUN_MIN_PWM, + RUN_RIGHT_EXTRA_PWM to the right): 0.556 / 0.417 V.
// Saved in flash when measured or set; reloaded at boot; "defaults" restores
// these compiled values and clears flash.
float FRIC_POS_V = (RUN_MIN_PWM + RUN_RIGHT_EXTRA_PWM) * U_FULL_SCALE_V / SPEED_MAX_PWM;   // [online]
float FRIC_NEG_V = RUN_MIN_PWM * U_FULL_SCALE_V / SPEED_MAX_PWM;                           // [online]
#define FRIC_MAX_V          1.5f     // limit for stored values (PARAMS range)

// --- Free swing (experiment 1): motor off, recording time ---
float FS_DURATION_S   = 30.0f;    // s   [online]

// --- Static friction (experiment 2): ramp until the cart moves ---
float FR_RAMP_VPS     = 0.5f;     // V/s ramp of the raw motor command   [online]
#define FR_MOVE_COUNTS      13       // ~1 mm of cart travel = "it moved"
#define FR_SETTLE_MS        1000     // motor off between the two directions
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

// --- Speed loop step test (the "StepTest" experiment) ---
// +STEP_V for STEP_MS, -STEP_V for STEP_MS, STEP_CYCLES times, then the run
// ends. Prints speed every STEP_PRINT_MS.
// Also checks the pendulum model: with the pendulum hanging, each cart
// acceleration a should kick it at th'' = +3.21*a (th moving from 180 toward
// -179, -178... when accelerating toward +x).
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
void readEncoders();   // defined with the encoder code below

enum Dir { DIR_LEFT, DIR_RIGHT };

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
// "run end" reason when step() returns false: "done", or "error" when the
// experiment failed (the console then shows the run in red)
const char* expEndReason = "done";

enum RunPhase { PH_IDLE, PH_HANG, PH_EXP, PH_BRAKE };
RunPhase      phase      = PH_IDLE;
bool          runActive  = false;    // "run start" printed, "run end" not yet
unsigned long runStartMs = 0;

// Declared up here (not next to stallCheck) because the Arduino IDE inserts
// auto-generated function prototypes before the first function in the file.
struct StallGuard {
  int32_t       ref_pos;
  unsigned long ref_time;
  int           kicks;
  int32_t       kick_pos;   // position right after the last kick
};


// =============================================================================
// Online parameters: changeable over USB serial (bridge / website) while idle.
// The limits are enforced HERE, so the website can't push the rig outside them
// even if someone bypasses the frontend. Defaults are the values the variables
// start with (captured in setup()). Safety limits and calibration (soft limit,
// overspeed, PWM cap, counts/rev, PEND_SIGN, PEND_OMEGA0) are deliberately not
// in this table.
// =============================================================================
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
  // Friction compensation used by motorVolts() (measured and saved by "Friction")
  { "Friction", "FRIC_POS_V",      &FRIC_POS_V,      0.0f,   FRIC_MAX_V, "V",  "breakaway command driving right (+x)" },
  { "Friction", "FRIC_NEG_V",      &FRIC_NEG_V,      0.0f,   FRIC_MAX_V, "V",  "breakaway command driving left (-x)" },
  // FreeSwing
  { "FreeSwing", "FS_DURATION_S",  &FS_DURATION_S,   5.0f, 180.0f,  "s",       "how long to record the free swing" },
  // Friction (the measured values are FRIC_POS_V / FRIC_NEG_V above)
  { "Friction", "FR_RAMP_VPS",     &FR_RAMP_VPS,     0.1f,   2.0f,  "V/s",     "how fast the motor command ramps up" },
};

const int N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// --- USB serial commands ---
bool serialStart  = false;   // "home" received: acts like the START button
bool serialStop   = false;   // "stop" received: acts like the STOP button
bool homingActive = false;   // inside runHoming() (settings locked)
char cmdBuf[80];
int  cmdLen = 0;

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

  // D lines run at 500 Hz (~20 KB/s): a TX buffer keeps printing from
  // blocking the control step, and 921600 baud leaves plenty of headroom
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  Serial.println();
  Serial.println("=== ESP32 Single-Board Pendulum Controller v" FW_VERSION " ===");
  registerExperiments();    // fills EXPS[], selects EXPS[0]
  captureParamDefaults();   // the starting values become the "defaults"
  loadFriction();           // measured friction compensation, if saved

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
