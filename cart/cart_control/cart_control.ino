// =============================================================================
// ESP32 #2 — Motor Control Firmware
// Version: 10.0
// Description: Controls BTS7960 IBT-2 H-bridge motor driver.
//              Holds motor outputs LOW at boot to prevent runaway on power-up.
//              Waits for START button OR Pi GPIO trigger before homing.
//              Runs homing routine, sends 0xAA to ESP32 #1 on completion.
//              Drives GPIO 18 HIGH when homing complete — Pi reads this to know
//              when to launch Simulink model.
//              Pi GPIO stop signal mirrors physical STOP button behavior.
//              Normal operation reads DAC voltage from ADS1115 and drives motor,
//              or (STANDALONE_SWINGUP) swings the pendulum up and balances it
//              on this board alone, with no Pi.
//
// Changes in 10.0 (requires encoder firmware v9.2 — flash both boards):
//   - UART packet now also carries the pendulum count (7 bytes).
//   - STANDALONE_SWINGUP mode: after homing, wait for the pendulum to hang
//     still, pump energy until it swings up, catch it near upright and
//     balance with state feedback on a cart acceleration (gains computed by
//     pole placement from the measured swing period). Falls back to swing-up
//     if it drops, stops at a software cart limit.
//   - Homing-complete message to ESP32 #1 is AA 55 A5 5A (was one 0xAA byte,
//     which motor noise produced at random, zeroing the counts mid-run).
//   - SIGN_CHECK: motor stays off after homing and angle/position are
//     printed, to confirm sign conventions before the motor is allowed to run.
//   - Swing-up runs on a cart speed loop (SPEED_MAX_PWM 180); STEP_TEST
//     checks that loop on its own first. Soft-limit and overspeed fault stops.
//
// Changes in 9.8 (requires encoder firmware v9.1 — flash both boards):
//   - New UART packet format with 7-bit data bytes. The old format put 0xFF
//     in a data byte for positions -256..-1, freezing cart_position there and
//     making centering hunt back and forth when center was near 0.
//   - After centering, wait for the cart to stop before sending 0xAA, so
//     ESP32 #1 zeroes its counts where the cart actually rests.
//   - Centering slows down over the last 3000 counts (was 500) and re-approaches
//     if the cart settles more than CENTER_ACCEPT from center. With the short
//     ramp it arrived at full speed and coasted ~1400 counts past center.
//
// Changes in 9.7:
//   - Homing: separate speed per direction. At PWM 45 the cart could not keep
//     moving right at all, so HOMING_PWM_RIGHT is higher than HOMING_PWM_LEFT.
//   - Homing: stall detection with a short breakaway kick as a backstop.
//   - Homing: every wait loop has a timeout. A stalled motor now fails homing
//     with an error instead of sitting powered forever.
//   - Run mode: motor only follows the DAC after it has seen the 2.5V neutral
//     ("armed"), and disarms if the signal sits near 0V. Without the Pi, the
//     ADS1115 reads ~0V, which v9.6 treated as full-speed left.
//   - Boot banner printed first thing so the running version is obvious.
//
// Pin Assignments:
//   GPIO 25 : RPWM  (IBT-2)
//   GPIO 26 : LPWM  (IBT-2)
//   GPIO 27 : START button         (INPUT_PULLUP, active LOW)
//   GPIO 14 : STOP button          (INPUT_PULLUP, active LOW)
//   GPIO 32 : Left limit switch    (INPUT_PULLUP, active LOW)
//   GPIO 33 : Right limit switch   (INPUT_PULLUP, active LOW)
//   GPIO 17 : UART2 TX → ESP32 #1
//   GPIO 16 : UART2 RX ← ESP32 #1
//   GPIO 21 : I2C SDA (ADS1115)
//   GPIO 22 : I2C SCL (ADS1115)
//   GPIO  4 : Pi homing start trigger  (INPUT_PULLDOWN, active HIGH)
//   GPIO  5 : Pi stop trigger          (INPUT_PULLDOWN, active HIGH)
//   GPIO 18 : Homing complete signal   (OUTPUT, Pi GPIO 13 Pin 33)
//
// UART Protocol (receive from ESP32 #1, encoder firmware v9.2+):
//   7-byte framed packet every 2ms:
//   0xFF | m[6:0] | m[13:7] | m[15:14] | p[6:0] | p[13:7] | p[15:14]
//   → int16 cart count, low 16 bits of pendulum count (unwrapped here).
//   Data bytes are 7-bit so never equal 0xFF.
//
// UART Protocol (send to ESP32 #1):
//   AA 55 A5 5A = homing complete (was a single 0xAA — see doHoming)
// =============================================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define FW_VERSION "10.0"

Adafruit_ADS1115 ads;

// --- Motor pins ---
#define RPWM_PIN      25
#define LPWM_PIN      26

// --- Button pins ---
#define START_BTN     27
#define STOP_BTN      14

// --- Limit switch pins ---
#define LEFT_LIMIT    32
#define RIGHT_LIMIT   33

// --- UART2 pins ---
#define UART_TX       17
#define UART_RX       16

// --- Pi GPIO trigger pins ---
#define PI_START_PIN   4   // Pi GPIO 16, Pi Pin 36 — MQTT homing trigger
#define PI_STOP_PIN    5   // Pi GPIO 20, Pi Pin 38 — MQTT stop trigger

// --- Homing complete signal to Pi ---
#define HOMING_DONE_PIN 18  // Pi GPIO 13, Pi Pin 33 — HIGH when homing complete

// --- Control constants ---
#define NEUTRAL_VOLTAGE   2.5f
#define DEADBAND          0.05f
#define MAX_INPUT         2.5f
#define MAX_PWM           47
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

// --- Settle after centering (before telling ESP32 #1 to zero its counts) ---
#define SETTLE_COUNTS     5      // moving less than this counts as stopped
#define SETTLE_MS         200    // must stay stopped this long
#define SETTLE_TIMEOUT_MS 2000

// --- DAC signal guard (run mode) ---
// A disconnected Pi/DAC reads ~0V on the ADS1115. Below SIGNAL_LOST_V for
// SIGNAL_LOST_MS → treat as lost and stop until neutral (2.5V) is seen again.
#define SIGNAL_LOST_V     0.1f
#define SIGNAL_LOST_MS    200

// =============================================================================
// Standalone swing-up + balance (no Pi)
// =============================================================================
// 1 = after homing this board swings the pendulum up and balances it itself.
// 0 = after homing follow the Pi's DAC voltage (original behaviour).
#define STANDALONE_SWINGUP 1
// 1 = motor stays OFF after homing; prints angle and position so the signs
// can be checked by hand. Set to 0 only after the check passes (docs/esp32.md).
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
#define BAL_POLE            5.0f     // rad/s
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
#define CATCH_ANGLE         0.25f    // rad (~14 deg) — switch to balance inside this
// rad/s — too fast to catch above this. Stopping rotation w takes a cart speed
// change of ~w * g/w0^2 = 0.31*w m/s: a catch at 2.37 rad/s saturated the
// speed command (0.8 m/s) and fell. Faster passes just keep swinging.
#define CATCH_RATE          2.0f
#define DROP_ANGLE          0.6f     // rad (~35 deg) — lost it, back to swing-up

// --- Cart speed loop (swing-up and step test) ---
// u = V_FF*vRef + KV_P*(vRef - v), mapped to RUN_MIN_PWM..SPEED_MAX_PWM.
// Commanding PWM directly, the cart took ~half a swing to reverse, and every
// swing-up rule built on that fought the lag (overshot the track, or braked
// at the wrong moment and lost energy). The loop makes reversals fast and
// fixes the cart's travel by the commanded speed.
#define SPEED_MAX_PWM       180
#define V_FF                0.25f    // u per m/s (homing: PWM 50-60 ran ~0.5 m/s)
#define KV_P                1.5f     // u per m/s of speed error
#define V_MAX               0.8f     // m/s: speed command cap
#define SPEED_TRIP          1.8f     // m/s: faster than this = fault stop

// --- Swing-up (energy pumping on the speed loop) ---
// Energy E = 0.5*(th'/w0)^2 + cos(th) - 1: -2 hanging still, 0 upright still.
// Cart speed command = SWING_V opposite to the bob's side, reversing as the bob
// passes the bottom (when it is fastest), minus a pull back to centre. Travel
// per half swing ~ SWING_V * 0.56 s (0.22 m at 0.4 m/s).
#define SWING_V             0.4f     // m/s
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
#define SWING_E_TARGET      0.02f
#define SWING_E_SLOW        0.6f
#define SWING_TOP_ZONE      0.5f     // rad (~30 deg): running E trusted inside this
#define SWING_END_W         0.3f     // rad/s: |th'| above this sets the swing direction
// (Every swing-up run before the PEND_SIGN fix had the bob's side mirrored, so
// the observations below were made with the wrong sign.)
// When the reversal happens, as a phase lead on the bob's position (degrees):
// 0 = as the bob passes the bottom, 90 = at the swing ends. Set it to the
// cart's remaining reversal lag. Without the loop, the cart lagged ~90 deg, so
// 90 was the only value that pumped. Step test with the loop: +0.3 -> -0.3 m/s
// passed the target within ~180 ms, midpoint ~50-70 ms = ~15-20 deg.
#define SWING_PHASE_DEG     15.0f
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
#define K_X_SWING           1.0f     // m/s per m: pull back toward centre
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
bool signalArmed    = false;   // true once the DAC has been seen at neutral
bool signalLow      = false;
unsigned long signalLowSince = 0;

// --- Cart and pendulum position received from ESP32 #1 via UART ---
volatile int16_t cart_position = 0;
int16_t track_total  = 0;
int16_t track_center = 0;
int     cartDir      = 1;       // +1 if driving right increases the count (set by homing)
int32_t pend_position = 0;      // unwrapped pendulum count
int16_t pend_raw      = 0;      // last 16-bit value received
bool    pend_seen     = false;
bool    newSample     = false;  // a full packet arrived since last control step

// --- UART receive buffer ---
#define UART_DATA_BYTES 6
uint8_t uart_buf[UART_DATA_BYTES];
int     uart_idx = UART_DATA_BYTES;   // wait for first header

enum Dir { DIR_LEFT, DIR_RIGHT };

// Standalone controller state (declared up here for the Arduino prototype
// generator, like StallGuard below)
enum BalState { BAL_HANG, BAL_CHECK, BAL_SWING, BAL_BALANCE, BAL_STEP, BAL_DONE, BAL_BRAKE };
BalState balState = BAL_HANG;

// Declared up here (not next to stallCheck) because the Arduino IDE inserts
// auto-generated function prototypes before the first function in the file.
struct StallGuard {
  int16_t       ref_pos;
  unsigned long ref_time;
  int           kicks;
  int16_t       kick_pos;   // position right after the last kick
};

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

void clearHomingDone() {
  digitalWrite(HOMING_DONE_PIN, LOW);
}

void setHomingDone() {
  digitalWrite(HOMING_DONE_PIN, HIGH);
}

bool leftLimitHit()  { return digitalRead(LEFT_LIMIT)  == LOW; }
bool rightLimitHit() { return digitalRead(RIGHT_LIMIT) == LOW; }
bool anyLimitHit()   { return leftLimitHit() || rightLimitHit(); }

// Limit in the direction of travel, and the one we are moving away from
int homingPwm(Dir d) { return d == DIR_LEFT ? HOMING_PWM_LEFT : HOMING_PWM_RIGHT; }

bool limitAhead(Dir d)  { return d == DIR_LEFT ? leftLimitHit()  : rightLimitHit(); }
bool limitBehind(Dir d) { return d == DIR_LEFT ? rightLimitHit() : leftLimitHit(); }

// =============================================================================
// Trigger helpers — unify button and Pi GPIO inputs
// =============================================================================
bool startTriggered() {
  return (digitalRead(START_BTN) == LOW) || (digitalRead(PI_START_PIN) == HIGH);
}

bool stopTriggered() {
  // STOP_BTN is normally closed wired to GND:
  //   resting = closed = LOW = not stopped
  //   pressed = open   = HIGH (pulled up) = stopped
  return (digitalRead(STOP_BTN) == HIGH) || (digitalRead(PI_STOP_PIN) == HIGH);
}

// =============================================================================
// UART read — parse framed packets from ESP32 #1
// Packet: 0xFF | m[6:0] | m[13:7] | m[15:14] | p[6:0] | p[13:7] | p[15:14]
// Data bytes are 7-bit, so 0xFF only ever appears as the header.
// =============================================================================
void readUART() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (b == 0xFF) {
      uart_idx = 0;
    } else if (b & 0x80) {
      uart_idx = UART_DATA_BYTES;   // corrupt byte — drop until next header
    } else if (uart_idx < UART_DATA_BYTES) {
      uart_buf[uart_idx++] = b;
      if (uart_idx == UART_DATA_BYTES) {
        cart_position = (int16_t)(uart_buf[0] | (uart_buf[1] << 7) | (uart_buf[2] << 14));
        int16_t p = (int16_t)(uart_buf[3] | (uart_buf[4] << 7) | (uart_buf[5] << 14));
        // int16 difference handles the 16-bit wrap; accumulate into 32 bits
        if (pend_seen) pend_position += (int16_t)(p - pend_raw);
        else           { pend_position = p; pend_seen = true; }
        pend_raw  = p;
        newSample = true;
      }
    }
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
  int16_t start = cart_position;
  unsigned long t0 = millis();
  drive(d, BREAKAWAY_PWM);
  while (millis() - t0 < BREAKAWAY_MS) {
    readUART();
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
  Serial.printf("Stall driving %s at PWM %d (pos %d) — breakaway kick %d/%d\n",
                dirName(d), pwm, cart_position, g.kicks, MAX_KICKS);
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
//   5. Send 0xAA to ESP32 #1 on success
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
    readUART();

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
  readUART();
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
    readUART();

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
    Dir d = error > 0 ? DIR_LEFT : DIR_RIGHT;
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
  int16_t ref = cart_position;
  unsigned long refTime = millis();
  unsigned long t0 = millis();
  while (millis() - t0 < SETTLE_TIMEOUT_MS) {
    readUART();
    if (abs(cart_position - ref) > SETTLE_COUNTS) {
      ref = cart_position;
      refTime = millis();
    } else if (millis() - refTime >= SETTLE_MS) {
      break;
    }
    delay(5);
  }
  Serial.printf("Settled at count %d (center %d, off by %d)\n",
                cart_position, track_center, cart_position - track_center);
}

// Approach center, let the cart stop, and re-approach if it coasted too far.
bool centerCart() {
  Serial.println("Homing: driving to center...");
  for (int attempt = 1; attempt <= CENTER_ATTEMPTS; attempt++) {
    if (!approachCenter()) return false;
    waitForSettle();
    int off = cart_position - track_center;
    if (abs(off) <= CENTER_ACCEPT) return true;
    if (attempt < CENTER_ATTEMPTS) {
      Serial.printf("Overshot center by %d — correcting (attempt %d/%d)\n",
                    off, attempt + 1, CENTER_ATTEMPTS);
    }
  }
  Serial.printf("WARNING: still %d counts from center after %d attempts — accepting\n",
                cart_position - track_center, CENTER_ATTEMPTS);
  return true;
}

bool doHoming() {
  if (!homeToLimit(DIR_LEFT)) return false;
  int16_t left_pos = cart_position;
  Serial.print("Left limit at count: "); Serial.println(left_pos);

  if (!homeToLimit(DIR_RIGHT)) return false;
  int16_t right_pos = cart_position;
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

  // Notify ESP32 #1 — it will set FLAG_HOMING_COMPLETE in SPI status byte to Pi
  // and zero both counts. 4 bytes, not 1: noise on the line made single 0xAA
  // bytes and zeroed the counts mid-run.
  static const uint8_t HOMING_MAGIC[4] = { 0xAA, 0x55, 0xA5, 0x5A };
  Serial2.write(HOMING_MAGIC, sizeof(HOMING_MAGIC));

  return true;
}

// Full homing sequence plus enabling run mode on success.
void runHoming() {
  clearHomingDone();  // clear while homing in progress
  systemEnabled = false;
  signalArmed   = false;

  homingComplete = doHoming();
  if (!homingComplete) {
    stopMotor();
    Serial.println("ERROR: Homing failed. Press START to retry.");
    return;
  }

  // Cart is centered, so no limit should be pressed. Give switches a moment
  // to settle; if one still reads pressed, something is wrong.
  unsigned long t0 = millis();
  while (anyLimitHit() && millis() - t0 < 2000) { readUART(); delay(10); }
  if (anyLimitHit()) {
    homingComplete = false;
    Serial.println("ERROR: Limit switch still pressed after centering. Press START to retry.");
    return;
  }
  delay(300);  // additional debounce settle time

  setHomingDone();  // signal Pi that homing is complete
  systemEnabled = true;
#if STANDALONE_SWINGUP
  standaloneStart();
#else
  Serial.println("System enabled. Waiting for 2.5V neutral on DAC before driving...");
#endif
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
int16_t       lastCart = 0;
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
  // Drain packets buffered during homing's delays, so the window starts from
  // the current count and not from before ESP32 #1 zeroed it on 0xAA
  readUART();
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
  balTrim    = 0;
  balanceGains();
  Serial.println("Standalone: waiting for the pendulum to hang still...");
}

void enterState(BalState s) {
  balState = s;
  const char* names[] = { "HANG", "CHECK", "SWING", "BALANCE", "STEP TEST", "DONE", "BRAKE" };
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
  xPos   = cartDir * cart_position * CART_M_PER_COUNT;   // ESP32 #1 zeroed at centre
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
      Serial.printf("%s: cart at %.0f mm, %.2f m/s — braking. Press START to re-home.\n",
                    fault, xPos * 1000.0f, xDot);
      clearHomingDone();
      brakeStart = millis();
      enterState(BAL_BRAKE);
    }

    switch (balState) {
      case BAL_BRAKE:
        // Powered stop, then disable. Timeout in case the loop can't stop it.
        if (fabsf(xDot) < BRAKE_DONE_V || millis() - brakeStart > BRAKE_MS) {
          stopMotor();
          uOut = vRefOut = 0;
          systemEnabled = false;
          Serial.printf("Stopped at %.0f mm.\n", xPos * 1000.0f);
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
        if (fabsf(theta) < CATCH_ANGLE && fabsf(thetaDot) < CATCH_RATE) {
          enterState(BAL_BALANCE);
          balanceStart();
          speedDrive(balanceV());
        } else {
          speedDrive(swingV());
        }
        break;
      case BAL_BALANCE:
        if (fabsf(theta) > DROP_ANGLE) {
          enterState(BAL_SWING);
          speedDrive(swingV());
        } else {
          speedDrive(balanceV());
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
  Serial.println("=== ESP32 #2 Motor Control v" FW_VERSION " ===");

  // --- Homing done pin — drive LOW at boot ---
  pinMode(HOMING_DONE_PIN, OUTPUT);
  clearHomingDone();

  // Now attach LEDC PWM (25kHz, 8-bit, above audible range)
  ledcAttach(RPWM_PIN, 25000, 8);
  ledcAttach(LPWM_PIN, 25000, 8);
  stopMotor();   // redundant but explicit

  // --- GPIO setup ---
  pinMode(START_BTN,    INPUT_PULLUP);
  pinMode(STOP_BTN,     INPUT_PULLUP);
  pinMode(LEFT_LIMIT,   INPUT_PULLUP);
  pinMode(RIGHT_LIMIT,  INPUT_PULLUP);
  pinMode(PI_START_PIN, INPUT_PULLDOWN);  // Pi drives HIGH to trigger
  pinMode(PI_STOP_PIN,  INPUT_PULLDOWN);  // Pi drives HIGH to trigger

  // --- UART2 ---
  // 7-byte packets every 2ms = 3.5 KB/s. The default 256-byte buffer overflows
  // during homing's blocking delays and would leave a stale cart position.
  Serial2.setRxBufferSize(8192);
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  // --- I2C + ADS1115 ---
  Wire.begin(21, 22);
  if (ads.begin()) {
    ads.setGain(GAIN_ONE);  // ±4.096V range — covers 0–4.096V of the DAC output
  } else {
#if STANDALONE_SWINGUP
    // Only the DAC-follow mode reads the ADS1115
    Serial.println("WARNING: ADS1115 not found — fine in standalone mode.");
#else
    Serial.println("ERROR: ADS1115 not found! Check wiring. Halting.");
    while (1) stopMotor();
#endif
  }

  Serial.println("Ready. Waiting for START (button or Pi GPIO 4) to home...");

  // --- Wait for start trigger (button OR Pi GPIO) ---
  while (!startTriggered()) {
    readUART();
    delay(10);
  }
  Serial.println(digitalRead(PI_START_PIN) == HIGH ? "START from Pi" : "START from button");
  delay(200);  // debounce

  runHoming();
}

// =============================================================================
// Loop — normal motor control from ADS1115 DAC voltage
// =============================================================================
void loop() {
  readUART();

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
    systemEnabled = false;
    Serial.printf("LIMIT HIT (%s) — motor stopped. Press START to re-home.\n",
                  leftLimitHit() ? "left" : "right");
  }

  // --- Stop trigger (button or Pi GPIO) ---
  if (stopTriggered() && systemEnabled) {
    stopMotor();
    clearHomingDone();  // clear signal to Pi
    systemEnabled = false;
    Serial.println("STOP triggered — motor stopped. Press START to re-home.");
    delay(200);
    return;
  }

  // --- Not enabled: hold motor stopped, re-home on START ---
  if (!systemEnabled) {
    stopMotor();
    if (startTriggered()) {
      delay(200);  // debounce
      runHoming();
    } else {
      delay(10);
    }
    return;
  }

#if STANDALONE_SWINGUP
  // --- Standalone: swing-up + balance, one control step per UART packet ---
  standaloneStep();
  return;
#endif

  // --- Normal operation: read DAC voltage and drive motor ---
  int16_t raw       = ads.readADC_SingleEnded(0);
  float   voltage   = ads.computeVolts(raw);
  float   deviation = voltage - NEUTRAL_VOLTAGE;

  // --- DAC signal guard ---
  if (voltage < SIGNAL_LOST_V) {
    if (!signalLow) {
      signalLow = true;
      signalLowSince = millis();
    } else if (signalArmed && millis() - signalLowSince >= SIGNAL_LOST_MS) {
      signalArmed = false;
      Serial.println("DAC signal lost (~0V) — motor stopped until 2.5V neutral is seen.");
    }
  } else {
    signalLow = false;
  }

  if (!signalArmed) {
    stopMotor();
    if (fabsf(deviation) < DEADBAND) {
      signalArmed = true;
      Serial.println("DAC at neutral — following DAC.");
    }
    delay(10);
    return;
  }

  if (fabsf(deviation) < DEADBAND) {
    stopMotor();
  } else if (deviation > 0) {
    int pwm = constrain(
      map((int)(deviation * 1000),
          (int)(DEADBAND * 1000),
          (int)(MAX_INPUT * 1000),
          0, MAX_PWM),
      0, MAX_PWM
    );
    driveRight(pwm);
  } else {
    int pwm = constrain(
      map((int)(fabsf(deviation) * 1000),
          (int)(DEADBAND * 1000),
          (int)(MAX_INPUT * 1000),
          0, MAX_PWM),
      0, MAX_PWM
    );
    driveLeft(pwm);
  }

  delay(10);
}
