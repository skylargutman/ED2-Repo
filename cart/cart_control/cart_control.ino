// =============================================================================
// ESP32 #2 — Motor Control Firmware
// Version: 9.5
// Description: Controls BTS7960 IBT-2 H-bridge motor driver.
//              Holds motor outputs LOW at boot to prevent runaway on power-up.
//              Waits for START button OR Pi GPIO trigger before homing.
//              Runs homing routine, sends 0xAA to ESP32 #1 on completion.
//              Drives GPIO 18 HIGH when homing complete — Pi reads this to know
//              when to launch Simulink model.
//              Pi GPIO stop signal mirrors physical STOP button behavior.
//              Normal operation reads DAC voltage from ADS1115 and drives motor.
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
// UART Protocol (receive from ESP32 #1):
//   3-byte framed packet: 0xFF | pos_low | pos_high  → int16 cart encoder count
//
// UART Protocol (send to ESP32 #1):
//   Single byte 0xAA = homing complete
// =============================================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

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
#define HOMING_PWM        45
#define CENTER_TOLERANCE  20     // counts — within this = at center (~1.5mm)
#define MIN_TRACK_COUNTS  100    // sanity check — reject if track shorter than this

// --- State ---
bool systemEnabled  = false;
bool homingComplete = false;

// --- Cart position received from ESP32 #1 via UART ---
volatile int16_t cart_position = 0;
int16_t track_total  = 0;
int16_t track_center = 0;

// --- UART receive buffer ---
uint8_t uart_buf[2];
int     uart_idx = 0;

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

void clearHomingDone() {
  digitalWrite(HOMING_DONE_PIN, LOW);
}

void setHomingDone() {
  digitalWrite(HOMING_DONE_PIN, HIGH);
}

bool leftLimitHit()  { return digitalRead(LEFT_LIMIT)  == LOW; }
bool rightLimitHit() { return digitalRead(RIGHT_LIMIT) == LOW; }

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
// Packet: 0xFF | pos_low | pos_high
// =============================================================================
void readUART() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (b == 0xFF) {
      uart_idx = 0;
    } else if (uart_idx < 2) {
      uart_buf[uart_idx++] = b;
      if (uart_idx == 2) {
        cart_position = (int16_t)(uart_buf[0] | (uart_buf[1] << 8));
      }
    }
  }
}

// =============================================================================
// Homing routine
// Steps:
//   1. Drive left to left limit
//   2. Drive right to right limit (wait for left limit release first)
//   3. Calculate center from total track counts
//   4. Drive to center (wait for right limit release first, proportional speed)
//   5. Send 0xAA to ESP32 #1 on success
// Returns true on success, false on any error.
// =============================================================================
bool doHoming() {
  Serial.println("Homing: driving left...");

  // Step 1 — drive left until left limit hit
  driveLeft(HOMING_PWM);
  // If starting on right limit, wait for it to release before checking as error
  while (rightLimitHit()) {
    readUART();
    delay(5);
  }
  Serial.println("Right limit cleared, driving to left...");
  while (!leftLimitHit()) {
    readUART();
    if (rightLimitHit()) {
      stopMotor();
      Serial.println("ERROR: Right limit hit during step 1");
      return false;
    }
    delay(5);
  }
  stopMotor();
  delay(200);
  int16_t left_pos = cart_position;
  Serial.print("Left limit at count: "); Serial.println(left_pos);

  // Step 2 — drive right, wait for left limit to release, then continue to right limit
  Serial.println("Homing: driving right...");
  driveRight(HOMING_PWM);
  while (leftLimitHit()) {           // wait for release before checking safety
    readUART();
    delay(5);
  }
  Serial.println("Left limit released, continuing right...");
  while (!rightLimitHit()) {
    readUART();
    if (leftLimitHit()) {
      stopMotor();
      Serial.println("ERROR: Left limit hit during step 2");
      return false;
    }
    delay(5);
  }
  stopMotor();
  delay(200);
  int16_t right_pos = cart_position;
  Serial.print("Right limit at count: "); Serial.println(right_pos);

  // Step 3 — calculate center
  track_total  = abs(right_pos - left_pos);
  track_center = left_pos + (track_total / 2);
  Serial.print("Track total counts: "); Serial.println(track_total);
  Serial.print("Center count: ");       Serial.println(track_center);

  if (track_total < MIN_TRACK_COUNTS) {
    Serial.println("ERROR: Track too short — check encoder wiring");
    return false;
  }

  // Step 4 — start moving left, wait for right limit to release, then center
  Serial.println("Homing: driving to center...");
  driveLeft(HOMING_PWM);
  while (rightLimitHit()) {          // wait for release before checking position
    readUART();
    delay(5);
  }
  Serial.println("Right limit released, centering...");

  while (true) {
    readUART();
    int16_t error = cart_position - track_center;

    if (abs(error) < CENTER_TOLERANCE) {
      stopMotor();
      break;
    }

    // Proportional speed — slow down as we approach center
    int pwm = constrain(
      map(abs(error), CENTER_TOLERANCE, 500, 20, HOMING_PWM),
      20, HOMING_PWM
    );

    if (error > 0) driveLeft(pwm);
    else           driveRight(pwm);

    if (leftLimitHit() || rightLimitHit()) {
      stopMotor();
      Serial.println("ERROR: Limit hit during centering");
      return false;
    }
    delay(5);
  }

  Serial.println("Homing complete!");

  // Notify ESP32 #1 — it will set FLAG_HOMING_COMPLETE in SPI status byte to Pi
  Serial2.write(0xAA);

  return true;
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  // --- Hold motor outputs LOW immediately — prevents runaway on boot ---
  // Do this before ledcAttach so pins are driven low, not floating
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);

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
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  // --- I2C + ADS1115 ---
  Wire.begin(21, 22);
  if (!ads.begin()) {
    Serial.println("ERROR: ADS1115 not found! Check wiring. Halting.");
    while (1) stopMotor();
  }
  ads.setGain(GAIN_ONE);  // ±4.096V range — covers 0–5V DAC output

  Serial.println("ESP32 #2 v9.5 ready. Waiting for START to home...");

  // --- Wait for start trigger (button OR Pi GPIO) ---
  while (!startTriggered()) {
    readUART();
    delay(10);
  }
  delay(200);  // debounce

  // --- Run homing ---
  homingComplete = doHoming();
  if (homingComplete) {
    // Wait for both limit switches to fully release and settle before
    // enabling the loop — prevents false limit trigger from switch bounce
    while (leftLimitHit() || rightLimitHit()) {
      delay(10);
    }
    delay(300);  // additional debounce settle time
    setHomingDone();  // signal Pi that homing is complete
    systemEnabled = true;
    Serial.println("System enabled. Running.");
  } else {
    Serial.println("ERROR: Homing failed. Press START to retry.");
  }
}

// =============================================================================
// Loop — normal motor control from ADS1115 DAC voltage
// =============================================================================
void loop() {
  readUART();

  // --- Hardware safety: limit switches override everything (except during homing) ---
  if (leftLimitHit() || rightLimitHit()) {
    if (systemEnabled) {
      stopMotor();
      systemEnabled = false;
      Serial.println("LIMIT HIT — motor stopped. Press START to re-home.");
    }
    // Allow start trigger to re-home even from limit switch position
    if (startTriggered()) {
      delay(200);  // debounce
      clearHomingDone();  // clear while homing in progress
      homingComplete = doHoming();
      if (homingComplete) {
        while (leftLimitHit() || rightLimitHit()) { delay(10); }
        delay(300);
        setHomingDone();  // signal Pi homing complete
        systemEnabled = true;
        Serial.println("Re-homed. System enabled.");
      } else {
        Serial.println("ERROR: Homing failed. Press START to retry.");
      }
      return;
    }
    delay(50);
    return;
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

  // --- Start trigger when not enabled: re-home ---
  if (startTriggered() && !systemEnabled) {
    delay(200);  // debounce
    clearHomingDone();  // clear while homing in progress
    homingComplete = doHoming();
    if (homingComplete) {
      while (leftLimitHit() || rightLimitHit()) { delay(10); }
      delay(300);
      setHomingDone();  // signal Pi homing complete
      systemEnabled = true;
      Serial.println("Re-homed. System enabled.");
    }
    return;
  }

  // --- Not enabled: hold motor stopped ---
  if (!systemEnabled) {
    stopMotor();
    return;
  }

  // --- Normal operation: read DAC voltage and drive motor ---
  int16_t raw      = ads.readADC_SingleEnded(0);
  float   voltage  = ads.computeVolts(raw);
  float   deviation = voltage - NEUTRAL_VOLTAGE;

  if (abs(deviation) < DEADBAND) {
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
      map((int)(abs(deviation) * 1000),
          (int)(DEADBAND * 1000),
          (int)(MAX_INPUT * 1000),
          0, MAX_PWM),
      0, MAX_PWM
    );
    driveLeft(pwm);
  }

  delay(10);
}
