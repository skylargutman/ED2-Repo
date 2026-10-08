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
