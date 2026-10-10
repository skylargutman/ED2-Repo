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
// It is measured from the hanging reference only, without PEND_TRIM_DEG: the
// trim describes where upright balances, not where the pendulum hangs, so
// hanging reads exactly pi and the balanced pendulum reads about +0.096 rad.
void printDataLine() {
  const float k = 2.0f * PI / PEND_COUNTS_PER_REV;
  float thCont = PEND_SIGN * (pend_position - pendDownRef) * k + PI;
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
