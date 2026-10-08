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
