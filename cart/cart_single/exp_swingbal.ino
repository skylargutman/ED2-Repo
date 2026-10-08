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
