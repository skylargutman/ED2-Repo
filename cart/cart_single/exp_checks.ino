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
