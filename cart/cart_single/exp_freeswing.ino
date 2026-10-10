// =============================================================================
// Experiment 1: Pendulum free swing (Feedback PendulumTest). Motor off. The
// student lifts the pendulum, lets go, and the run records it swinging down
// for FS_DURATION_S seconds (damping and natural frequency from the .mat).
// =============================================================================
unsigned long fsStartMs = 0;

void fsStart() {
  stopMotor();
  uOut = vRefOut = 0;
  fsStartMs = millis();
  announce("FREE");
  Serial.println("Free swing: lift the pendulum and let it go. Recording...");
}

bool fsStep(float dt) {
  stopMotor();
  uOut = vRefOut = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.printf("FREE th %7.1f deg  w %6.2f  x %6.1f mm  E %5.2f\n",
                  theta * RAD_TO_DEG, thetaDot, xPos * 1000.0f, energy);
  }
  return millis() - fsStartMs < (unsigned long)(FS_DURATION_S * 1000.0f);
}

const char* fsState() { return "FREE"; }

// Motor off the whole time: no software soft limit (the cart can be pushed by hand)
Experiment EXP_FREESWING = { "FreeSwing", "Pendulum free swing (Ex. 1)", false, false, fsStart, fsStep, fsState };
