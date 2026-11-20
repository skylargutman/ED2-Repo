// =============================================================================
// Motor command in volts (Feedback convention, +-U_FULL_SCALE_V = full scale)
// and the friction compensation stored in flash.
//   motorRawVolts(u): no compensation, PWM = |u| / 2.5 V * SPEED_MAX_PWM. Used
//                     by the Friction experiment to find the breakaway point.
//   motorVolts(u):    compensated. Output jumps to the breakaway command of the
//                     direction, then rises linearly to full scale at 2.5 V:
//                     v = off + |u| * (2.5 - off) / 2.5.
//                     Example, off = 0.6 V: u = 0.1 V -> 0.676 V -> PWM 49;
//                     u = 2.5 V -> 2.5 V -> PWM 180.
// =============================================================================
Preferences frictionPrefs;

static int voltsToPwm(float v) {
  return constrain((int)(v / U_FULL_SCALE_V * SPEED_MAX_PWM + 0.5f), 0, SPEED_MAX_PWM);
}

void motorRawVolts(float u) {
  u = constrain(u, -U_FULL_SCALE_V, U_FULL_SCALE_V);
  uOut = u / U_FULL_SCALE_V;
  int pwm = voltsToPwm(fabsf(u));
  if (pwm == 0)   stopMotor();
  else if (u > 0) driveRight(pwm);
  else            driveLeft(pwm);
}

void motorVolts(float u) {
  u = constrain(u, -U_FULL_SCALE_V, U_FULL_SCALE_V);
  uOut = u / U_FULL_SCALE_V;
  if (fabsf(u) < U_EPS * U_FULL_SCALE_V) { stopMotor(); return; }
  float off = u > 0 ? FRIC_POS_V : FRIC_NEG_V;
  float v   = off + fabsf(u) * (U_FULL_SCALE_V - off) / U_FULL_SCALE_V;
  int   pwm = voltsToPwm(v);
  if (u > 0) driveRight(pwm);
  else       driveLeft(pwm);
}

static bool fricValid(float v) { return v >= 0.0f && v <= FRIC_MAX_V; }   // false for NaN

// Boot: take the stored values if present and in range, else keep the defaults
void loadFriction() {
  frictionPrefs.begin("friction", true);                 // read-only
  float p = frictionPrefs.getFloat("pos", NAN);
  float n = frictionPrefs.getFloat("neg", NAN);
  frictionPrefs.end();
  if (fricValid(p) && fricValid(n)) {
    FRIC_POS_V = p;
    FRIC_NEG_V = n;
    Serial.printf("Friction compensation from flash: +%.3f V / -%.3f V\n", p, n);
  } else {
    Serial.printf("Friction compensation: defaults +%.3f V / -%.3f V (not measured yet)\n",
                  FRIC_POS_V, FRIC_NEG_V);
  }
}

void saveFriction() {
  frictionPrefs.begin("friction", false);
  frictionPrefs.putFloat("pos", FRIC_POS_V);
  frictionPrefs.putFloat("neg", FRIC_NEG_V);
  frictionPrefs.end();
}

void clearFriction() {
  frictionPrefs.begin("friction", false);
  frictionPrefs.clear();
  frictionPrefs.end();
}
