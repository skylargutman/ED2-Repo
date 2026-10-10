// =============================================================================
// Experiment 2: Static friction (Feedback PendulumFriction, Ex. 3). Ramps the
// raw motor command up from 0 at FR_RAMP_VPS until the cart has moved
// FR_MOVE_COUNTS (~1 mm) IN THE DRIVE DIRECTION, first driving right, then
// driving left. Before each ramp the cart must be still (|v| < FR_STILL_V for
// FR_SETTLE_MS), so a cart that is still coasting, or is nudged, can't be
// mistaken for a breakaway. The two breakaway commands are reported as results
// and become the friction compensation of motorVolts() (saved in flash).
// Failures end the run as "run end error" with a WARNING, and leave the stored
// values unchanged: the cart doesn't move by 2.5 V (motor power off, cart
// blocked), won't stay still, or a value is implausibly low (< FR_MIN_V).
// =============================================================================
#define FR_STILL_V          0.005f   // m/s: "still" before a ramp
#define FR_STILL_TIMEOUT_MS 10000    // give up if the cart never stays still
#define FR_MIN_V            0.1f     // V: a lower breakaway is not believable

int           frPhase   = 0;     // 0 settle, 1 ramp right, 2 settle, 3 ramp left
float         frU       = 0;     // current raw command magnitude (V)
float         frPosV    = 0;     // measured breakaway driving right (V)
int32_t       frStartCount = 0;
unsigned long frPhaseMs = 0;     // when the current phase started
unsigned long frStillMs = 0;     // when the cart was last seen moving (settle phases)

// End the run as a failure: motor off, WARNING (shown in the console), stored
// values untouched
bool frFail(const char* why) {
  stopMotor();
  uOut = 0;
  Serial.printf("WARNING: Friction test failed: %s Stored values unchanged.\n", why);
  expEndReason = "error";
  return false;
}

void frStart() {
  frPhase   = 0;
  frU       = 0;
  frPhaseMs = frStillMs = millis();
  stopMotor();
  announce("FRICTION");
  Serial.printf("Friction: waiting for the cart to be still, then ramping right at %.2f V/s...\n",
                FR_RAMP_VPS);
}

// Clamp a measurement to the stored range; returns the value to store
float frClamp(float v, const char* dir) {
  if (v > FRIC_MAX_V) {
    Serial.printf("WARNING: breakaway driving %s was %.3f V, above %.1f V: stored as %.1f V. "
                  "Check the belt tension.\n", dir, v, FRIC_MAX_V, FRIC_MAX_V);
    return FRIC_MAX_V;
  }
  return v;
}

bool frStep(float dt) {
  if (frPhase == 0 || frPhase == 2) {                 // settle: motor off until still
    stopMotor();
    uOut = 0;
    if (fabsf(xDot) >= FR_STILL_V) frStillMs = millis();
    if (millis() - frPhaseMs > FR_STILL_TIMEOUT_MS)
      return frFail("the cart would not stay still before the ramp.");
    if (millis() - frStillMs >= FR_SETTLE_MS) {
      frPhase     += 1;
      frU          = 0;
      frStartCount = cart_position;
      Serial.printf("Friction: ramping %s...\n", frPhase == 1 ? "right" : "left");
    }
  } else {                                            // ramp until it moves
    int dir = frPhase == 1 ? 1 : -1;
    frU += FR_RAMP_VPS * dt;
    if (frU > U_FULL_SCALE_V)
      return frFail(dir > 0 ? "the cart did not move driving right up to 2.5 V (motor power off or cart blocked?)."
                            : "the cart did not move driving left up to 2.5 V (motor power off or cart blocked?).");
    motorRawVolts(dir * frU);
    // Signed travel in the drive direction (cartDir maps "right" to the count's sign)
    int32_t moved = dir * cartDir * (cart_position - frStartCount);
    if (moved >= FR_MOVE_COUNTS) {
      stopMotor();
      Serial.printf("result friction_%s_V %.3f\n", dir > 0 ? "pos" : "neg", frU);
      if (dir > 0) {
        frPosV    = frU;
        frPhase   = 2;
        frPhaseMs = frStillMs = millis();
      } else {
        if (frPosV < FR_MIN_V || frU < FR_MIN_V)
          return frFail("a breakaway below 0.1 V is not believable (was the cart pushed?).");
        FRIC_POS_V = frClamp(frPosV, "right");
        FRIC_NEG_V = frClamp(frU, "left");
        saveFriction();
        Serial.printf("Friction compensation saved: +%.3f V / -%.3f V\n", FRIC_POS_V, FRIC_NEG_V);
        return false;                                 // "run end done"
      }
    }
  }
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.printf("FRICTION phase %d  u %5.3f V  moved %ld counts  v %6.3f  x %6.1f mm\n",
                  frPhase, frU, (long)(cart_position - frStartCount), xDot, xPos * 1000.0f);
  }
  return true;
}

const char* frState() { return "FRICTION"; }

Experiment EXP_FRICTION = { "Friction", "Static friction (Ex. 3)", false, true, frStart, frStep, frState };
