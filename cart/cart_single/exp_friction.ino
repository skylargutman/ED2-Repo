// =============================================================================
// Experiment 2: Static friction (Feedback PendulumFriction, Ex. 3). Ramps the
// raw motor command up from 0 at FR_RAMP_VPS until the cart has moved
// FR_MOVE_COUNTS (~1 mm), first driving right, then (after the cart settles)
// driving left. The two breakaway commands are reported as results and become
// the friction compensation of motorVolts() (saved in flash).
// If the cart doesn't move by 2.5 V (motor power off, cart blocked) the run
// ends and the stored values are left unchanged.
// =============================================================================
int           frPhase   = 0;     // 0 ramp right, 1 settle, 2 ramp left, 3 store
float         frU       = 0;     // current raw command magnitude (V)
float         frPosV    = 0;     // measured breakaway driving right (V)
int32_t       frStartCount = 0;
unsigned long frPhaseMs = 0;

void frStart() {
  frPhase      = 0;
  frU          = 0;
  frStartCount = cart_position;
  stopMotor();
  announce("FRICTION");
  Serial.printf("Friction: ramping right at %.2f V/s until the cart moves...\n", FR_RAMP_VPS);
}

// Clamp to the stored range and save. Returns the value actually stored.
float frStore(float v, const char* dir) {
  if (v > FRIC_MAX_V) {
    Serial.printf("WARNING: breakaway driving %s was %.3f V, above %.1f V: stored as %.1f V. "
                  "Check the belt tension.\n", dir, v, FRIC_MAX_V, FRIC_MAX_V);
    return FRIC_MAX_V;
  }
  return v;
}

bool frStep(float dt) {
  if (frPhase == 0 || frPhase == 2) {
    int dir = frPhase == 0 ? 1 : -1;
    frU += FR_RAMP_VPS * dt;
    if (frU > U_FULL_SCALE_V) {
      stopMotor();
      uOut = 0;
      Serial.printf("Friction: the cart did not move driving %s up to %.1f V "
                    "(motor power off or cart blocked?). Stored values unchanged.\n",
                    dir > 0 ? "right" : "left", U_FULL_SCALE_V);
      return false;
    }
    motorRawVolts(dir * frU);
    if (abs(cart_position - frStartCount) >= FR_MOVE_COUNTS) {
      stopMotor();
      Serial.printf("result friction_%s_V %.3f\n", dir > 0 ? "pos" : "neg", frU);
      if (dir > 0) frPosV = frU;
      frPhase  += 1;
      frPhaseMs = millis();
      if (frPhase == 3) {
        FRIC_POS_V = frStore(frPosV, "right");
        FRIC_NEG_V = frStore(frU, "left");
        saveFriction();
        Serial.printf("Friction compensation saved: +%.3f V / -%.3f V\n", FRIC_POS_V, FRIC_NEG_V);
        return false;
      }
    }
  } else if (frPhase == 1) {
    stopMotor();
    uOut = 0;
    if (millis() - frPhaseMs >= FR_SETTLE_MS) {
      frPhase      = 2;
      frU          = 0;
      frStartCount = cart_position;
      Serial.println("Friction: ramping left...");
    }
  }
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.printf("FRICTION phase %d  u %5.3f V  moved %ld counts  x %6.1f mm\n",
                  frPhase, frU, (long)abs(cart_position - frStartCount), xPos * 1000.0f);
  }
  return true;
}

const char* frState() { return "FRICTION"; }

Experiment EXP_FRICTION = { "Friction", "Static friction (Ex. 3)", false, true, frStart, frStep, frState };
