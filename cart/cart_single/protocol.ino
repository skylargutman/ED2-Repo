// =============================================================================
// USB serial command interface (for the bridge / website)
// One command per line. Every reply starts with "ok ", "err ", "param ",
// "params end" or "status ", so it can be told apart from the telemetry lines.
//   home | start        start homing + swing-up (only while idle)
//   stop                stop now (always accepted, also during homing)
//   status              -> status IDLE | HOMING | HANG | SWING | BALANCE | ...
//   params              -> param NAME VALUE MIN MAX DEFAULT UNIT | description
//   set NAME VALUE      change a setting (only while idle, within its limits)
//   defaults            restore every setting to its default (only while idle)
//   io                  -> io cart N pend N left 0|1 right 0|1 start 0|1 stop 0|1
//                          pins cartA cartB pendA pendB (encoder pin levels)
//                          (counts, switch states and encoder levels, for bring-up)
// =============================================================================
bool isIdle() { return !systemEnabled && !homingActive; }

const char* stateName() {
  if (homingActive)   return "HOMING";
  if (!systemEnabled) return "IDLE";
  return BAL_NAMES[balState];
}


void captureParamDefaults() {
  for (int i = 0; i < N_PARAMS; i++) PARAMS[i].def = *PARAMS[i].value;
}

int findParam(const char* name) {
  for (int i = 0; i < N_PARAMS; i++)
    if (strcasecmp(PARAMS[i].name, name) == 0) return i;
  return -1;
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    const Param& p = PARAMS[i];
    Serial.printf("param %s %g %g %g %g %s | %s\n",
                  p.name, *p.value, p.lo, p.hi, p.def, p.unit, p.desc);
  }
  Serial.println("params end");
}

// One line with every setting, printed when a run starts so the log records it
void printRunParams() {
  Serial.print("run params:");
  for (int i = 0; i < N_PARAMS; i++)
    Serial.printf(" %s=%g", PARAMS[i].name, *PARAMS[i].value);
  Serial.println();
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "stop")) {
    serialStop = true;
    Serial.println("ok stop");
  } else if (!strcasecmp(cmd, "home") || !strcasecmp(cmd, "start")) {
    if (!isIdle()) { Serial.printf("err home busy (%s)\n", stateName()); return; }
    serialStart = true;
    Serial.println("ok home");
  } else if (!strcasecmp(cmd, "status")) {
    Serial.printf("status %s\n", stateName());
  } else if (!strcasecmp(cmd, "params")) {
    printParams();
  } else if (!strcasecmp(cmd, "defaults")) {
    if (!isIdle()) { Serial.printf("err defaults busy (%s)\n", stateName()); return; }
    for (int i = 0; i < N_PARAMS; i++) *PARAMS[i].value = PARAMS[i].def;
    Serial.println("ok defaults");
  } else if (!strcasecmp(cmd, "set")) {
    char* name = strtok(NULL, " \t");
    char* val  = strtok(NULL, " \t");
    if (!name || !val) { Serial.println("err set usage: set NAME VALUE"); return; }
    int i = findParam(name);
    if (i < 0) { Serial.printf("err %s unknown setting\n", name); return; }
    if (!isIdle()) { Serial.printf("err %s busy (%s): change settings while idle\n", PARAMS[i].name, stateName()); return; }
    char* end;
    float f = strtof(val, &end);
    if (end == val || *end) { Serial.printf("err %s not a number\n", PARAMS[i].name); return; }
    if (!(f >= PARAMS[i].lo && f <= PARAMS[i].hi)) {   // also rejects NaN
      Serial.printf("err %s range %g..%g\n", PARAMS[i].name, PARAMS[i].lo, PARAMS[i].hi);
      return;
    }
    *PARAMS[i].value = f;
    Serial.printf("ok %s %g\n", PARAMS[i].name, f);
  } else if (!strcasecmp(cmd, "io")) {
    readEncoders();
    // Raw encoder pin levels too: if a level toggles while its count stays put,
    // the fault is in the counting; if it never toggles, it's the wiring.
    Serial.printf("io cart %ld pend %ld left %d right %d start %d stop %d"
                  "  pins cartA %d cartB %d pendA %d pendB %d\n",
                  (long)cart_position, (long)pend_position, leftLimitHit(), rightLimitHit(),
                  digitalRead(START_BTN) == LOW, digitalRead(STOP_BTN) == HIGH,
                  digitalRead(CART_ENC_A), digitalRead(CART_ENC_B),
                  digitalRead(PEND_ENC_A), digitalRead(PEND_ENC_B));
  } else {
    Serial.printf("err %s unknown command (home, stop, status, params, set, defaults, io)\n", cmd);
  }
}

// Non-blocking: call often, including inside homing loops so "stop" works there
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      handleCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// STOP check used inside the homing routine. Homing used to ignore STOP for its
// whole ~10 s, which matters once a website can start the rig.
bool homingStopped() {
  pollSerial();
  if (!stopTriggered()) return false;
  stopMotor();
  Serial.println("STOP during homing — aborted.");
  return true;
}
