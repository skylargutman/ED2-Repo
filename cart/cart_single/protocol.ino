// =============================================================================
// USB serial command interface (for the bridge / website)
// One command per line. Replies start with "ok ", "err ", "param ",
// "params end", "status ", "exp " or "io ", so they can be told apart from
// telemetry ("D ", "SWING", "BAL", "HANG", ...).
//   home | start | run  home, then run the selected experiment (only while idle)
//   stop                stop now (always accepted, also during homing)
//   status              -> status STATE EXPERIMENT
//   exp list            -> exp NAME | title  (one per experiment), then exp end
//   exp select NAME     experiment for the next run (only while idle)
//   params              -> param NAME VALUE MIN MAX DEFAULT UNIT | description
//                          (shared settings + the selected experiment's)
//   set NAME VALUE      change a setting (only while idle, within its limits)
//   defaults            restore every setting to its default (only while idle)
//   io                  -> io cart N pend N left 0|1 right 0|1 start 0|1 stop 0|1
//                          pins cartA cartB pendA pendB (encoder pin levels)
// =============================================================================
bool isIdle() { return !systemEnabled && !homingActive; }

const char* stateName() {
  if (homingActive)      return "HOMING";
  if (!systemEnabled)    return "IDLE";
  if (phase == PH_HANG)  return "HANG";
  if (phase == PH_BRAKE) return "BRAKE";
  if (phase == PH_EXP)   return curExp->state();
  return "IDLE";
}

// A setting is visible when it is shared or belongs to the selected experiment
bool paramVisible(int i) {
  return PARAMS[i].exp == NULL || strcmp(PARAMS[i].exp, curExp->name) == 0;
}

void captureParamDefaults() {
  for (int i = 0; i < N_PARAMS; i++) PARAMS[i].def = *PARAMS[i].value;
}

int findParam(const char* name) {
  for (int i = 0; i < N_PARAMS; i++)
    if (paramVisible(i) && strcasecmp(PARAMS[i].name, name) == 0) return i;
  return -1;
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    if (!paramVisible(i)) continue;
    const Param& p = PARAMS[i];
    Serial.printf("param %s %g %g %g %g %s | %s\n",
                  p.name, *p.value, p.lo, p.hi, p.def, p.unit, p.desc);
  }
  Serial.println("params end");
}

// "run start NAME K=V ..." with every setting the run uses, so each saved
// run records what produced it
void printRunStart() {
  Serial.printf("run start %s", curExp->name);
  for (int i = 0; i < N_PARAMS; i++)
    if (paramVisible(i)) Serial.printf(" %s=%g", PARAMS[i].name, *PARAMS[i].value);
  Serial.println();
}

int findExperiment(const char* name) {
  for (int i = 0; i < N_EXPS; i++)
    if (strcasecmp(EXPS[i]->name, name) == 0) return i;
  return -1;
}

// "exp list" / "exp select NAME" (strtok already consumed "exp")
void handleExpCommand() {
  char* sub  = strtok(NULL, " \t");
  char* name = strtok(NULL, " \t");
  if (sub && !strcasecmp(sub, "list")) {
    for (int i = 0; i < N_EXPS; i++) Serial.printf("exp %s | %s\n", EXPS[i]->name, EXPS[i]->title);
    Serial.println("exp end");
  } else if (sub && !strcasecmp(sub, "select") && name) {
    if (!isIdle()) { Serial.printf("err exp busy (%s)\n", stateName()); return; }
    int i = findExperiment(name);
    if (i < 0) { Serial.printf("err exp %s unknown experiment\n", name); return; }
    curExp = EXPS[i];
    Serial.printf("ok exp %s\n", curExp->name);
  } else {
    Serial.println("err exp usage: exp list | exp select NAME");
  }
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "stop")) {
    serialStop = true;
    Serial.println("ok stop");
  } else if (!strcasecmp(cmd, "home") || !strcasecmp(cmd, "start") || !strcasecmp(cmd, "run")) {
    if (!isIdle()) { Serial.printf("err home busy (%s)\n", stateName()); return; }
    serialStart = true;
    Serial.println("ok home");
  } else if (!strcasecmp(cmd, "status")) {
    Serial.printf("status %s %s\n", stateName(), curExp->name);
  } else if (!strcasecmp(cmd, "exp")) {
    handleExpCommand();
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
    // Raw encoder pin levels too: if a level toggles while its count stays put,
    // the fault is in the counting; if it never toggles, it's the wiring.
    readEncoders();
    Serial.printf("io cart %ld pend %ld left %d right %d start %d stop %d"
                  "  pins cartA %d cartB %d pendA %d pendB %d\n",
                  (long)cart_position, (long)pend_position, leftLimitHit(), rightLimitHit(),
                  digitalRead(START_BTN) == LOW, digitalRead(STOP_BTN) == HIGH,
                  digitalRead(CART_ENC_A), digitalRead(CART_ENC_B),
                  digitalRead(PEND_ENC_A), digitalRead(PEND_ENC_B));
  } else {
    Serial.printf("err %s unknown command (home, run, stop, status, exp, params, set, defaults, io)\n", cmd);
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
