// =============================================================================
// Simulink: the Raspberry Pi runs the controller (spec 2026-10-10).
// Lockstep over UART2 (pi_link.ino): simService() runs on every loop() pass,
// applies each command from the Pi with motorVolts(u) at once and answers
// with a fresh measurement. The Simulink models run a 1 ms step, faster than
// the 2 ms control step, so this can't wait for runStep(). Until the first
// command a measurement goes out every SIM_IDLE_TX_MS, so the model starts
// with valid counts. simStep() (2 ms) only ends the run on a timeout; the
// safety checks stay in control.ino / loop(), and once they brake or stop,
// commands are ignored and not answered.
// A Pi start pulse selects this experiment (setup() / loop()). GPIO 18 tells
// the Pi the run is open: HIGH from simStart() until the run ends.
// =============================================================================
extern Experiment EXP_SIMULINK;

unsigned long simStartMs    = 0;
unsigned long simLastCmdMs  = 0;
unsigned long simLastIdleTx = 0;
bool          simGotCmd     = false;

bool simulinkSelected() { return curExp == &EXP_SIMULINK; }
bool simRunOpen()       { return simulinkSelected() && phase == PH_EXP; }

void selectSimulink() {
  if (simulinkSelected()) return;
  curExp = &EXP_SIMULINK;
  Serial.println("ok exp Simulink");   // the bridge follows the selection
}

uint8_t simFlags() {
  uint8_t f = 0;
  if (simRunOpen())                         f |= PI_FLAG_READY;
  if (digitalRead(HOMING_DONE_PIN) == HIGH) f |= PI_FLAG_HOMED;
  if (simulinkSelected())                   f |= PI_FLAG_SIMULINK;
  return f;
}

void simStart() {
  stopMotor();
  uOut = vRefOut = 0;
  piLinkReset();                 // nothing from before this run is applied
  simStartMs = simLastIdleTx = millis();
  simGotCmd  = false;
  piReadyLine(true);
  announce("SIMWAIT");
  Serial.println("Simulink: waiting for commands from the Pi on UART2...");
  piLinkSend(simFlags());
}

// Every loop() pass. Outside an open Simulink run, Pi bytes are discarded.
void simService() {
  if (!simRunOpen()) {
    while (Serial2.available()) Serial2.read();
    return;
  }
  float u;
  if (piLinkRead(&u)) {
    motorVolts(u);
    readEncoders();
    piLinkSend(simFlags());
    simLastCmdMs = millis();
    if (!simGotCmd) {
      simGotCmd = true;
      announce("SIM");
    }
  } else if (!simGotCmd && millis() - simLastIdleTx >= SIM_IDLE_TX_MS) {
    simLastIdleTx = millis();
    piLinkSend(simFlags());
  }
}

bool simStep(float dt) {
  unsigned long now = millis();
  if (!simGotCmd) {
    stopMotor();
    uOut = vRefOut = 0;
    if (now - simStartMs > SIM_FIRST_TIMEOUT_MS) {
      Serial.println("Simulink: no command from the Pi within 30 s (is the model running?)");
      expEndReason = "timeout";
      return false;
    }
    return true;
  }
  if ((long)(now - simLastCmdMs) > SIM_TIMEOUT_MS) {
    Serial.printf("Simulink: no command for %lu ms, stopping.\n", now - simLastCmdMs);
    expEndReason = "timeout";
    return false;
  }
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("SIM u %6.3f V  th %7.1f deg  x %6.1f mm  cmds %lu  drops %lu\n",
                  uOut * U_FULL_SCALE_V, theta * RAD_TO_DEG, xPos * 1000.0f,
                  (unsigned long)piLinkCommands(), (unsigned long)piLinkDrops());
  }
  return true;
}

const char* simState() { return simGotCmd ? "SIM" : "SIMWAIT"; }

// Called by endRun() before "run end": the link counts become run results
void simFinish() {
  Serial.printf("result pi_commands %lu\n", (unsigned long)piLinkCommands());
  Serial.printf("result pi_drops %lu\n", (unsigned long)piLinkDrops());
}

// Safety on (soft limit, overspeed); no automatic re-home: the Pi decides
Experiment EXP_SIMULINK = { "Simulink", "Simulink on the Raspberry Pi", false, true, simStart, simStep, simState };
