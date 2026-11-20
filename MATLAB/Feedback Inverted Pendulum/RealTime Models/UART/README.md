# UART versions of the Feedback models (single-board ESP32 rig)

The Simulink model runs on the Raspberry Pi 5 and talks to the ESP32 over UART
(`/dev/ttyAMA0`, **230400** baud, the most the Raspberry Pi Blockset Serial
blocks offer). Design: `docs/superpowers/specs/2026-10-10-simulink-uart-design.md`.

- `CartDataReader.slx`: Serial Read (12 bytes per 1 ms step) -> `decode_packet`
  (A5 5A, cart int32, pend int32, flags, XOR). It keeps the last valid packet when
  a read is short or misaligned. Then cart -> metres (0.156/2048 m per count) and
  pendulum -> radians (1968 counts/rev, hanging = pi).
- `CartDataWriter.slx`: `u` (volts) -> gain +1 (since fix 4: model +x = ESP32 + = right)
  -> clamp to +-2.5 V -> `encode_command` (A5 5A + float32) -> Serial Write. The
  ESP32's `motorVolts()` maps the volts to PWM and adds the friction
  compensation, so the model must NOT add a 2.5 V offset or its own friction
  compensation.
- `SwingHoldPendulumPiMod2.slx`: swing up and hold, using the two subsystems
  above. Saved in R2026b.

`fix_models_2026_10_19.m` is the script that turned Alex's first UART version
into these files. It removes the old DAC conventions (the 2.5 V offset and the
0..5 V clamp), the double friction compensation and the GPIO 6 reset; it sets
1968 counts/rev and holds the last valid packet. Run it in the folder with the
original files. The originals are backed up next to them.

Building for the Pi needs **Simulink Coder + Embedded Coder** (`ert.tlc`) on
top of the Raspberry Pi Blockset.

## Fixes since 2026-10-19

Each `fix_models_N_*.m` backs up what it changes into `C:\Users\super\Documents\ED2\backup_<date>_before_fixes_N\`
and works on the copies in `C:\Users\super\Documents\ED2`, where MATLAB builds; synced copies are here.

| Script | Change |
|---|---|
| `fix_models_3_2026_10_26.m` | SwingHold: +-0.3 rad/s hysteresis relay on the swing-up velocity sign; kick -0.3 V |
| `fix_models_4_2026_10_26.m` | Sign fix: the port mirrored the cart but not the pendulum. Reader `Counts->Meters = +0.156/2048`, writer gain +1 |
| `fix_models_5_2026_10_26.m` | SwingHold angle PID back to Feedback's P=25, I=0.2, D=1.5 |
| `fix_models_6_2026_10_30.m` | Telemetry "control" now taken after the stabilization/swing-up switch (it showed the balance controller's output even while swinging up) |
| `port_to_uart.m` (2026-11-06) | Ports the other 11 PiMod2 models: reader/writer subsystems, one-sprintf telemetry, Stop after StopTime |
| `fix_models_7_2026_11_20.m` | Reader's angle First Order Hold starts at **pi** (hanging), not 0 (= upright): PendSwingUp's upright-stop fired on step 1 |
| `fix_models_8_2026_11_20.m` | SwingHoldPendulumExtra swing-up push 0.15 -> 0.5 V (0.15 V stayed below breakaway) |

`build_pi.sh` also patches the generated code on the Pi: `ert_main.c` honours Stop blocks, and the
UART Serial Write runs only in major time steps (models with continuous states rerun their first
step as a minor step, which sent a second command for one reply).

Models regenerated with fix 7 so far: PendSwingUp, SwingHoldPendulumExtra. Still on the 2026-11-06
code: CraneStab, InvPendIdent, UpDownPendulum (continuous states, so they also want the UART-write
guard), and the rest. ESP32 friction offsets for Simulink mode: `FRIC_POS_V 0.39 / FRIC_NEG_V 0.30`
(2026-11-20, from `tools/pi_motor_symmetry.py`; right needs ~0.15 V more to start moving).
