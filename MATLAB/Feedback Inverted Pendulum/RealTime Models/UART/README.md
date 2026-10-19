# UART versions of the Feedback models (single-board ESP32 rig)

The Simulink model runs on the Raspberry Pi 5 and talks to the ESP32 over UART
(`/dev/ttyAMA0`, **230400** baud, the most the Raspberry Pi Blockset Serial
blocks offer). Design: `docs/superpowers/specs/2026-10-10-simulink-uart-design.md`.

- `CartDataReader.slx`: Serial Read (12 bytes per 1 ms step) -> `decode_packet`
  (A5 5A, cart int32, pend int32, flags, XOR). It keeps the last valid packet when
  a read is short or misaligned. Then cart -> metres (0.156/2048 m per count) and
  pendulum -> radians (1968 counts/rev, hanging = pi).
- `CartDataWriter.slx`: `u` (volts) -> x(-1) (model +x = left, ESP32 + = right)
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
