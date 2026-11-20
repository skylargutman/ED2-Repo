# Building a UART model for the Pi without a direct connection

The Raspberry Pi Blockset normally builds over SSH to the Pi (the model's
`DeviceAddress`, 192.168.64.20). When this PC can't reach that address,
generate the code here and compile it on the Pi:

1. In MATLAB (R2026b, with Simulink Coder + Embedded Coder + Raspberry Pi Blockset):
   ```matlab
   load_system('SwingHoldPendulumPiMod2'); set_param(gcs,'GenCodeOnly','on');   % don't save this
   slbuild(gcs);
   bi = load('SwingHoldPendulumPiMod2_ert_rtw/buildInfo.mat');
   packNGo(bi.buildInfo,'packType','hierarchical','fileName','SwingHoldPendulumPiMod2_pkg.zip');
   ```
2. Copy these files to the Pi: the zip, `SwingHoldPendulumPiMod2_ert_rtw/SwingHoldPendulumPiMod2.mk`,
   `codertarget_assembly_flags.mk`, `MW_target_hardware_resources.h`, and
   `MW_MQTT_Parameters.h` (this folder).
3. On the Pi (`WS=~/picontrol/MATLAB_ws/R2026b`; the makefile expects the PC's
   paths mirrored under `$WS/C/...`):
   ```bash
   SD=$WS/C/Users/<you>/Documents/ED2; ML=$WS/C/Program_Files/MATLAB
   unzip pkg.zip; unzip -o sDirFiles.zip -d $SD; unzip -o mlrFiles.zip -d $ML
   cp *.mk MW_*.h $SD/SwingHoldPendulumPiMod2_ert_rtw/
   cd $SD/SwingHoldPendulumPiMod2_ert_rtw && touch rtw_proj.tmw
   make -f SwingHoldPendulumPiMod2.mk MATLAB_WORKSPACE=$WS \
        'LINUX_TARGET_LIBS_MACRO=-lmwraspiperipheral -lgpiod -lpaho-mqtt3as -ljson-c'
   ```
   The result is `$SD/SwingHoldPendulumPiMod2.elf`.

`build_pi.sh` does steps 2-3 from this PC through sciencelabtoyou.com, including the patch below.

**Patch on every build:** the generated `ert_main.c` computes `stopRequested` but its base-rate loop only
tests `runModel`, so Stop blocks (the 40 s stop, "Stop if cart moves too far") never end the program.
`build_pi.sh` adds `if (stopRequested) runModel = 0;` after that line. Verified 2026-10-19: the model exits
after 40.02 s.

**MQTT payload:** the R2026b Blockset publishes a text message as decimal byte codes ("123 34 ...").
The website decodes them (`WebInterface/telemetry_payload.py`).

Notes:
- `MW_MQTT_Parameters.h` is normally written by the Blockset's own remote build.
  Everything in it is optional. The broker comes from `-D` flags in the makefile,
  and the anonymous broker needs no user/password. With no client ID, the code
  uses `<hostname>_<random>`.
- `MQTTsample` (0.1 s) is in the model workspace.
- The Pi's peripheral library is the R2025b server (`mw_server_v25.1.0`); it
  linked fine against R2026b code (2026-10-19).
