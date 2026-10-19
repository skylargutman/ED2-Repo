#!/bin/bash
# Copy the generated code for SwingHoldPendulumPiMod2 to the Pi and compile it there.
set -e
K=/c/Users/super/.ssh; S="ssh -F $K/config -i $K/id_ed25519 -o UserKnownHostsFile=$K/known_hosts -o BatchMode=yes sciencelabtoyou.com"
SRC=/c/Users/super/Documents/ED2; R=$SRC/SwingHoldPendulumPiMod2_ert_rtw
rm -rf /tmp/bundle && mkdir -p /tmp/bundle
cp $SRC/SwingHoldPendulumPiMod2_pkg.zip $R/SwingHoldPendulumPiMod2.mk $R/codertarget_assembly_flags.mk $R/MW_target_hardware_resources.h /tmp/bundle/
cp "/c/Users/super/Documents/ED2/ED2-Repo/MATLAB/Feedback Inverted Pendulum/RealTime Models/UART/MW_MQTT_Parameters.h" /tmp/bundle/
tar -C /tmp/bundle -cf - . | timeout 120 $S "ssh -o BatchMode=yes pendulum 'rm -rf /tmp/bundle && mkdir -p /tmp/bundle && tar -C /tmp/bundle -xf -'"
timeout 400 $S "timeout 390 ssh -o BatchMode=yes pendulum 'bash -s'" <<'PI'
set -e
WS=~/picontrol/MATLAB_ws/R2026b; SD=$WS/C/Users/super/Documents/ED2; ML=$WS/C/Program_Files/MATLAB
mkdir -p $SD $ML; cd /tmp/bundle && rm -rf x && mkdir x && cd x && unzip -q ../SwingHoldPendulumPiMod2_pkg.zip
rm -rf $SD/SwingHoldPendulumPiMod2_ert_rtw
unzip -qo sDirFiles.zip -d $SD; unzip -qo mlrFiles.zip -d $ML
cp /tmp/bundle/*.mk /tmp/bundle/MW_*.h $SD/SwingHoldPendulumPiMod2_ert_rtw/
cd $SD/SwingHoldPendulumPiMod2_ert_rtw && touch rtw_proj.tmw && rm -f ../SwingHoldPendulumPiMod2.elf
# Patch: the generated base-rate loop computes stopRequested but only tests runModel, so Stop blocks never end the program
python3 - <<'PYPATCH'
p = "ert_main.c"; s = open(p).read()
key = "stopRequested = !((rtmGetErrorStatus"
i = s.index(key); j = s.index(";", i) + 1
if "patched: honour Stop blocks" not in s:
    s = s[:j] + chr(10) + "    if (stopRequested) runModel = 0;   /* patched: honour Stop blocks */" + s[j:]
open(p, "w").write(s); print("ert_main patched")
PYPATCH
make -f SwingHoldPendulumPiMod2.mk MATLAB_WORKSPACE=$WS 'LINUX_TARGET_LIBS_MACRO=-lmwraspiperipheral -lgpiod -lpaho-mqtt3as -ljson-c' -j4 > build.log 2>&1 \
  && echo BUILD-OK || { echo BUILD-FAILED; grep -iE "error|undefined" build.log | head -15; exit 1; }
ls -l --time-style=+%H:%M:%S ../SwingHoldPendulumPiMod2.elf
PI
