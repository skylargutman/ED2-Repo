#!/bin/bash
# Copy the generated code for a model to the Pi and compile it there.
# Usage: bash build_pi.sh [MODEL]   (default SwingHoldPendulumPiMod2; generate its code first)
set -e
MDL=${1:-SwingHoldPendulumPiMod2}
K=/c/Users/super/.ssh; S="ssh -F $K/config -i $K/id_ed25519 -o UserKnownHostsFile=$K/known_hosts -o BatchMode=yes sciencelabtoyou.com"
SRC=/c/Users/super/Documents/ED2; R=$SRC/${MDL}_ert_rtw
rm -rf /tmp/bundle && mkdir -p /tmp/bundle
cp $SRC/${MDL}_pkg.zip $R/${MDL}.mk $R/codertarget_assembly_flags.mk $R/MW_target_hardware_resources.h /tmp/bundle/
cp "/c/Users/super/Documents/ED2/ED2-Repo/MATLAB/Feedback Inverted Pendulum/RealTime Models/UART/MW_MQTT_Parameters.h" /tmp/bundle/
tar -C /tmp/bundle -cf - . | timeout 120 $S "ssh -o BatchMode=yes pendulum 'rm -rf /tmp/bundle && mkdir -p /tmp/bundle && tar -C /tmp/bundle -xf -'"
timeout 400 $S "timeout 390 ssh -o BatchMode=yes pendulum 'MDL=$MDL bash -s'" <<'PI'
set -e
WS=~/picontrol/MATLAB_ws/R2026b; SD=$WS/C/Users/super/Documents/ED2; ML=$WS/C/Program_Files/MATLAB
mkdir -p $SD $ML; cd /tmp/bundle && rm -rf x && mkdir x && cd x && unzip -q ../${MDL}_pkg.zip
rm -rf $SD/${MDL}_ert_rtw
unzip -qo sDirFiles.zip -d $SD; unzip -qo mlrFiles.zip -d $ML
cp /tmp/bundle/*.mk /tmp/bundle/MW_*.h $SD/${MDL}_ert_rtw/
cd $SD/${MDL}_ert_rtw && touch rtw_proj.tmw && rm -f ../${MDL}.elf
# Patch: the generated base-rate loop computes stopRequested but only tests runModel, so Stop blocks never end the program
python3 - <<'PYPATCH'
p = "ert_main.c"; s = open(p).read()
key = "stopRequested = !((rtmGetErrorStatus"
i = s.index(key); j = s.index(";", i) + 1
if "patched: honour Stop blocks" not in s:
    s = s[:j] + chr(10) + "    if (stopRequested) runModel = 0;   /* patched: honour Stop blocks */" + s[j:]
open(p, "w").write(s); print("ert_main patched")
PYPATCH
# Patch: send the UART command only in major time steps. Models with continuous states (TransferFcn,
# Derivative) rerun the whole base-rate step as a minor step when a First Order Hold flags its output
# inconsistent (always on the first step). The UART Read is generated inside a major-step guard but the
# UART Write is not, so the rerun sent a second command for one reply and broke the lockstep with the
# ESP32: PendSwingUp sent 2 commands and then blocked forever (2026-11-20).
MDL=$MDL python3 - <<'PYPATCH'
import os, re
mdl = os.environ["MDL"]; p = mdl + ".c"; s = open(p).read()
if "patched: UART write in major steps only" not in s:
    s, n = re.subn(r"(\n([ \t]*)MW_SCI_Transmit\(([^;]*);)",
                   lambda m: "\n%sif (rtmIsMajorTimeStep(%s_M)) {   /* patched: UART write in major steps only */%s\n%s}" % (m.group(2), mdl, m.group(1).replace("\n", "\n  "), m.group(2)),
                   s)
    open(p, "w").write(s); print("UART write guarded:", n)
PYPATCH
make -f ${MDL}.mk MATLAB_WORKSPACE=$WS 'LINUX_TARGET_LIBS_MACRO=-lmwraspiperipheral -lgpiod -lpaho-mqtt3as -ljson-c' -j4 > build.log 2>&1 \
  && echo BUILD-OK || { echo BUILD-FAILED; grep -iE "error|undefined" build.log | head -15; exit 1; }
ls -l --time-style=+%H:%M:%S ../${MDL}.elf
PI
