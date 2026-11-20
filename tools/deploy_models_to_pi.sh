#!/bin/bash
# Build the ported UART Simulink models on the Pi and point the website's experiment names at them.
# Usage (from the repo root): bash tools/deploy_models_to_pi.sh [Exp ...]     (default: all 11 ported models)
#
# The website sends an experiment name (e.g. CartControl); dac_daemon runs
#   /home/owlsley/picontrol/models/MATLAB_ws/R2025b/<Exp>.elf
# build_pi.sh compiles <Exp>PiMod2 into ~/picontrol/MATLAB_ws/R2026b/.../<Exp>PiMod2.elf.
# This script links <Exp>.elf -> that build. An old DAC-era <Exp>.elf is kept as <Exp>DAC.elf.
# Building never moves the cart; only a run started from the website does.
set -u
MODELS=("$@")
[ ${#MODELS[@]} -eq 0 ] && MODELS=(CartControl CartIdent CraneIdent CraneStab InvPendIdent PendStabPD PendSwingUp PendulumFriction PendulumTest SwingHoldPendulumExtra UpDownPendulum)
K=${SSH_DIR:-$HOME/.ssh}; S="ssh -F $K/config -i $K/id_ed25519 -o UserKnownHostsFile=$K/known_hosts -o BatchMode=yes sciencelabtoyou.com"
UART="$(cd "$(dirname "$0")/.." && pwd)/MATLAB/Feedback Inverted Pendulum/RealTime Models/UART"

if ! timeout 40 $S "timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=10 pendulum true"; then
  echo "Pi not reachable through sciencelabtoyou.com (reverse tunnel on port 2223 is down)."; exit 2
fi

declare -A RESULT
for e in "${MODELS[@]}"; do
  echo "=================== $e"
  if ! (cd "$UART" && bash build_pi.sh "${e}PiMod2"); then RESULT[$e]="BUILD FAILED"; continue; fi
  out=$(timeout 60 $S "timeout 50 ssh -o BatchMode=yes pendulum 'E=$e bash -s'" <<'PI'
set -e
MD=~/picontrol/models/MATLAB_ws/R2025b
NEW=~/picontrol/MATLAB_ws/R2026b/C/Users/super/Documents/ED2/${E}PiMod2.elf
[ -x "$NEW" ] || { echo "no elf at $NEW"; exit 1; }
mkdir -p $MD; cd $MD
if [ -e "$E.elf" ] && [ ! -L "$E.elf" ]; then
  if [ -e "${E}DAC.elf" ]; then mv "$E.elf" "$E.elf.old.$(date +%Y%m%d%H%M%S)"; else mv "$E.elf" "${E}DAC.elf"; fi
fi
ln -sfn "$NEW" "$E.elf"
echo "linked $(ls -l $E.elf | awk '{print $9,$10,$11}')"
PI
) && RESULT[$e]="OK: $out" || RESULT[$e]="LINK FAILED: $out"
done
echo; echo "=================== summary"
for e in "${MODELS[@]}"; do echo "$e: ${RESULT[$e]}"; done
