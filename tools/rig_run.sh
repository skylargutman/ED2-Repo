#!/bin/bash
# Start one experiment exactly as the website does (MQTT pendulum/cmd), follow the Pi's daemon log,
# and count the telemetry the website receives. Sends Stop if the run outlasts MAXSEC.
# Usage: bash tools/rig_run.sh EXPERIMENT [MAXSEC=100]     MOVES THE CART: someone must be at the rig.
EXP=$1; MAXSEC=${2:-100}
K=${SSH_DIR:-$HOME/.ssh}   # config, id_ed25519, known_hosts for sciencelabtoyou.com
ssh -F $K/config -i $K/id_ed25519 -o UserKnownHostsFile=$K/known_hosts -o BatchMode=yes sciencelabtoyou.com "EXP=$EXP MAXSEC=$MAXSEC bash -s" <<'EOF'
L=/tmp/run_$EXP.journal; T=/tmp/run_$EXP.telemetry
timeout $((MAXSEC+30)) mosquitto_sub -h localhost -p 1885 -t raspi/to_django > $T 2>/dev/null & SUB=$!
timeout $((MAXSEC+25)) ssh -o BatchMode=yes pendulum "journalctl -u dac_daemon -f -n 0 --no-pager -o cat" > $L 2>&1 & JRN=$!
# ESP32 USB log for the run. Plain open: Linux raises DTR+RTS together, which does not reset the ESP32
# (setting dtr/rts False first does). Data lines "D ..." are dropped; everything else is kept.
E=/tmp/run_$EXP.esp32
timeout $((MAXSEC+22)) ssh -o BatchMode=yes pendulum "timeout $((MAXSEC+20)) python3 -u -c '
import serial, sys
s = serial.Serial(\"/dev/ttyUSB0\", 921600, timeout=0.2)
buf = b\"\"
while True:
    buf += s.read(65536)
    *lines, buf = buf.split(b\"\n\")
    for l in lines:
        if not l.startswith(b\"D \"): sys.stdout.write(l.decode(errors=\"replace\") + \"\n\")
'" > $E 2>&1 & ESP=$!
sleep 2
mosquitto_pub -h localhost -p 1885 -t pendulum/cmd -m "{\"command\":\"sta\",\"experiment\":\"$EXP\",\"parameters\":{}}"
t0=$(date +%s); result=TIMEOUT
while [ $(( $(date +%s) - t0 )) -lt $MAXSEC ]; do
  sleep 1
  if grep -q -E "Experiment stopped/crashed|Model not found|Failed to launch|Homing timed out" $L; then result=ENDED; break; fi
done
if [ $result = TIMEOUT ]; then mosquitto_pub -h localhost -p 1885 -t pendulum/cmd -m '{"command":"sto"}'; sleep 3; fi
sleep 2; kill $SUB $JRN $ESP 2>/dev/null; ssh -n -o BatchMode=yes pendulum "pkill -f '[p]ython3 -u -c'" 2>/dev/null; wait 2>/dev/null   # [p]: never matches its own command line
echo "##### $EXP: $result after $(( $(date +%s) - t0 ))s"
grep -E "Homing complete|Launched|Experiment running|stopped/crashed|not found|Failed|ERROR|WARNING|timeout|Stop" $L | sed 's/^.*INFO: //;s/^.*\[DAC DAEMON\] //'
echo "ESP32 ($(wc -l < $E) lines):"; grep -v "^D " $E | sort | uniq -c | sort -rn | awk '$1>3{print "  x"$0; next}{print}' | head -25 | sed 's/^ *//'
python3 - $T <<'PY'
import sys, json
rows = []
for l in open(sys.argv[1]):
    l = l.strip()
    if not l: continue
    try:
        txt = bytes(int(b) for b in l.split()).decode() if l[0].isdigit() else l
        rows.append({k: float(v) for k, v in json.loads(txt).items()})
    except Exception: pass
if not rows: print("telemetry: NONE received"); sys.exit()
t=[r.get("time",0) for r in rows]; x=[r.get("position",0) for r in rows]; a=[r.get("angle",0) for r in rows]; u=[r.get("control",0) for r in rows]
print(f"telemetry: {len(rows)} msgs, t {t[0]:.1f}..{t[-1]:.1f}s | x {min(x):+.3f}..{max(x):+.3f} m (end {x[-1]:+.3f}) | angle {min(a):+.2f}..{max(a):+.2f} | u {min(u):+.2f}..{max(u):+.2f} V")
fr = next((i for i in range(len(x)-1, 0, -1) if (x[i], a[i]) != (x[i-1], a[i-1])), 0)
if len(x) - fr > 20 and max(abs(v) for v in u[fr:]) > 0.05:
    print(f"  WARNING: position/angle frozen for the last {len(x)-fr} samples ({(len(x)-fr)*0.1:.1f}s) while the model still commands up to {max(abs(v) for v in u[fr:]):.2f} V: ESP32 stopped the motor (check the ESP32 lines above)")
elif len(x) - fr > 20:
    print(f"  note: nothing moved for the last {(len(x)-fr)*0.1:.1f}s and the command was ~0 V (model idle)")
PY
EOF
