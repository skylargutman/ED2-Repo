#!/usr/bin/env python3
"""Measure the cart's speed for the same motor voltage driving right vs left (run ON THE PI).

Uses the Simulink UART link (picontrol/scripts/pilink.py, uart_link_check.py) exactly like a model:
start pulse -> ESP32 homes and opens a Simulink run -> 1 kHz lockstep commands. Set the ESP32's
FRIC_POS_V / FRIC_NEG_V to 0 first so the commanded volts reach the motor unchanged.

For each level V: +V for HALF s, -V for HALF s, then a position hold back to center. The steady
speed is the slope of x over the second half of each phase. Fitting speed = k * (V - V0) per
direction gives the kinetic threshold V0 the friction offset should cancel.
MOVES THE CART. Safety: any |x| > XMAX switches to 0 V; the ESP32 soft limit still applies.
Usage: sudo python3 pi_motor_symmetry.py [levels...]
"""
import sys, time, json
sys.path.insert(0, "/home/owlsley/picontrol/scripts")
import serial
from pilink import encode_command, MeasurementParser
import uart_link_check as ulc

M_PER_COUNT = 0.156 / 2048      # CartDataReader: counts -> metres
RIGHT_SIGN = +1                 # the packet's cart count is already + to the right (2026-11-20: -1 made the hold run away)
HALF, XMAX, RATE = 0.4, 0.25, 1000.0
# Usage: [--fric POS NEG] levels...   --fric: test WITH these offsets (kept afterwards); default raw (0/0, restored to 0.3/0.3)
args = sys.argv[1:]; FRIC = None
if args[:1] == ["--fric"]: FRIC = (float(args[1]), float(args[2])); args = args[3:]
levels = [float(a) for a in args] or [0.70, 0.85, 1.00, 1.20]

usb = serial.Serial("/dev/ttyUSB0", 921600, timeout=0.2)   # plain open: does not reset the ESP32
def console(cmd, wait=0.6):
    usb.write((cmd + "\n").encode()); t = time.monotonic(); out = b""
    while time.monotonic() - t < wait: out += usb.read(65536)
    return [l for l in out.decode(errors="replace").splitlines() if l and not l.startswith("D ")]
def set_fric(p, n):
    console("exp select Friction")
    print(console(f"set FRIC_POS_V {p}") + console(f"set FRIC_NEG_V {n}"), flush=True)
    console("exp select Simulink")
set_fric(*(FRIC or (0, 0)))      # raw volts, or the offsets under test
ser = serial.Serial("/dev/ttyAMA0", 230400, timeout=0.0005); ser.reset_input_buffer()
print(console("run", 0.3), flush=True)   # homes, then opens the Simulink run (the daemon holds the GPIO lines)
parser = MeasurementParser(); q = []
m = ulc.wait_for_ready(ser, parser, 60)
if m is None: print("no ready"); sys.exit(1)
c0 = m.cart
x_of = lambda c: RIGHT_SIGN * (c - c0) * M_PER_COUNT
log = []                         # (t, u, x)
t0 = time.monotonic(); n = 0; tripped = False

def step(u):
    """One lockstep command; returns x (m) or None if the run ended."""
    global n, tripped
    n += 1; ulc._wait_until(t0 + n / RATE)
    x_prev = log[-1][2] if log else 0.0
    if abs(x_prev) > XMAX: u = 0.0; tripped = True
    ser.write(encode_command(u))
    meas = ulc._next_measurement(ser, parser, q, time.monotonic() + 0.005)
    if meas is None: return x_prev
    if not meas.ready: return None
    x = x_of(meas.cart); log.append((time.monotonic() - t0, u, x)); return x

def hold_center(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        x = log[-1][2] if log else 0.0
        if step(max(-1.0, min(1.0, -4.0 * x))) is None: return False   # P hold, +u = right
    return True

def phase(u):
    start = len(log); end = time.monotonic() + HALF
    while time.monotonic() < end:
        if step(u) is None: return None
    seg = log[start:]; seg = seg[len(seg) // 2:]
    if len(seg) < 20: return None
    ts = [s[0] for s in seg]; xs = [s[2] for s in seg]
    mt, mx = sum(ts) / len(ts), sum(xs) / len(xs)
    return sum((a - mt) * (b - mx) for a, b in zip(ts, xs)) / sum((a - mt) ** 2 for a in ts)

results = []
ok = hold_center(1.0)
for V in levels:
    if not ok: break
    vr = phase(+V); vl = phase(-V)
    if vr is None or vl is None: ok = False; break
    results.append({"V": V, "v_right": round(vr, 4), "v_left": round(-vl, 4), "tripped": tripped})
    print(f"V {V:.2f}: right {vr:+.3f} m/s   left {-vl:+.3f} m/s   x now {log[-1][2]:+.3f} m", flush=True)
    ok = hold_center(1.5)
for _ in range(100): step(0.0)
time.sleep(0.3)                  # no more commands: the ESP32 ends the run after 50 ms
print([l for l in console("status", 0.5)], flush=True)
if FRIC is None: set_fric(0.3, 0.3)   # raw test: back to the pre-test values
print("RESULTS " + json.dumps(results))
