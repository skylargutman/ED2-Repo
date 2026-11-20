#!/usr/bin/env python3
"""Check the Pi <-> ESP32 UART link (Simulink mode) without Simulink.

Run on the Pi, from the picontrol folder:
  python3 uart_link_check.py                      # u = 0 for 10 s
  python3 uart_link_check.py --u-amp 0.8          # +-0.8 V square wave, 0.5 s each way
  python3 uart_link_check.py --garble 100         # every 100th command is NaN (must be dropped)
  python3 uart_link_check.py --gpio               # pulse start first and stop at the end
  python3 uart_link_check.py --blast --seconds 20  # write commands without waiting (stale-byte check)

Without --gpio, start a Simulink run another way: console "exp select Simulink"
then Run, or the START button with Simulink selected. With --gpio, stop
dac_daemon.py first (it owns the start/stop lines).
When the check ends without --gpio, the ESP32 ends the run itself with
"run end timeout" 50 ms after the last command. That is expected.
"""
import argparse
import statistics
import time
from dataclasses import dataclass, field

from pilink import MEAS_LEN, MeasurementParser, encode_command

GPIO_HOME_START = 16      # Pi GPIO -> ESP32 GPIO 4 (start pulse)
GPIO_STOP = 20            # Pi GPIO -> ESP32 GPIO 5 (stop pulse)
PULSE_S = 0.2


@dataclass
class LinkStats:
    commands: int = 0
    replies: int = 0
    rtts: list = field(default_factory=list)
    bad_checksums: int = 0
    last: object = None
    end: str = ""
    missed_replies: int = 0


def _next_measurement(ser, parser, queue, deadline):
    while not queue:
        if time.monotonic() > deadline:
            return None
        queue.extend(parser.feed(ser.read(MEAS_LEN)))   # returns as soon as one packet is in
    return queue.pop(0)


def wait_for_ready(ser, parser, timeout_s):
    """Read the ESP32's 100 ms idle packets until one says a Simulink run is open."""
    queue = []
    deadline = time.monotonic() + timeout_s
    while True:
        m = _next_measurement(ser, parser, queue, deadline)
        if m is None or m.ready:
            return m


def lockstep(ser, parser, duration_s, u_of_t, garble_every=0, reply_timeout_s=0.005,
             give_up_s=0.04, rate_hz=0):
    """Send a command, wait for its reply, repeat. A missing reply (a lost byte)
    is counted and the next command is sent; only give_up_s without any reply
    ends the check (the ESP32 itself stops after 50 ms). rate_hz paces the
    commands like a Simulink model step (0 = as fast as the link allows)."""
    stats = LinkStats()
    queue = []
    t0 = time.monotonic()
    last_reply = t0
    while True:
        t = time.monotonic() - t0
        if t >= duration_s:
            stats.end = "time up"
            break
        if rate_hz:
            _wait_until(t0 + stats.commands / rate_hz)
            t = time.monotonic() - t0
        stats.commands += 1
        if garble_every and stats.commands % garble_every == 0:
            ser.write(encode_command(float("nan")))   # the ESP32 must drop it, not answer
            continue
        sent = time.monotonic()
        ser.write(encode_command(u_of_t(t)))
        m = _next_measurement(ser, parser, queue, sent + reply_timeout_s)
        if m is None:
            stats.missed_replies += 1
            if time.monotonic() - last_reply >= give_up_s:
                stats.end = f"no reply for {give_up_s * 1000:.0f} ms (run ended on the ESP32?)"
                break
            continue
        last_reply = time.monotonic()
        stats.rtts.append(last_reply - sent)
        stats.replies += 1
        stats.last = m
        if not m.ready:
            stats.end = "ESP32 left the run (ready flag off)"
            break
    stats.bad_checksums = parser.bad_checksums
    return stats


def _wait_until(t_target):
    while time.monotonic() < t_target:
        pass


def blast(ser, duration_s, u, rate_hz=1000):
    """Write commands at rate_hz without reading or waiting for ready: leaves
    bytes in the ESP32's UART2 (rig check for stale bytes). Returns the count."""
    n = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        ser.write(encode_command(u))
        n += 1
        _wait_until(t0 + n / rate_hz)
    return n


def square_wave(amp, half_period_s=0.5):
    return lambda t: amp if int(t / half_period_s) % 2 == 0 else -amp


def _pulse(handle, pin):
    import lgpio
    lgpio.gpio_write(handle, pin, 1)
    time.sleep(PULSE_S)
    lgpio.gpio_write(handle, pin, 0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=230400)   # Simulink Serial blocks max out here
    ap.add_argument("--seconds", type=float, default=10.0, help="lockstep duration")
    ap.add_argument("--u-amp", type=float, default=0.0, help="square wave amplitude in volts (0 = u stays 0)")
    ap.add_argument("--half-period", type=float, default=0.5,
                    help="seconds each way of the square wave (the cart reaches ~1 m/s in 0.35 s at 0.5 V)")
    ap.add_argument("--garble", type=int, default=0, metavar="N", help="send NaN as every Nth command")
    ap.add_argument("--rate", type=float, default=1000.0, help="commands per second, like the model step (0 = free-run)")
    ap.add_argument("--blast", action="store_true",
                    help="only write commands (u = --u-amp or 0.8 V) for --seconds, never wait for ready")
    ap.add_argument("--gpio", action="store_true", help="pulse start before and stop after")
    ap.add_argument("--gpio-chip", type=int, default=15, help="lgpio chip (dac_daemon.py uses 15 on the Pi 5)")
    ap.add_argument("--ready-timeout", type=float, default=60.0, help="seconds to wait for the run to open")
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.0005)
    ser.reset_input_buffer()
    if args.blast:
        u = args.u_amp or 0.8
        n = blast(ser, args.seconds, u, args.rate or 1000.0)
        print(f"Blasted {n} commands of {u:.2f} V without waiting. A run started now must not move until its own commands arrive.")
        return 0
    handle = None
    if args.gpio:
        import lgpio
        handle = lgpio.gpiochip_open(args.gpio_chip)
        for pin in (GPIO_HOME_START, GPIO_STOP):
            lgpio.gpio_claim_output(handle, pin, 0)
        _pulse(handle, GPIO_HOME_START)
        print("Start pulse sent.")

    print("Waiting for a Simulink run (homing + hanging reference take about 15 s)...")
    parser = MeasurementParser()
    m = wait_for_ready(ser, parser, args.ready_timeout)
    if m is None:
        print(f"No ready packet within {args.ready_timeout:.0f} s. Is Simulink selected and running?")
        return 1
    print(f"Ready: cart {m.cart}, pend {m.pend}, flags 0x{m.flags:02x}")

    u_of_t = square_wave(args.u_amp, args.half_period) if args.u_amp else (lambda t: 0.0)
    stats = lockstep(ser, parser, args.seconds, u_of_t, args.garble, rate_hz=args.rate)
    if handle is not None:
        _pulse(handle, GPIO_STOP)
        print("Stop pulse sent.")

    rate = stats.replies / args.seconds if stats.end == "time up" else float("nan")
    print(f"Ended: {stats.end}")
    print(f"Commands {stats.commands}, replies {stats.replies}, missed replies {stats.missed_replies}, "
          f"loop rate {rate:.0f} Hz")
    if stats.rtts:
        print(f"Round trip: median {statistics.median(stats.rtts) * 1000:.3f} ms, "
              f"max {max(stats.rtts) * 1000:.3f} ms")
    print(f"Bad checksums {stats.bad_checksums}, skipped bytes {parser.skipped_bytes}")
    if stats.last is not None:
        print(f"Last: cart {stats.last.cart}, pend {stats.last.pend}, flags 0x{stats.last.flags:02x}")
    return 0 if stats.end == "time up" and stats.bad_checksums == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
