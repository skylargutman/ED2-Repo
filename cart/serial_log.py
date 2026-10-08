"""Log the ESP32 serial output to text files.

Logs every USB serial port it finds (or the ones given) at the same time, one
file per port in cart/logs/, each line stamped with the PC time. Also echoes
everything to the terminal. Stop with Ctrl+C.

Type a command and press Enter (e.g. io, home, stop, status) to send it to the
board(s); it is written to the log as "> command" so replies line up with it.

After a run stops (STOP, limit hit, soft-limit stop), it also sends "io" by
itself until the pendulum hangs still, and logs "# rest drift: ..." = how many
pendulum counts were lost during the run (see RestDrift).

    python cart/serial_log.py              # all USB serial ports
    python cart/serial_log.py COM3 COM4    # just these

Close the Arduino IDE serial monitor first: only one program can open a port.
The port is opened with DTR/RTS released so the ESP32 is not reset.

Needs pyserial:  python -m pip install pyserial
"""

import datetime
import os
import re
import sys
import threading
import time

import serial
import serial.tools.list_ports

BAUD = 921600
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
USB_HINTS = ("CP210", "CH340", "CH910", "USB", "UART")

print_lock = threading.Lock()
open_ports = {}          # port -> (serial, log file, lock) while it is being logged


def stamp_now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def find_ports():
    return [p.device for p in serial.tools.list_ports.comports()
            if any(h in (p.description or "").upper() for h in USB_HINTS)]


class RestDrift:
    """After a run stops, find where the pendulum comes to rest.

    Sends "io" every 0.5 s (single-board firmware, cart_single) until the
    pendulum count has held within 1 count for 3 s, then reports how far it is
    from the "Hanging reference" of that run, ignoring whole turns. 0 means no
    counts were lost while swinging; more means the encoder dropped counts,
    which also shifts "upright".
    """
    CPR = 1968                      # PEND_COUNTS_PER_REV
    STOP_PREFIXES = ("STOP triggered", "LIMIT HIT", "Stopped at")
    HANG = re.compile(r"^Hanging reference: (-?\d+)")
    IO = re.compile(r"^io cart -?\d+ pend (-?\d+)")
    POLL_S, STILL_S, GIVE_UP_S = 0.5, 3.0, 180.0

    def __init__(self):
        self.ref = None             # hanging reference of the current run
        self.watching = False

    def on_line(self, text):
        """Returns a report line once the pendulum has settled, else None."""
        m = self.HANG.match(text)
        if m:
            self.ref = int(m.group(1))
            self.watching = False
        elif text.startswith("Homing: driving left"):
            self.watching = False    # a re-home or new run started
        elif text.startswith(self.STOP_PREFIXES) and self.ref is not None:
            self.watching = True
            self.started = time.time()
            self.next_poll = 0.0
            self.last = None
            self.still_since = None
        elif self.watching:
            m = self.IO.match(text)
            if m:
                return self._sample(int(m.group(1)))
        return None

    def _sample(self, pend):
        now = time.time()
        if self.last is None or abs(pend - self.last) > 1:
            self.last, self.still_since = pend, now
            return None
        if now - self.still_since < self.STILL_S:
            return None
        self.watching = False
        d = (pend - self.ref) % self.CPR
        if d > self.CPR // 2:
            d -= self.CPR
        return (f"# rest drift: pend {pend}, hanging ref {self.ref}, "
                f"drift {d:+d} counts ({d * 360 / self.CPR:+.1f} deg)")

    def poll_due(self):
        """True when it's time to send another "io"."""
        if not self.watching:
            return False
        now = time.time()
        if now - self.started > self.GIVE_UP_S:
            self.watching = False
            return False
        if now >= self.next_poll:
            self.next_poll = now + self.POLL_S
            return True
        return False


def log_port(port, stamp, stop):
    path = os.path.join(LOG_DIR, f"{stamp}_{port}.log")
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, BAUD, 0.2
    ser.dtr = False          # don't pulse EN / IO0 (auto-reset circuit)
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        with print_lock:
            print(f"[{port}] can't open: {e}  (is the Arduino serial monitor open?)")
        return
    with print_lock:
        print(f"[{port}] logging to {path}")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        file_lock = threading.Lock()
        open_ports[port] = (ser, f, file_lock)
        rest = RestDrift()
        buf = b""
        while not stop.is_set():
            try:
                if rest.poll_due():
                    ser.write(b"io\n")
                chunk = ser.read(4096)
            except serial.SerialException as e:
                with print_lock:
                    print(f"[{port}] disconnected: {e}")
                break
            if not chunk:
                continue
            buf += chunk
            *lines, buf = buf.split(b"\n")
            with file_lock:
                for raw in lines:
                    text = raw.decode("utf-8", errors="replace").rstrip("\r")
                    f.write(f"{stamp_now()} {text}\n")
                    with print_lock:
                        print(f"[{port}] {text}")
                    report = rest.on_line(text)
                    if report:
                        f.write(f"{stamp_now()} {report}\n")
                        with print_lock:
                            print(f"[{port}] {report}")
                f.flush()
        open_ports.pop(port, None)
    ser.close()


def send_typed():
    """Send each line typed in the terminal to every open port, and log it."""
    for line in sys.stdin:
        cmd = line.strip()
        if not cmd:
            continue
        for port, (ser, f, file_lock) in list(open_ports.items()):
            try:
                ser.write((cmd + "\n").encode())
            except serial.SerialException as e:
                with print_lock:
                    print(f"[{port}] can't send: {e}")
                continue
            with file_lock:
                f.write(f"{stamp_now()} > {cmd}\n")
                f.flush()


def main():
    ports = sys.argv[1:] or find_ports()
    if not ports:
        print("No USB serial ports found. Plug the boards in, or name the ports: "
              "python cart/serial_log.py COM3 COM4")
        return
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    stop = threading.Event()
    threads = [threading.Thread(target=log_port, args=(p, stamp, stop), daemon=True)
               for p in ports]
    for t in threads:
        t.start()
    threading.Thread(target=send_typed, daemon=True).start()
    print("Logging. Type a command + Enter to send it (io, home, stop, status). "
          "Ctrl+C to stop.")
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(timeout=1)
    print(f"Logs saved in {LOG_DIR}")


if __name__ == "__main__":
    main()
