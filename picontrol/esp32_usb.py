"""Run the ESP32's own experiments over its USB serial port (cart/cart_single, v11.2).

The firmware's swing-up & balance ("SwingBal") works on this rig; the Simulink
SwingHoldPendulum model doesn't yet. So the website's SwingHoldPendulum runs the
firmware experiment, and every other experiment stays on Simulink over UART.

The two paths don't interfere: the Pi start pulse (GPIO 16) always selects the
firmware's "Simulink" experiment, and this module selects its experiment with
"exp select NAME" right before "run".

USB protocol (cart_single/protocol.ino, 921600 baud, one command per line):
  exp select NAME -> "ok exp NAME"      run -> "ok home" (homes, then runs)
  stop            -> "ok stop"          (always accepted, also while homing)
  while a run is open: "D t_ms x theta u ref" every 2 ms, then "run end REASON".
  theta is continuous: 0 = upright, +pi = hanging at run start.
After a "soft-limit" end SwingBal re-homes and runs again (up to 3 times), so a
soft-limit end only closes the session if no re-home follows.
"""
import glob
import json
import logging
import math
import os
import threading
import time

log = logging.getLogger(__name__)

# Website experiment name -> firmware experiment name
FIRMWARE_EXPERIMENTS = {"SwingHoldPendulum": "SwingBal"}

BAUD = 921600
RUN_S = 60.0              # a website run ends after this long (SwingBal itself never stops)
TELEMETRY_PERIOD_S = 0.1  # same rate as the Simulink models' MQTT publishing
REHOME_GRACE_S = 3.0      # after "run end soft-limit", wait this long for an automatic re-home
REPLY_TIMEOUT_S = 2.0
LOG_DIR = os.path.expanduser("~/picontrol/logs")


def find_port():
    """The ESP32's USB serial device, or None. ESP32_USB_PORT overrides."""
    if os.environ.get("ESP32_USB_PORT"):
        return os.environ["ESP32_USB_PORT"]
    ports = sorted(glob.glob("/dev/serial/by-id/*")) or sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def wrap_angle(theta):
    """Continuous firmware angle -> (-pi, pi]: 0 = upright, +-pi = hanging."""
    return -((-theta + math.pi) % (2 * math.pi) - math.pi)


def parse_data_line(line):
    """"D t_ms x theta u ref" -> (t_s, x_m, theta_rad, u_volts), or None."""
    parts = line.split()
    if len(parts) < 5 or parts[0] != "D":
        return None
    try:
        return int(parts[1]) / 1000.0, float(parts[2]), float(parts[3]), float(parts[4])
    except ValueError:
        return None


def telemetry_json(t, x, theta, u):
    """Same fields and formatting as the Simulink models' MQTT message."""
    return json.dumps({"time": f"{t:.3f}", "position": f"{x:.4f}",
                       "angle": f"{wrap_angle(theta):.4f}", "control": f"{u:.3f}"})


def open_serial(port):
    """Open without resetting the ESP32. DTR/RTS drive its EN/BOOT pins through the
    usual two-transistor circuit, which resets only when the two lines differ. Linux
    raises both on open; leave them there. Setting dtr/rts False lowers them one at a
    time, and the moment in between reset the ESP32 on every open (Pi, 2026-11-06)."""
    import serial
    return serial.Serial(port, BAUD, timeout=0.2)


class Esp32Usb:
    """Owns the ESP32's USB port: logs every line, runs one firmware session at a time.

    publish_telemetry(json_text) and publish_status(text) are called from the reader thread.
    """

    def __init__(self, ser, publish_telemetry, publish_status, clock=time.monotonic, log_dir=LOG_DIR):
        self.ser = ser
        self.publish_telemetry = publish_telemetry
        self.publish_status = publish_status
        self.clock = clock
        self.log_dir = log_dir
        self.lock = threading.Lock()
        self.replies = []            # "ok ..." / "err ..." lines, for wait_reply()
        self.session = None          # website experiment name while a session is open
        self.deadline = None
        self.ended_at = None         # clock time of a soft-limit "run end", while awaiting a re-home
        self.next_pub = 0.0
        self.t0 = 0.0
        self.log_file = None
        self._stop = threading.Event()
        self._thread = None

    # --- commands ------------------------------------------------------------

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self._log(f"> {cmd}")

    def wait_reply(self, prefix, timeout=REPLY_TIMEOUT_S):
        """Wait for a reply line starting with prefix (or any "err"); returns it or None."""
        end = self.clock() + timeout
        while self.clock() < end:
            with self.lock:
                for i, r in enumerate(self.replies):
                    if r.startswith(prefix) or r.startswith("err"):
                        del self.replies[i]
                        return r
            time.sleep(0.01)
        return None

    def start(self, experiment):
        """Select and run the firmware experiment for a website experiment. Returns True if started."""
        fw = FIRMWARE_EXPERIMENTS[experiment]
        self.stop_session(send_stop=False)
        with self.lock:
            self.replies.clear()
        self._open_log(fw)
        self.send(f"exp select {fw}")
        r = self.wait_reply("ok exp")
        if r != f"ok exp {fw}":
            log.error(f"ESP32 did not select {fw}: {r}")
            return False
        self.send("run")
        r = self.wait_reply("ok home")
        if r != "ok home":
            log.error(f"ESP32 did not start {fw}: {r}")
            return False
        with self.lock:
            self.session = experiment
            self.t0 = self.next_pub = self.clock()
            self.deadline = self.t0 + RUN_S
            self.ended_at = None
        self.publish_status("homing")
        return True

    def stop_session(self, send_stop=True, reason="stopped"):
        with self.lock:
            was_open = self.session is not None
            self.session = self.deadline = self.ended_at = None
        if send_stop:
            self.send("stop")
        if was_open:
            log.info(f"Firmware session ended: {reason}")
            self.publish_status("stopped")

    @property
    def running(self):
        return self.session is not None

    # --- reader --------------------------------------------------------------

    def handle_line(self, line):
        """Process one line from the ESP32 (called by the reader thread; also used by tests)."""
        self._log(line)
        if line.startswith(("ok ", "err ")):
            with self.lock:
                self.replies.append(line)
            return
        with self.lock:
            session = self.session
        if session is None:
            return
        if line.startswith("run start"):
            with self.lock:
                self.ended_at = None
            self.publish_status(f"running:{session}")
        elif line.startswith("Homing") and self.ended_at is not None:
            with self.lock:
                self.ended_at = None      # automatic re-home after a soft-limit: same session
            self.publish_status("homing")
        elif line.startswith("run end"):
            reason = line[len("run end"):].strip()
            if reason == "soft-limit":
                with self.lock:
                    self.ended_at = self.clock()
            else:
                self.stop_session(send_stop=False, reason=f"run end {reason}")
        else:
            d = parse_data_line(line)
            now = self.clock()
            if d and now >= self.next_pub:
                self.next_pub = now + TELEMETRY_PERIOD_S
                _, x, theta, u = d
                self.publish_telemetry(telemetry_json(now - self.t0, x, theta, u))

    def tick(self):
        """Time-based endings (called by the reader thread every read timeout)."""
        with self.lock:
            session, deadline, ended_at = self.session, self.deadline, self.ended_at
        if session is None:
            return
        now = self.clock()
        if deadline is not None and now >= deadline:
            self.stop_session(reason=f"{RUN_S:.0f} s run time")
        elif ended_at is not None and now - ended_at >= REHOME_GRACE_S:
            self.stop_session(send_stop=False, reason="run end soft-limit")

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception as e:
                log.error(f"ESP32 USB read failed: {e}")
                self.stop_session(send_stop=False, reason="USB read failed")
                return
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle_line(raw.decode(errors="replace").strip())
            self.tick()

    def start_reader(self):
        self._thread = threading.Thread(target=self._reader, name="esp32-usb", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self.running:
            self.stop_session()
        if self._thread:
            self._thread.join(timeout=2)
        self.ser.close()
        if self.log_file:
            self.log_file.close()

    # --- log file ------------------------------------------------------------

    def _open_log(self, fw):
        if self.log_dir is None:
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            if self.log_file:
                self.log_file.close()
            name = time.strftime("%Y%m%d_%H%M%S") + f"_{fw}.log"
            self.log_file = open(os.path.join(self.log_dir, name), "w", buffering=1)
        except OSError as e:
            log.error(f"Cannot open ESP32 log: {e}")
            self.log_file = None

    def _log(self, line):
        if self.log_file:
            self.log_file.write(f"{time.strftime('%H:%M:%S')} {line}\n")
