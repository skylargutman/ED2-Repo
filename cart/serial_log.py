"""Log the ESP32 serial output to text files.

Logs every USB serial port it finds (or the ones given) at the same time, one
file per port in cart/logs/, each line stamped with the PC time. Also echoes
everything to the terminal. Stop with Ctrl+C.

    python cart/serial_log.py              # all USB serial ports
    python cart/serial_log.py COM3 COM4    # just these

Close the Arduino IDE serial monitor first: only one program can open a port.
The port is opened with DTR/RTS released so the ESP32 is not reset.

Needs pyserial:  python -m pip install pyserial
"""

import datetime
import os
import sys
import threading
import time

import serial
import serial.tools.list_ports

BAUD = 115200
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
USB_HINTS = ("CP210", "CH340", "CH910", "USB", "UART")

print_lock = threading.Lock()


def find_ports():
    return [p.device for p in serial.tools.list_ports.comports()
            if any(h in (p.description or "").upper() for h in USB_HINTS)]


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
        buf = b""
        while not stop.is_set():
            try:
                chunk = ser.read(4096)
            except serial.SerialException as e:
                with print_lock:
                    print(f"[{port}] disconnected: {e}")
                break
            if not chunk:
                continue
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                text = raw.decode("utf-8", errors="replace").rstrip("\r")
                now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                f.write(f"{now} {text}\n")
                with print_lock:
                    print(f"[{port}] {text}")
            f.flush()
    ser.close()


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
    print("Logging. Ctrl+C to stop.")
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
