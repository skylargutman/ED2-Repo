"""Packets of the Pi <-> ESP32 UART link used in Simulink mode.

Wiring: Pi GPIO14 (TX, pin 8) -> ESP32 GPIO16, Pi GPIO15 (RX, pin 10) <- ESP32
GPIO17, 230400 baud 8N1 (the Raspberry Pi Blockset Serial blocks' maximum), /dev/ttyAMA0 on the Pi. Lockstep: every command the Pi
sends is answered with one measurement.

  Pi -> ESP32 command, 6 bytes:      A5 5A  u (float32 LE, volts, + = drive right)
  ESP32 -> Pi measurement, 12 bytes: A5 5A  cart (int32 LE)  pend (int32 LE)
                                     flags  XOR of bytes 2-10
Cart count: 0 at centre, + to the right. Pendulum count: 0 hanging.
Bytes 2-11 of a measurement are the old 10-byte SPI packet, so the models'
unpack_spi block reads them unchanged. The firmware side is
cart/cart_single/pi_link.ino; both use the examples in tests/test_pilink.py.
"""
import struct
from dataclasses import dataclass

HEADER = b"\xa5\x5a"
CMD_LEN = 6
MEAS_LEN = 12

FLAG_READY = 0x01      # a Simulink run is open; commands are applied
FLAG_HOMED = 0x02      # homing-complete line (ESP32 GPIO 18) is HIGH
FLAG_SIMULINK = 0x04   # the ESP32 has the Simulink experiment selected


def encode_command(u: float) -> bytes:
    return HEADER + struct.pack("<f", u)


def decode_command(packet: bytes) -> float:
    if len(packet) != CMD_LEN or packet[:2] != HEADER:
        raise ValueError(f"not a command packet: {packet.hex(' ')}")
    return struct.unpack("<f", packet[2:])[0]


def _xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def encode_measurement(cart: int, pend: int, flags: int) -> bytes:
    body = struct.pack("<iiB", cart, pend, flags)
    return HEADER + body + bytes([_xor(body)])


@dataclass(frozen=True)
class Measurement:
    cart: int
    pend: int
    flags: int

    @property
    def ready(self) -> bool:
        return bool(self.flags & FLAG_READY)

    @property
    def homed(self) -> bool:
        return bool(self.flags & FLAG_HOMED)

    @property
    def simulink(self) -> bool:
        return bool(self.flags & FLAG_SIMULINK)


class MeasurementParser:
    """Splits a byte stream into measurements. Resyncs on the header, so a lost
    or corrupted byte costs one packet, not the stream."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_checksums = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list:
        self._buf += data
        out = []
        while True:
            i = self._buf.find(HEADER)
            if i < 0:
                keep = 1 if self._buf[-1:] == HEADER[:1] else 0   # may be half a header
                self.skipped_bytes += len(self._buf) - keep
                del self._buf[:len(self._buf) - keep]
                return out
            if i > 0:
                self.skipped_bytes += i
                del self._buf[:i]
            if len(self._buf) < MEAS_LEN:
                return out
            body = bytes(self._buf[2:11])
            if _xor(body) != self._buf[11]:
                self.bad_checksums += 1
                del self._buf[:1]            # look for the next header after this one
                continue
            del self._buf[:MEAS_LEN]
            out.append(Measurement(*struct.unpack("<iiB", body)))
