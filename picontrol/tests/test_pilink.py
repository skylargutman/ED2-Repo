import math
import struct

import pytest

from pilink import (FLAG_HOMED, FLAG_READY, FLAG_SIMULINK, Measurement, MeasurementParser,
                    decode_command, encode_command, encode_measurement)

# The same example bytes are in the comments of cart/cart_single/pi_link.ino
CMD_HALF = bytes.fromhex("a5 5a 00 00 00 3f")            # u = 0.5 V
CMD_NEG = bytes.fromhex("a5 5a 00 00 a0 bf")             # u = -1.25 V
MEAS_A = bytes.fromhex("a5 5a e8 03 00 00 fb ff ff ff 07 e8")   # cart 1000, pend -5, flags 7
MEAS_B = bytes.fromhex("a5 5a c0 1d fe ff 02 00 00 00 02 dc")   # cart -123456, pend 2, flags 2


def test_encode_command_examples():
    assert encode_command(0.5) == CMD_HALF
    assert encode_command(-1.25) == CMD_NEG


def test_decode_command_round_trip_and_errors():
    assert decode_command(CMD_NEG) == -1.25
    assert math.isnan(decode_command(encode_command(float("nan"))))
    with pytest.raises(ValueError):
        decode_command(CMD_HALF[:5])
    with pytest.raises(ValueError):
        decode_command(b"\x00\x00" + CMD_HALF[2:])


def test_encode_measurement_examples():
    assert encode_measurement(1000, -5, 7) == MEAS_A
    assert encode_measurement(-123456, 2, 2) == MEAS_B


def test_flags():
    m = Measurement(0, 0, FLAG_READY | FLAG_SIMULINK)
    assert m.ready and m.simulink and not m.homed
    assert Measurement(0, 0, FLAG_HOMED).homed


def test_parser_whole_packets():
    p = MeasurementParser()
    assert p.feed(MEAS_A + MEAS_B) == [Measurement(1000, -5, 7), Measurement(-123456, 2, 2)]
    assert p.bad_checksums == 0 and p.skipped_bytes == 0


def test_parser_byte_by_byte():
    p = MeasurementParser()
    got = []
    for b in MEAS_A:
        got += p.feed(bytes([b]))
    assert got == [Measurement(1000, -5, 7)]


def test_parser_skips_noise_before_header():
    p = MeasurementParser()
    assert p.feed(b"\x00\xa5\x13\xff" + MEAS_A) == [Measurement(1000, -5, 7)]
    assert p.skipped_bytes == 4


def test_parser_bad_checksum_resyncs_on_next_packet():
    bad = MEAS_A[:11] + bytes([MEAS_A[11] ^ 0x01])
    p = MeasurementParser()
    assert p.feed(bad + MEAS_B) == [Measurement(-123456, 2, 2)]
    assert p.bad_checksums == 1


def test_parser_header_bytes_inside_data():
    # cart = 0x5AA5 puts "a5 5a" inside the packet body
    pkt = encode_measurement(0x5AA5, 0, 0)
    p = MeasurementParser()
    assert p.feed(pkt + pkt) == [Measurement(0x5AA5, 0, 0)] * 2


def test_command_is_float32_little_endian():
    assert struct.unpack("<f", encode_command(2.5)[2:])[0] == 2.5
