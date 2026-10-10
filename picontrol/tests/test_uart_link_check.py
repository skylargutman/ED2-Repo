import math

from pilink import (FLAG_HOMED, FLAG_READY, FLAG_SIMULINK, CMD_LEN, MeasurementParser,
                    decode_command, encode_measurement)
from uart_link_check import lockstep, square_wave, wait_for_ready

READY = FLAG_READY | FLAG_HOMED | FLAG_SIMULINK


class FakeEsp:
    """Answers each valid command with a measurement, like the firmware.
    Stops answering after `answer_limit` commands (run ended)."""

    def __init__(self, answer_limit=10**9, flags=READY, idle=b""):
        self.out = bytearray(idle)
        self.inbuf = bytearray()
        self.commands = []
        self.answer_limit = answer_limit
        self.flags = flags

    def write(self, data):
        self.inbuf += data
        while len(self.inbuf) >= CMD_LEN:
            u = decode_command(bytes(self.inbuf[:CMD_LEN]))
            del self.inbuf[:CMD_LEN]
            self.commands.append(u)
            if math.isfinite(u) and abs(u) <= 5.0 and len(self.commands) <= self.answer_limit:
                self.out += encode_measurement(len(self.commands), -len(self.commands), self.flags)
        return len(data)

    def read(self, n=1):
        data, self.out = bytes(self.out[:n]), self.out[n:]
        return data


def test_wait_for_ready_skips_not_ready_packets():
    idle = encode_measurement(0, 0, FLAG_SIMULINK) + encode_measurement(5, 6, READY)
    m = wait_for_ready(FakeEsp(idle=idle), MeasurementParser(), timeout_s=0.5)
    assert m is not None and m.cart == 5 and m.ready


def test_wait_for_ready_times_out():
    assert wait_for_ready(FakeEsp(), MeasurementParser(), timeout_s=0.05) is None


def test_lockstep_runs_until_time_up():
    esp = FakeEsp()
    stats = lockstep(esp, MeasurementParser(), 0.05, lambda t: 0.25)
    assert stats.end == "time up"
    assert stats.commands == stats.replies == len(esp.commands) > 0
    assert set(esp.commands) == {0.25}
    assert stats.last.cart == stats.replies and stats.bad_checksums == 0


def test_lockstep_stops_when_esp_stops_answering():
    stats = lockstep(FakeEsp(answer_limit=3), MeasurementParser(), 1.0, lambda t: 0.0)
    assert stats.replies == 3 and stats.commands == 4
    assert stats.end.startswith("no reply")


def test_lockstep_stops_when_ready_flag_drops():
    stats = lockstep(FakeEsp(flags=FLAG_HOMED), MeasurementParser(), 1.0, lambda t: 0.0)
    assert stats.replies == 1 and stats.end.startswith("ESP32 left the run")


def test_garble_sends_nan_and_skips_waiting_for_its_reply():
    esp = FakeEsp()
    stats = lockstep(esp, MeasurementParser(), 0.05, lambda t: 0.1, garble_every=3)
    assert stats.end == "time up"
    nans = [u for u in esp.commands if math.isnan(u)]
    assert len(nans) == len(esp.commands) // 3
    assert stats.replies == len(esp.commands) - len(nans)


def test_square_wave():
    f = square_wave(0.8, 0.5)
    assert f(0.1) == 0.8 and f(0.6) == -0.8 and f(1.1) == 0.8
