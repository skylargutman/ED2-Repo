import math
import statistics
import time

from pilink import (FLAG_HOMED, FLAG_READY, FLAG_SIMULINK, CMD_LEN, MeasurementParser,
                    decode_command, encode_measurement)
from uart_link_check import blast, lockstep, square_wave, wait_for_ready

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
    # keeps resending for 40 ms after the last reply, then gives up
    stats = lockstep(FakeEsp(answer_limit=3), MeasurementParser(), 1.0, lambda t: 0.0)
    assert stats.replies == 3 and stats.commands > 4
    assert stats.missed_replies == stats.commands - 3
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


class PyserialLikeEsp(FakeEsp):
    """read(n) like pyserial with a timeout: returns at once only when n bytes are
    waiting, otherwise sits out the timeout and returns what is there."""

    def read(self, n=1):
        if len(self.out) < n:
            time.sleep(0.0005)
        return FakeEsp.read(self, n)


class LosesReplies(FakeEsp):
    """Never answers the given command numbers (a byte of that command was lost)."""

    def __init__(self, lost, **kw):
        super().__init__(**kw)
        self.lost = set(lost)

    def write(self, data):
        before = len(self.out)
        n = len(self.commands)
        super().write(data)
        if len(self.commands) > n and len(self.commands) in self.lost:
            del self.out[before:]
        return len(data)


def test_reply_is_read_without_waiting_for_the_serial_timeout():
    stats = lockstep(PyserialLikeEsp(), MeasurementParser(), 0.05, lambda t: 0.0)
    assert statistics.median(stats.rtts) < 0.0003


def test_one_lost_reply_does_not_end_the_run():
    esp = LosesReplies({5})
    stats = lockstep(esp, MeasurementParser(), 0.1, lambda t: 0.0)
    assert stats.end == "time up"
    assert stats.missed_replies == 1
    assert stats.replies == stats.commands - 1


def test_rate_paces_the_commands():
    stats = lockstep(FakeEsp(), MeasurementParser(), 0.1, lambda t: 0.0, rate_hz=200)
    assert 17 <= stats.commands <= 21


def test_blast_writes_commands_without_reading():
    esp = FakeEsp()
    esp.read = None                     # blast must never read
    n = blast(esp, 0.05, 0.75, rate_hz=1000)   # 0.75 is exact in float32
    assert n == len(esp.commands) > 10
    assert set(esp.commands) == {0.75}


def test_default_port_is_the_gpio14_15_uart_on_a_pi5(monkeypatch):
    # On the Pi 5, /dev/serial0 is the debug connector (ttyAMA10); pins 8/10 are ttyAMA0
    import sys as _sys
    import types

    opened = {}

    class StopHere(Exception):
        pass

    def fake_serial(port, baud, timeout):
        opened["port"] = port
        raise StopHere

    monkeypatch.setitem(_sys.modules, "serial", types.SimpleNamespace(Serial=fake_serial))
    import uart_link_check
    try:
        uart_link_check.main([])
    except StopHere:
        pass
    assert opened["port"] == "/dev/ttyAMA0"
