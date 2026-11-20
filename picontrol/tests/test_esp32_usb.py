import json
import math

import pytest

import esp32_usb
from esp32_usb import Esp32Usb, parse_data_line, telemetry_json, wrap_angle


@pytest.mark.parametrize("theta, wrapped", [
    (0.0, 0.0),                      # upright
    (math.pi, math.pi),              # hanging at run start
    (-math.pi, math.pi),             # hanging, swung the other way
    (2 * math.pi + 0.1, 0.1),        # upright after a full turn
    (9.4982, 9.4982 - 4 * math.pi),  # the 10-30 website reading, fallen and spun past a turn
    (-0.2, -0.2),
])
def test_wrap_angle(theta, wrapped):
    assert wrap_angle(theta) == pytest.approx(wrapped)
    assert -math.pi < wrap_angle(theta) <= math.pi


def test_parse_data_line():
    assert parse_data_line("D 3826 -0.0002 3.4321 0.000 nan") == (3.826, -0.0002, 3.4321, 0.0)
    assert parse_data_line("SIM u  0.000 V  th 161.7 deg") is None
    assert parse_data_line("D 12 garbled") is None


def test_telemetry_matches_the_simulink_message_format():
    msg = json.loads(telemetry_json(1.23456, 0.01234, math.pi, -0.5))
    assert msg == {"time": "1.235", "position": "0.0123", "angle": "3.1416", "control": "-0.500"}


class FakeSerial:
    """Answers commands the way cart_single/protocol.ino does."""

    def __init__(self, owner=None):
        self.owner = owner
        self.sent = []

    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        if cmd.startswith("exp select "):
            self.owner.handle_line(f"ok exp {cmd.split()[2]}")
        elif cmd == "run":
            self.owner.handle_line("ok home")
        elif cmd == "stop":
            self.owner.handle_line("ok stop")

    def close(self):
        pass


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def rig():
    ser = FakeSerial()
    telemetry, status, clock = [], [], Clock()
    esp = Esp32Usb(ser, telemetry.append, status.append, clock=clock, log_dir=None)
    ser.owner = esp
    return esp, ser, telemetry, status, clock


def test_start_selects_the_firmware_experiment_then_runs(rig):
    esp, ser, _, status, _ = rig
    assert esp.start("SwingHoldPendulum")
    assert ser.sent == ["exp select SwingBal", "run"]
    assert esp.running and status == ["homing"]


def test_start_fails_if_the_esp32_is_busy(rig):
    esp, ser, _, _, _ = rig
    ser.write = lambda data: esp.handle_line("err exp busy (HOMING)") if b"exp" in data else None
    assert not esp.start("SwingHoldPendulum")
    assert not esp.running


def test_data_lines_become_telemetry_at_10_hz(rig):
    esp, _, telemetry, status, clock = rig
    esp.start("SwingHoldPendulum")
    esp.handle_line("run start SwingBal BAL_POLE=5")
    assert status[-1] == "running:SwingHoldPendulum"
    for i in range(100):                      # 100 lines, 2 ms apart = 0.2 s
        clock.t += 0.002
        esp.handle_line(f"D {2 * i} 0.0100 3.1416 0.250 nan")
    assert len(telemetry) == 2
    assert json.loads(telemetry[0])["control"] == "0.250"


def test_data_lines_outside_a_session_are_not_published(rig):
    esp, _, telemetry, _, _ = rig
    esp.handle_line("D 2 0.0 3.14 0.0 nan")
    assert telemetry == []


def test_run_ends_after_run_s(rig):
    esp, ser, _, status, clock = rig
    esp.start("SwingHoldPendulum")
    clock.t += esp32_usb.RUN_S + 0.1
    esp.tick()
    assert ser.sent[-1] == "stop"
    assert not esp.running and status[-1] == "stopped"


def test_stop_and_limit_switch_end_the_session(rig):
    esp, ser, _, status, _ = rig
    esp.start("SwingHoldPendulum")
    esp.handle_line("run end limit-switch")
    assert not esp.running and status[-1] == "stopped"
    assert ser.sent == ["exp select SwingBal", "run"]     # nothing extra sent


def test_soft_limit_followed_by_a_rehome_keeps_the_session(rig):
    esp, _, _, status, clock = rig
    esp.start("SwingHoldPendulum")
    esp.handle_line("run end soft-limit")
    clock.t += 1.0
    esp.handle_line("Homing: driving left...")
    clock.t += esp32_usb.REHOME_GRACE_S
    esp.tick()
    assert esp.running and status[-1] == "homing"


def test_soft_limit_without_a_rehome_ends_the_session(rig):
    esp, _, _, status, clock = rig
    esp.start("SwingHoldPendulum")
    esp.handle_line("run end soft-limit")
    clock.t += esp32_usb.REHOME_GRACE_S
    esp.tick()
    assert not esp.running and status[-1] == "stopped"
