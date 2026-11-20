import logging
import sys
import types

import pytest

# The daemon imports Pi-only modules at the top; stub them for the test.
# "def setup_mqtt() -> mqtt.Client" is evaluated at import, so the stub needs Client.
for name in ("lgpio", "smbus2", "paho", "paho.mqtt", "paho.mqtt.client"):
    sys.modules.setdefault(name, types.ModuleType(name))
if not hasattr(sys.modules["paho.mqtt.client"], "Client"):
    sys.modules["paho.mqtt.client"].Client = object

import dac_daemon  # noqa: E402


class FakeBus:
    def __init__(self, present):
        self.present = present
        self.writes = []

    def read_byte(self, addr):
        if not self.present:
            raise OSError(121, "Remote I/O error")
        return 0

    def write_i2c_block_data(self, addr, reg, data):
        self.writes.append((addr, reg, data))


@pytest.fixture(autouse=True)
def reset_dac(monkeypatch):
    monkeypatch.setattr(dac_daemon, "_dac_present", False)


def test_absent_dac_is_skipped_quietly(monkeypatch, caplog):
    bus = FakeBus(present=False)
    monkeypatch.setattr(dac_daemon, "_bus", bus)
    with caplog.at_level(logging.INFO):
        assert dac_daemon.dac_probe() is False
        dac_daemon.dac_set_neutral()
        dac_daemon.dac_write(1.0)
    assert bus.writes == []
    assert "DAC write failed" not in caplog.text
    assert "not found" in caplog.text


def test_no_i2c_bus_counts_as_absent(monkeypatch):
    monkeypatch.setattr(dac_daemon, "_bus", None)
    assert dac_daemon.dac_probe() is False
    dac_daemon.dac_write(2.5)          # must not raise


def test_present_dac_is_written(monkeypatch):
    bus = FakeBus(present=True)
    monkeypatch.setattr(dac_daemon, "_bus", bus)
    assert dac_daemon.dac_probe() is True
    dac_daemon.dac_set_neutral()
    assert len(bus.writes) == 1 and bus.writes[0][0] == dac_daemon.MCP4725_ADDR


def test_homing_wait_covers_homing_plus_hanging_reference():
    # GPIO 13 now goes HIGH only after homing (~10 s) AND the hanging reference
    assert dac_daemon.HOMING_TIMEOUT_S >= 60


def test_homing_timeout_pulses_stop(monkeypatch, tmp_path):
    pulses, published = [], []
    monkeypatch.setattr(dac_daemon, "stop_experiment", lambda: None)
    monkeypatch.setattr(dac_daemon, "MODELS_DIR", str(tmp_path))
    (tmp_path / "M.elf").write_text("")
    monkeypatch.setattr(dac_daemon, "gpio_pulse", lambda pin, *a: pulses.append(pin))
    monkeypatch.setattr(dac_daemon, "mqtt_publish", published.append)
    monkeypatch.setattr(dac_daemon.lgpio, "gpio_read", lambda h, pin: 0, raising=False)
    monkeypatch.setattr(dac_daemon, "HOMING_TIMEOUT_S", 0.2)
    dac_daemon.launch_experiment("M", {})
    assert pulses == [dac_daemon.GPIO_HOME_START, dac_daemon.GPIO_STOP]
    assert "error:homing_timeout" in published


class FakeMsg:
    topic = "pendulum/cmd"

    def __init__(self, payload):
        self.payload = payload.encode()


class FakeEsp:
    def __init__(self, starts=True):
        self.starts, self.running, self.calls = starts, False, []

    def start(self, exp):
        self.calls.append(("start", exp))
        self.running = self.starts
        return self.starts

    def stop_session(self, **kw):
        self.calls.append(("stop",))
        self.running = False


@pytest.fixture
def routes(monkeypatch):
    calls, published = [], []
    monkeypatch.setattr(dac_daemon, "launch_experiment", lambda exp, params: calls.append(("simulink", exp)))
    monkeypatch.setattr(dac_daemon, "gpio_pulse", lambda pin, *a: calls.append(("pulse", pin)))
    monkeypatch.setattr(dac_daemon, "mqtt_publish", published.append)
    monkeypatch.setattr(dac_daemon, "_model_process", None)
    return calls, published


def test_swinghold_runs_on_the_esp32_firmware(monkeypatch, routes):
    calls, _ = routes
    esp = FakeEsp()
    monkeypatch.setattr(dac_daemon, "_esp", esp)
    dac_daemon.on_message(None, None, FakeMsg('{"command": "sta", "experiment": "SwingHoldPendulum", "parameters": {}}'))
    assert esp.calls == [("start", "SwingHoldPendulum")]
    assert calls == []                                   # no Simulink launch, no start pulse


def test_other_experiments_still_run_on_simulink(monkeypatch, routes):
    calls, _ = routes
    esp = FakeEsp()
    monkeypatch.setattr(dac_daemon, "_esp", esp)
    dac_daemon.on_message(None, None, FakeMsg('{"command": "sta", "experiment": "PendSwingUp", "parameters": {}}'))
    assert calls == [("simulink", "PendSwingUp")] and esp.calls == []


def test_firmware_experiment_without_the_usb_port_reports_an_error(monkeypatch, routes):
    _, published = routes
    monkeypatch.setattr(dac_daemon, "_esp", None)
    dac_daemon.on_message(None, None, FakeMsg('{"command": "sta", "experiment": "SwingHoldPendulum"}'))
    assert "error:esp32_usb_missing" in published


def test_stop_ends_a_firmware_session(monkeypatch, routes):
    calls, _ = routes
    esp = FakeEsp()
    esp.running = True
    monkeypatch.setattr(dac_daemon, "_esp", esp)
    dac_daemon.on_message(None, None, FakeMsg('{"command": "sto"}'))
    assert esp.calls == [("stop",)]
    assert ("pulse", dac_daemon.GPIO_STOP) in calls


def test_pi_pins_match_the_rig_wiring():
    # Jumper wires from Pi pins 36/38/33 to the board's J2 (docs/esp32.md)
    assert (dac_daemon.GPIO_HOME_START, dac_daemon.GPIO_STOP, dac_daemon.GPIO_HOME_DONE) == (16, 20, 13)
