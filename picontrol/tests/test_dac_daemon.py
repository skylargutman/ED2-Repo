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
