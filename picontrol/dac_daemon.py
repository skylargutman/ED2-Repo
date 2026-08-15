#!/usr/bin/env python3
# =============================================================================
# Pi DAC Daemon
# Version: 2.0
# Description: Manages MCP4725 DAC and Pi GPIO pins for inverted pendulum system.
#              Sets DAC to 2.5V (motor neutral) at startup and whenever Simulink
#              is not running. Monitors Simulink process and reclaims DAC on exit
#              or crash. Pulses GPIO pins for homing and stop triggers.
#              MQTT subscriber runs in background thread — listens for commands
#              on pendulum/cmd and publishes status to pendulum/status.
#              Supports JSON commands to launch experiments with parameters.
#
# DAC:
#   MCP4725 at I2C bus 1, address 0x60
#   Vref measured at 5.14V — used for accurate voltage scaling
#   2.5V neutral = DAC value 2000 (calibrated)
#
# GPIO (Pi 5, using lgpio on gpiochip0):
#   GPIO  6  (Pin 31) → ESP32 #1 GPIO 33 : system_ready / reset (owned by Simulink)
#   GPIO 16  (Pin 36) → ESP32 #2 GPIO  4 : homing start trigger
#   GPIO 20  (Pin 38) → ESP32 #2 GPIO  5 : stop trigger
#
# MQTT:
#   Broker : sciencelabtoyou.com:1885 (anonymous)
#   Subscribe : pendulum/cmd    — JSON: {"command":"sta","experiment":"ModelName","parameters":{...}}
#                                 JSON: {"command":"sto"}
#                                 plain: "home"
#   Publish   : pendulum/status — payloads: "online", "offline", "homing",
#                                 "running:ModelName", "stopped"
#
# Experiment launch:
#   Parameters written to /home/owlsley/picontrol/params/params.json
#   Model launched as: sudo /home/owlsley/picontrol/models/<exp>.elf
#   Simulink reads params.json via Initialize Function block at startup
#
# Usage:
#   python3 dac_daemon.py
#   or as a systemd service (see bottom of file for unit file template)
# =============================================================================

import time
import subprocess
import lgpio
import smbus2
import logging
import sys
import json
import os
import paho.mqtt.client as mqtt

# =============================================================================
# Configuration
# =============================================================================

# I2C
I2C_BUS         = 1
MCP4725_ADDR    = 0x60
VREF            = 5.14      # measured/calibrated Vref on this specific MCP4725 module
NEUTRAL_VOLTAGE = 2.5       # volts — motor controller idle

# GPIO chip (Pi 5 uses gpiochip0 / pinctrl-rp1)
GPIO_CHIP       = 15 # this was previously 0, but it should be 15 for the Pi 5 ?? - alex
GPIO_SYSREADY   = 6         # → ESP32 #1 GPIO 33 : owned by Simulink, not claimed here
GPIO_HOME_START = 16        # → ESP32 #2 GPIO  4 : homing start trigger
GPIO_STOP       = 20        # → ESP32 #2 GPIO  5 : stop trigger
GPIO_HOME_DONE  = 13        # ← ESP32 #2 GPIO 18 : homing complete signal (input)

PULSE_MS        = 200       # GPIO pulse duration in milliseconds
HOMING_TIMEOUT_S = 30      # max seconds to wait for homing complete signal

# Simulink process name to watch for (updated dynamically when experiment launches)
SIMULINK_PROCESS_NAME = ""  # empty until first experiment launched

# Model and params paths
MODELS_DIR  = "/home/owlsley/picontrol/models/MATLAB_ws/R2025b"
PARAMS_FILE = "/home/owlsley/picontrol/params/params.json"

# Polling interval for Simulink process watch
POLL_INTERVAL_S = 1.0

# MQTT
MQTT_BROKER            = "sciencelabtoyou.com"
MQTT_PORT              = 1885
MQTT_CMD               = "pendulum/cmd"
MQTT_STATUS            = "pendulum/status"

# =============================================================================
# Logging
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [DAC DAEMON] %(levelname)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger(__name__)

# =============================================================================
# Shared state — accessed by both main thread and MQTT thread
# =============================================================================
_bus            = None   # smbus2.SMBus — set in run_daemon()
_gpio_handle    = None   # lgpio handle — set in run_daemon()
_mqtt_client    = None   # paho client  — set in setup_mqtt()
_current_exp    = None   # currently running experiment name (no .elf)
_model_process  = None   # subprocess.Popen handle for running model

# =============================================================================
# MCP4725 DAC helpers
# =============================================================================

def voltage_to_dac(volts: float) -> int:
    """Convert voltage to 12-bit DAC value using measured Vref."""
    raw = int((volts / VREF) * 4096)
    return max(0, min(4095, raw))

def dac_write(volts: float):
    """Write voltage to MCP4725. Fast write mode (no EEPROM)."""
    val = voltage_to_dac(volts)
    upper = (val >> 4) & 0xFF
    lower = (val & 0x0F) << 4
    try:
        _bus.write_i2c_block_data(MCP4725_ADDR, 0x40, [upper, lower])
        log.info(f"DAC set to {volts:.3f}V (raw={val})")
    except Exception as e:
        log.error(f"DAC write failed: {e}")

def dac_set_neutral():
    """Set DAC to motor neutral (2.5V)."""
    dac_write(NEUTRAL_VOLTAGE)

# =============================================================================
# GPIO helpers
# =============================================================================

def gpio_pulse(pin: int, duration_ms: int = PULSE_MS):
    """Pulse a GPIO pin HIGH for duration_ms then back LOW."""
    try:
        lgpio.gpio_write(_gpio_handle, pin, 1)
        time.sleep(duration_ms / 1000.0)
        lgpio.gpio_write(_gpio_handle, pin, 0)
        log.info(f"GPIO {pin} pulsed HIGH for {duration_ms}ms")
    except Exception as e:
        log.error(f"GPIO pulse failed on pin {pin}: {e}")

def gpio_setup(handle):
    """Configure output pins, drive LOW initially.
    NOTE: GPIO_SYSREADY (GPIO 6) is owned by Simulink GPIO Write block.
    Daemon only claims the ESP32 #2 trigger pins and homing done input."""
    for pin in [GPIO_HOME_START, GPIO_STOP]:
        lgpio.gpio_claim_output(handle, pin, 0)
    lgpio.gpio_claim_input(handle, GPIO_HOME_DONE)
    log.info("GPIO pins configured (HOME_START=16, STOP=20, HOME_DONE=13 input)")

# =============================================================================
# MQTT helpers
# =============================================================================

def mqtt_publish(payload: str):
    """Publish a status message to pendulum/status."""
    if _mqtt_client is not None:
        try:
            _mqtt_client.publish(MQTT_STATUS, payload, qos=1, retain=True)
            log.info(f"MQTT published → {MQTT_STATUS}: {payload}")
        except Exception as e:
            log.error(f"MQTT publish failed: {e}")

# =============================================================================
# Experiment management
# =============================================================================

def write_params(params: dict):
    """Write experiment parameters to params.json for Simulink to read."""
    try:
        os.makedirs(os.path.dirname(PARAMS_FILE), exist_ok=True)
        with open(PARAMS_FILE, "w") as f:
            json.dump(params, f, indent=2)
        log.info(f"Params written to {PARAMS_FILE}: {params}")
    except Exception as e:
        log.error(f"Failed to write params: {e}")

def launch_experiment(exp_name: str, params: dict):
    """Stop any running model, home cart, wait for homing complete signal,
    write params, launch new experiment."""
    global _current_exp, _model_process, SIMULINK_PROCESS_NAME

    # Stop any currently running model first
    stop_experiment()

    elf_path = os.path.join(MODELS_DIR, f"{exp_name}.elf")
    if not os.path.isfile(elf_path):
        log.error(f"Model not found: {elf_path}")
        mqtt_publish(f"error:model_not_found:{exp_name}")
        return

    # Trigger homing on ESP32 #2
    log.info("Triggering homing before experiment launch...")
    mqtt_publish("homing")
    gpio_pulse(GPIO_HOME_START)

    # Wait for homing complete signal on GPIO 13 with timeout
    log.info(f"Waiting for homing complete signal (timeout={HOMING_TIMEOUT_S}s)...")
    start_time = time.time()
    homing_ok = False
    while time.time() - start_time < HOMING_TIMEOUT_S:
        if lgpio.gpio_read(_gpio_handle, GPIO_HOME_DONE) == 1:
            homing_ok = True
            log.info("Homing complete signal received on GPIO 13")
            break
        time.sleep(0.1)

    if not homing_ok:
        log.error("Homing timed out — experiment not launched")
        mqtt_publish("error:homing_timeout")
        return

    # Write parameters for Simulink to read at startup
    write_params(params)

    # Launch model as sudo
    try:
        _model_process = subprocess.Popen(
            ["sudo", elf_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        _current_exp = exp_name
        SIMULINK_PROCESS_NAME = f"{exp_name}.elf"
        log.info(f"Launched experiment: {exp_name} (PID {_model_process.pid})")
        mqtt_publish(f"running:{exp_name}")
    except Exception as e:
        log.error(f"Failed to launch {elf_path}: {e}")
        mqtt_publish(f"error:launch_failed:{exp_name}")

def stop_experiment():
    """Kill running model process, set DAC neutral.
    NOTE: GPIO 20 is NOT pulsed here — ESP32 #2 stop is only for explicit
    stop commands, not for model handoff during experiment launch."""
    global _current_exp, _model_process, SIMULINK_PROCESS_NAME

    if _model_process is not None:
        try:
            subprocess.run(["sudo", "kill", str(_model_process.pid)],
                           capture_output=True)
            _model_process.wait(timeout=3)
            log.info(f"Stopped experiment: {_current_exp}")
        except Exception as e:
            log.error(f"Failed to stop model: {e}")
        finally:
            _model_process        = None
            _current_exp          = None
            SIMULINK_PROCESS_NAME = ""

    dac_set_neutral()
    mqtt_publish("stopped")

def trigger_stop():
    """Stop running experiment, set DAC neutral, pulse GPIO 20 to stop ESP32 #2."""
    log.info("Stop triggered via MQTT")
    stop_experiment()
    gpio_pulse(GPIO_STOP)  # only pulse ESP32 #2 on explicit stop command

def trigger_homing_start():
    """Pulse GPIO 16 to tell ESP32 #2 to begin homing."""
    log.info("Homing start triggered via MQTT")
    mqtt_publish("homing")
    gpio_pulse(GPIO_HOME_START)

# =============================================================================
# MQTT callbacks
# =============================================================================

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        log.info(f"MQTT connected to {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe(MQTT_CMD, qos=1)
        log.info(f"Subscribed to {MQTT_CMD}")
        mqtt_publish("online")
    else:
        log.error(f"MQTT connection failed: reason code {reason_code}")

def on_disconnect(client, userdata, flags, reason_code, properties):
    log.warning(f"MQTT disconnected (reason={reason_code}) — will reconnect automatically")

def on_message(client, userdata, msg):
    """Handle incoming commands on pendulum/cmd.

    Supported formats:
      Plain string : "home"
      JSON         : {"command": "sta", "experiment": "ModelName", "key": {...}}
      JSON         : {"command": "sto"}
    """
    try:
        raw = msg.payload.decode("utf-8").strip()
        log.info(f"MQTT received ← {msg.topic}: {raw}")

        # Try JSON first
        try:
            data = json.loads(raw)
            cmd = data.get("command", "").lower()

            if cmd == "sta":
                exp    = data.get("experiment", "")
                params = data.get("parameters", {})
                if not exp:
                    log.error("sta command missing 'experiment' field")
                    return
                launch_experiment(exp, params)

            elif cmd == "sto":
                trigger_stop()

            elif cmd == "home":
                trigger_homing_start()

            else:
                log.warning(f"Unknown JSON command: {command}")

        except json.JSONDecodeError:
            # Fall back to plain string commands
            plain = raw.lower()
            if plain == "home":
                trigger_homing_start()
            elif plain == "sto":
                trigger_stop()
            else:
                log.warning(f"Unknown plain command: {plain}")

    except Exception as e:
        log.error(f"MQTT message handler error: {e}")

# =============================================================================
# MQTT setup — loop_start() runs network loop in background thread
# =============================================================================

def setup_mqtt() -> mqtt.Client:
    """Create and connect MQTT client with background network loop."""
    global _mqtt_client

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id="pi_dac_daemon"
    )

    # Last will — published by broker if daemon disconnects unexpectedly
    client.will_set(MQTT_STATUS, payload="offline", qos=1, retain=True)

    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_message    = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except Exception as e:
        log.error(f"MQTT initial connect failed: {e} — will retry in background")

    # Background thread handles network I/O and auto-reconnect
    client.loop_start()

    _mqtt_client = client
    return client

# =============================================================================
# Simulink process detection
# =============================================================================

def simulink_is_running() -> bool:
    """Return True if the current experiment .elf process is running."""
    if not SIMULINK_PROCESS_NAME:
        return False
    try:
        result = subprocess.run(
            ["pgrep", "-f", SIMULINK_PROCESS_NAME],
            capture_output=True,
            text=True
        )
        return result.returncode == 0
    except Exception as e:
        log.error(f"Process check failed: {e}")
        return False

# =============================================================================
# Main daemon logic
# =============================================================================

def run_daemon():
    global _bus, _gpio_handle

    log.info("=== Pi DAC Daemon v2.0 starting ===")

    # --- Open I2C bus ---
    try:
        _bus = smbus2.SMBus(I2C_BUS)
        log.info(f"I2C bus {I2C_BUS} opened")
    except Exception as e:
        log.error(f"Failed to open I2C bus: {e}")
        sys.exit(1)

    # --- Open GPIO chip ---
    try:
        _gpio_handle = lgpio.gpiochip_open(GPIO_CHIP)
        log.info(f"GPIO chip {GPIO_CHIP} opened")
    except Exception as e:
        log.error(f"Failed to open GPIO chip {GPIO_CHIP}: {e}")
        sys.exit(1)

    # --- Configure GPIO pins ---
    gpio_setup(_gpio_handle)

    # --- Set DAC to neutral immediately at power-up ---
    log.info("Power-up: setting DAC to neutral (2.5V)")
    dac_set_neutral()

    # --- Start MQTT background thread ---
    mqtt_client = setup_mqtt()

    # --- State tracking ---
    simulink_was_running = False

    log.info("Daemon running. Monitoring Simulink process and MQTT commands...")

    try:
        while True:
            simulink_running = simulink_is_running()

            if simulink_running and not simulink_was_running:
                log.info(f"Experiment running: {_current_exp} — DAC handed off")
                simulink_was_running = True

            elif not simulink_running and simulink_was_running:
                log.info(f"Experiment stopped/crashed: {_current_exp} — reclaiming DAC")
                dac_set_neutral()
                mqtt_publish("stopped")
                simulink_was_running = False

            time.sleep(POLL_INTERVAL_S)

    except KeyboardInterrupt:
        log.info("Keyboard interrupt received — shutting down")

    finally:
        # --- Clean shutdown ---
        log.info("Shutdown: setting DAC to neutral and releasing GPIO")
        mqtt_publish("offline")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        dac_set_neutral()
        for pin in [GPIO_HOME_START, GPIO_STOP]:
            lgpio.gpio_write(_gpio_handle, pin, 0)
        lgpio.gpiochip_close(_gpio_handle)
        _bus.close()
        log.info("=== Pi DAC Daemon stopped ===")

# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    run_daemon()

