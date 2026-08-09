# ED2 Inverted Pendulum — System Documentation

Remote-controlled inverted pendulum lab. Students log into a web dashboard,
pick an experiment, tune PID parameters, and watch the real rig respond over a
live video feed — with the hardware sitting in the lab and the web server in
Oracle Cloud.

## Documents

| Doc | Covers |
|---|---|
| [architecture.md](architecture.md) | End-to-end data flow, every port and MQTT topic, sequence diagrams |
| [server-oracle.md](server-oracle.md) | The Oracle Cloud VM: build runbook, services, TLS, firewalls |
| [raspberry-pis.md](raspberry-pis.md) | Both Pis — camera Pi and control Pi |
| [esp32.md](esp32.md) | Both ESP32 firmwares — encoder and motor control |
| [operations.md](operations.md) | Day-to-day: deploying updates, smoke tests, troubleshooting |
| [open-questions.md](open-questions.md) | Known gaps, bugs found during the 2026-08 rebuild, decisions pending |

## The five computers

```
┌────────────────────────────────────────────────────────────────────┐
│  ORACLE CLOUD VM   129.153.42.213 / sciencelabtoyou.com            │
│  Ubuntu 24.04 aarch64, Ampere A1, 1 OCPU / 6 GB                    │
│                                                                    │
│   nginx :443  ──┬─ Django ASGI (gunicorn+uvicorn, unix socket)     │
│                 ├─ /ws/sensor/    → Channels → Redis               │
│                 └─ /stream/whep/  → MediaMTX :8889                 │
│   mosquitto :1885     MariaDB :3306     Redis :6379                │
│   MediaMTX  :8890/udp SRT in, :8189/udp WebRTC media out           │
└────────────────────────────────────────────────────────────────────┘
          ▲                    ▲                      ▲
          │ SRT video          │ MQTT                 │ reverse SSH
          │ :8890/udp          │ :1885                │ (admin access)
          │                    │                      │
┌─────────┴─────────┐  ┌───────┴──────────────────────┴──────────────┐
│  CAMERA PI        │  │  CONTROL PI          user: owlsley          │
│  user: skylar     │  │  dac_daemon.py — MQTT commands → hardware   │
│  rpicam-vid       │  │  Simulink .elf models, MCP4725 DAC          │
│  → ffmpeg → SRT   │  │                                             │
└───────────────────┘  └──────┬──────────────────────┬───────────────┘
                              │ SPI + GPIO           │ GPIO
                       ┌──────┴────────┐      ┌──────┴────────┐
                       │  ESP32 #1     │◄────►│  ESP32 #2     │
                       │  Encoders     │ UART │  Motor / BTS7960│
                       │  (PCNT, SPI)  │      │  ADS1115, limits│
                       └───────────────┘      └───────────────┘
```

## Quick orientation

- **Commands flow down:** browser → Django → MQTT `pendulum/cmd` → control Pi →
  GPIO/DAC → ESP32s → motor.
- **Telemetry flows up:** encoders → ESP32 #1 → SPI → control Pi → MQTT
  `raspi/to_django` → Django Channels → WebSocket → browser graph.
- **Video is a separate path entirely:** camera Pi → SRT → MediaMTX → WebRTC →
  browser. It never touches Django except for the `/stream/whep/` proxy.

Start with [architecture.md](architecture.md) if you're new to the system.
