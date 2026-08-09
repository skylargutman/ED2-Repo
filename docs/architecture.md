# Architecture

## Port map

Everything that listens, and who is allowed to reach it.

### Oracle Cloud VM — `129.153.42.213` / `sciencelabtoyou.com`

| Port | Proto | Service | Exposure | Purpose |
|---|---|---|---|---|
| 22 | TCP | sshd | Public | Admin + the camera Pi's reverse tunnel |
| 80 | TCP | nginx | Public | ACME challenge, redirect to 443 |
| 443 | TCP | nginx | Public | The site, `/ws/sensor/`, `/stream/whep/` |
| 1885 | TCP | mosquitto | Public | MQTT bus — control Pi connects in |
| 8890 | **UDP** | MediaMTX | Public | SRT video ingest from camera Pi |
| 8189 | **UDP** | MediaMTX | Public | WebRTC media to browsers |
| 8889 | TCP | MediaMTX | **Loopback** | WHEP signalling — nginx proxies it |
| 6379 | TCP | Redis | **Loopback** | Channels layer |
| 3306 | TCP | MariaDB | **Loopback** | `egnsitedb` |
| — | unix | gunicorn | `/run/egnsite/gunicorn.sock` | Django ASGI |

Two independent firewalls guard these: the **OCI Security List** (cloud side)
and the instance's own **iptables** rules. A port must be open in both. See
[server-oracle.md](server-oracle.md).

### Control Pi (`owlsley`)

Listens on nothing. Outbound MQTT to `sciencelabtoyou.com:1885` only.

### Camera Pi (`skylar`)

Listens on nothing. Outbound SRT to `:8890` and an autossh reverse tunnel that
exposes the Pi's own port 22 as `localhost:2222` **on the server**.

## MQTT topics

Broker: `sciencelabtoyou.com:1885`, anonymous.

| Topic | Direction | Payload | Published by | Consumed by |
|---|---|---|---|---|
| `pendulum/cmd` | web → Pi | JSON | `MatlabApp/mqtt_utils.py` | `picontrol/dac_daemon.py` |
| `pendulum/status` | Pi → * | plain string | `dac_daemon.py` | ⚠️ **nothing** — see below |
| `raspi/to_django` | Pi → web | JSON | ⚠️ **unknown** — see below | `WebInterface/mqtt_subscriber.py` |

### `pendulum/cmd` payloads

Built in [`MatlabApp/views.py`](../WebInterface/MatlabApp/views.py) →
`send_experiment_command`, published by `mqtt_utils.send_command()`:

```jsonc
// Start an experiment
{"command": "sta", "experiment": "SwingHoldPendulum",
 "parameters": {"PID_D_proportional": 70, "PID_D_integral": 0.5, ...}}

// Stop
{"command": "sto"}

// Home the cart
{"command": "home"}      // also accepted as the bare string "home"
```

`dac_daemon.py` parses JSON first and falls back to plain-string commands.

### `pendulum/status` payloads

`online`, `offline` (MQTT Last Will), `homing`, `running:<ModelName>`,
`stopped`, `error:model_not_found:<name>`. Published with `retain=True`, so a
newly-connected subscriber immediately learns the rig's current state.

> ⚠️ **Nothing on the server subscribes to this topic.** The dashboard cannot
> currently show whether the rig is online, homing, or running. This is a
> genuine feature gap, not a misconfiguration. See
> [open-questions.md](open-questions.md).

## Data flow 1 — sending a command

```
Browser                Django                 mosquitto        Control Pi
   │                     │                        │                │
   │ POST /experiment/   │                        │                │
   │   <name>/command/   │                        │                │
   ├────────────────────►│                        │                │
   │                     │ @requires_control_lock │                │
   │                     │   (one user at a time, │                │
   │                     │    60 s heartbeat)     │                │
   │                     │                        │                │
   │                     │ publish pendulum/cmd   │                │
   │                     ├───────────────────────►│                │
   │                     │                        ├───────────────►│
   │                     │                        │                │ write params.json
   │                     │                        │                │ pulse GPIO 16 (home)
   │                     │                        │                │ wait GPIO 13 (done)
   │                     │                        │                │ launch <exp>.elf
   │ ◄───────────────────┤ JsonResponse           │                │
   │   {success: true}   │                        │                │
```

**The control lock** ([`views.py`](../WebInterface/MatlabApp/views.py)) is what
stops two students from fighting over one physical rig. A `ControlLock` row
holds a session key; the browser heartbeats every few seconds; a lock older
than `LOCK_TIMEOUT = 60` seconds is considered stale and can be taken over.
Instructors can seize it with `/control/estop/`. Accounts flagged `is_viewer`
can never acquire it — that's how the `showcase` demo account stays read-only.

## Data flow 2 — live sensor telemetry

```
Encoders → ESP32 #1 ──SPI──► Control Pi ──MQTT `raspi/to_django`──► server
                                                                      │
                                          mqtt_subscriber.py ◄────────┘
                                                │
                                    ┌───────────┴───────────┐
                                    ▼                       ▼
                            Message table            channel_layer.group_send
                            (capped at 5000)          'sensor_data'
                                                            │
                                                            ▼
                                                   SensorDataConsumer
                                                            │
                                             wss://…/ws/sensor/ → browser graph
```

`mqtt_subscriber.py` is a standalone script run by `egnsite-mqtt.service` — not
a `manage.py` subcommand. It bootstraps Django itself (`django.setup()`) so it
can use the ORM and the channel layer.

Two details worth knowing:

- It calls `connection.close()` on every message to force a fresh DB
  connection. That's defending against MySQL's `wait_timeout` killing an idle
  connection on a rig that sits unused between lab sessions.
- `parse_payload()` tries strict JSON, then retries after regex-quoting bare
  keys (`{foo: 1}` → `{"foo": 1}`), then falls back to `{'raw': text}`. That
  tolerance exists because MATLAB-side publishers emit near-JSON.

## Data flow 3 — video

```
Camera Pi                          Oracle VM                      Browser
   │                                   │                             │
   │ rpicam-vid → ffmpeg               │                             │
   │ (H.264, 640x480, 30fps,           │                             │
   │  2 Mbit, IDR every 15 frames)     │                             │
   │                                   │                             │
   ├──SRT :8890 streamid=              │                             │
   │   publish:pendulum ──────────────►│ MediaMTX                    │
   │                                   │  path "pendulum"            │
   │                                   │                             │
   │                                   │◄──── POST /stream/whep/ ────┤
   │                                   │      (SDP offer)            │
   │                                   ├──── SDP answer ────────────►│
   │                                   │                             │
   │                                   │◄═══ WebRTC media :8189/udp ═►│
```

The Pi does all H.264 encoding; the server only remuxes (`ffmpeg -c:v copy`).
That's why a 1-OCPU instance is sufficient.

**Why WebRTC and not HLS:** the template comment in
`experiment_run_dynamic.html` records the switch — HLS added seconds of
latency, which makes a balance-control demo useless. The player also sets
`jitterBufferTarget = 0` to shave buffering further.

## Repository layout

| Path | Runs on | Contents |
|---|---|---|
| `WebInterface/` | Oracle VM | Django project `EGNSite`, apps `MatlabApp` + `accounts` |
| `picontrol/` | Control Pi | `dac_daemon.py`, `RemoteControl.cpp` |
| `picamera/` | Camera Pi | `stream.sh`, systemd units, stock `mediamtx.yml` |
| `ESP32 code/` | ESP32s | `esp32_encoder_v9.ino`, `esp32_motor_v9.5.ino` |
| `MATLAB/` | Dev machine | Simulink models, `*PiMod2.slx` = Pi-targeted variants |
| `simulink-param-retrieval/` | Dev machine | Generated code for reading `params.json` into models |
| `KiCad/` | — | Pendulum PCB and 10-pin box adapter |
| `deploy/` | Oracle VM | nginx, systemd, mosquitto, mediamtx configs + bootstrap |
| `docs/` | — | This documentation |
