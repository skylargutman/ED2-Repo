# The Raspberry Pis

There are **two**, with different users and different jobs. They are easy to
confuse because both talk to the same server.

| | Camera Pi | Control Pi |
|---|---|---|
| Linux user | `skylar` | `owlsley` |
| Repo folder | `picamera/` | `picontrol/` |
| Job | H.264 video → server | Run experiments, drive hardware |
| Talks to server via | SRT :8890, SSH :22 | MQTT :1885 |
| Hardware | Pi camera module | MCP4725 DAC, GPIO to both ESP32s |

---

# Camera Pi (`skylar`)

## `pistream.service` → `stream.sh`

Captures with `rpicam-vid` and pushes MPEG-TS over SRT:

```
640x480 @ 30fps, H.264, 2 Mbit, --intra 15, --inline
  → ffmpeg -c:v copy -f mpegts
  → srt://sciencelabtoyou.com:8890?streamid=publish:pendulum&latency=50000
```

Notes on the tuning, since these values are deliberate:

- `--intra 15` puts a keyframe every half-second. A browser can't start
  decoding until it sees one, so this bounds join latency.
- `--inline` repeats SPS/PPS headers with every keyframe — required, otherwise
  a viewer joining mid-stream never gets the decoder configuration.
- `-c:v copy` means ffmpeg never re-encodes. The Pi's hardware encoder does the
  work and the server just relays.
- `latency=50000` is SRT's 50 ms receiver buffer, trading a little delay for
  tolerance of campus wifi jitter.
- The whole thing is wrapped in a `while true` loop with a 5 s retry, so a
  dropped network comes back without systemd having to restart the unit.

The path name **`pendulum`** must match the `paths:` entry in the server's
`mediamtx.yml` and the `/pendulum/whep` target in the nginx config.

## `autossh-tunnel.service`

Maintains a reverse SSH tunnel so you can reach the Pi behind campus NAT:

```
autossh -M 0 -N -R 2222:localhost:22 -i /home/skylar/.ssh/id_ed25519 opc@sciencelabtoyou.com
```

From the server, `ssh -p 2222 skylar@localhost` then reaches the Pi.

> ⚠️ **Must be updated for the new server.** The unit says `opc@` — that user
> exists on Oracle Linux, not on Ubuntu 24.04. Change to `ubuntu@`:
>
> ```bash
> sudo sed -i 's/opc@sciencelabtoyou.com/ubuntu@sciencelabtoyou.com/' \
>     /etc/systemd/system/autossh-tunnel.service
> sudo systemctl daemon-reload && sudo systemctl restart autossh-tunnel
> ```
>
> The Pi's public key (`/home/skylar/.ssh/id_ed25519.pub`) must also be added
> to `/home/ubuntu/.ssh/authorized_keys` on the new server, since the old
> server's host identity and authorized keys are gone.

`-R 2222:localhost:22` binds to the server's loopback only, so the tunnel isn't
exposed to the internet. `ExitOnForwardFailure=yes` makes autossh tear down and
retry if the port is already held by a stale session.

Both units use `ExecStartPre=/bin/sleep` (15 s / 10 s) to let wifi and DNS come
up before connecting.

---

# Control Pi (`owlsley`)

## `dac_daemon.py` — the main program

Runs as a systemd service (template at the bottom of the file). Responsibilities:

1. Hold the motor at **neutral** whenever no experiment is running.
2. Subscribe to `pendulum/cmd` and launch/stop Simulink models.
3. Pulse GPIO to trigger homing and stops on ESP32 #2.
4. Watch the running model and reclaim the DAC if it exits or crashes.

### Hardware interfaces

**MCP4725 DAC** — I²C bus 1, address `0x60`.

```
Vref  = 5.14 V   ← measured on this specific module, not nominal 5.0
2.5 V = raw 2000 ← motor-controller neutral
```

`voltage_to_dac()` scales by that measured Vref. **If the DAC module is ever
replaced, re-measure Vref and update the constant**, or neutral will sit off
-centre and the cart will creep.

**GPIO** (Pi 5, `lgpio` on `gpiochip0`):

| Pi GPIO | Pin | Direction | Goes to | Meaning |
|---|---|---|---|---|
| 6 | 31 | out | ESP32 #1 GPIO 33 | `system_ready` — **owned by Simulink**, not the daemon |
| 16 | 36 | out | ESP32 #2 GPIO 4 | homing start trigger |
| 20 | 38 | out | ESP32 #2 GPIO 5 | stop trigger |
| 13 | 33 | in | ESP32 #2 GPIO 18 | homing complete |

GPIO 6 is deliberately *not* claimed by the daemon — the Simulink model's GPIO
Write block owns it. Claiming it in both places causes a resource conflict.

Triggers are 200 ms pulses (`PULSE_MS`).

### Experiment launch sequence

```
receive {"command":"sta","experiment":X,"parameters":{...}}
   │
   ├─ stop_experiment()          kill any running model
   ├─ verify <X>.elf exists      else publish error:model_not_found:X
   ├─ publish "homing"
   ├─ pulse GPIO 16              tell ESP32 #2 to home the cart
   ├─ wait for GPIO 13 HIGH      timeout HOMING_TIMEOUT_S = 30 s
   ├─ write params.json
   └─ sudo <MODELS_DIR>/<X>.elf
```

Homing happens **before every run** — the cart must be centred and the encoder
zeroed or the controller has no valid reference.

### Paths

```
MODELS_DIR  = /home/owlsley/picontrol/models/MATLAB_ws/R2025b
PARAMS_FILE = /home/owlsley/picontrol/params/params.json
```

> ⚠️ `RemoteControl.cpp` uses `/home/owlsley/picontrol/MATLAB_ws/R2025b` —
> without the `models/` segment. The two disagree; `dac_daemon.py` is the one
> in service.

The Simulink model reads `params.json` at startup via an Initialize Function
block. See `simulink-param-retrieval/` for the generated code that does this.

### MQTT client

Uses **paho-mqtt 2.x** correctly:

```python
mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="pi_dac_daemon")
client.will_set(MQTT_STATUS, payload="offline", qos=1, retain=True)
client.loop_start()   # background thread, auto-reconnect
```

The Last Will means that if the Pi loses power or network, the broker publishes
`offline` on its behalf — the server learns the rig is gone without polling.

## `RemoteControl.cpp`

A C++ implementation of the same MQTT subscriber (libmosquitto + nlohmann/json)
that also subscribes to `pendulum/cmd` and dispatches Simulink models.

**It appears superseded by `dac_daemon.py`** and is not the daemon in service:

- `dispatch_to_simulink()` contains an unconditional `return` *before* the
  `system()` call that would launch the model — so it prints parameters and
  exits without ever starting anything. That reads like debugging left in.
- It has no DAC handling, no homing sequence, and no status publishing.

Treat it as reference or a parked alternative, not as running code.
