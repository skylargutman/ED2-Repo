# Open Questions & Known Issues

Found while rebuilding the server in August 2026. Ordered by how likely each is
to bite during the redeploy.

---

## 1. 🔴 Nothing in this repo publishes telemetry to `raspi/to_django`

**The live sensor graph has no identified data source.**

`WebInterface/mqtt_subscriber.py` subscribes to `raspi/to_django` and is the
only thing feeding the `sensor_data` channel group. But searching the entire
repo for a publisher to that topic finds nothing:

- `dac_daemon.py` publishes only to `pendulum/status`
- `RemoteControl.cpp` only subscribes
- the ESP32s have no networking at all

So the publisher is either a Simulink MQTT block inside the `.elf` models, or a
script on the control Pi that was never committed.

**Needed:** on the control Pi, run
`mosquitto_sub -h sciencelabtoyou.com -p 1885 -t 'raspi/to_django' -v` during an
experiment. If data appears, find the publisher and commit it. If it doesn't,
the graph never worked and needs building.

## 2. ✅ `mqtt_subscriber.py` migrated to the paho 2.x callback API — RESOLVED

The script previously called a bare `mqtt.Client()` with v1-style
`on_connect(client, userdata, flags, rc)`.

> **Correction:** an earlier revision of this document claimed that would raise
> `ValueError` on paho-mqtt 2.x. That is wrong. Checking the paho 2.1.0 source,
> `callback_api_version` **defaults to `CallbackAPIVersion.VERSION1`**:
>
> ```python
> def __init__(
>     self,
>     callback_api_version: CallbackAPIVersion = CallbackAPIVersion.VERSION1,
>     ...
> ```
>
> The `ValueError` fires only when the first positional argument is a *string*
> — a guard for people passing `client_id` the 1.x way. A bare `mqtt.Client()`
> emits a `DeprecationWarning` and works fine. **This was therefore never a
> startup crash, and is not an explanation for issue #1.**

Migrated to `CallbackAPIVersion.VERSION2` anyway, because v1 is deprecated and
scheduled for removal, and because `dac_daemon.py` on the Pi already uses v2 —
having both halves of the same bus on the same API is worth the small change.

Also picked up while in there:
- subscriptions moved inside `on_connect`, so they are restored automatically
  after a reconnect rather than being lost
- `reconnect_delay_set(1, 60)` backoff instead of hammering a down broker
- `connect_async` + `loop_forever(retry_first_connection=True)`, so the service
  survives starting before mosquitto at boot
- broker host/port/topics read from `deploy/.env`, defaulting to loopback

## 3. ✅ Nothing consumed `pendulum/status` — RESOLVED

`dac_daemon.py` publishes rich retained status — `online`, `offline` (via Last
Will), `homing`, `running:<Model>`, `stopped`, `error:model_not_found:<name>` —
and no server-side subscriber existed, so the dashboard could not show whether
the rig was powered on, mid-homing, or already running someone else's
experiment.

Now wired end to end:

| Layer | Change |
|---|---|
| `mqtt_subscriber.py` | subscribes to `pendulum/status` (QoS 1), parses it via `parse_status()` |
| `consumers.py` | new `rig_status` handler on `SensorDataConsumer` |
| `experiment_run_dynamic.html` | `#rigStatus` badge + `updateRigStatus()` |

Because the Pi publishes with `retain=True`, a freshly-opened dashboard learns
the current state immediately instead of waiting for the next transition.

Two design notes:

- Status messages are **not** written to the `Message` table. That table backs
  the sensor charts and the data-history page; status strings would pollute
  both, and the broker already retains them.
- Status shares the existing `/ws/sensor/` socket, tagged `type: "rig_status"`,
  and the client routes on that field *before* the charting path. Sensor
  telemetry keeps its original wire format, so nothing else needed changing.
  This assumes telemetry payloads never contain a top-level `type` key — worth
  confirming once issue #1 identifies the publisher.

## 4. 🟡 Secrets are in git history

| Secret | Location |
|---|---|
| `SECRET_KEY` | `EGNSite/settings.py`, `production_settings.py` |
| MySQL password `Kings1305` | `production_settings.py:31` |

Now overridable via `deploy/.env`, and the new deployment uses fresh values —
but the old ones remain in history. If this repo is or becomes public, treat
both as burned. Rotating (done) is sufficient; scrubbing history is optional.

## 5. 🟡 Mosquitto is anonymous and world-reachable

Port 1885 accepts anonymous publishes from any IP. Anyone who finds it can
publish to `pendulum/cmd` and start experiments on real hardware with real
motors.

Options, cheapest first:

1. **Restrict by source IP** in the OCI Security List to the campus egress
   range — no code changes.
2. **`password_file`** in mosquitto plus credentials in `deploy/.env` and on
   the Pi — small change to `mqtt_utils.py` and `dac_daemon.py`.
3. **TLS on 8885** with certs — most work, best posture.

Option 1 is probably the right trade for a lab rig, if the campus IP is stable.

## 6. 🟡 `production_settings.py` is stale and unused

It sets `ALLOWED_HOSTS = ['domain.com', 'pc-ip-address', ...]` — placeholders,
never filled in — and a different DB engine (`mysql.connector.django`) than
`settings.py` uses (`django.db.backends.mysql` + pymysql). Nothing imports it;
`manage.py`, `asgi.py`, and `wsgi.py` all point at `EGNSite.settings`.

Now that `settings.py` reads `deploy/.env`, this file is dead weight and should
probably be deleted to stop someone deploying with it by mistake.

## 7. 🟢 `dac_daemon.py` — NameError on unknown JSON command

```python
else:
    log.warning(f"Unknown JSON command: {command}")   # `command` is undefined
```

The variable is `cmd`. Only reachable on an unrecognised command, and the outer
`try` swallows it — so the effect is a lost warning, not a crash. Worth fixing
next time that file is touched.

## 8. 🟢 Model path disagreement

| File | Path |
|---|---|
| `dac_daemon.py` | `/home/owlsley/picontrol/models/MATLAB_ws/R2025b` |
| `RemoteControl.cpp` | `/home/owlsley/picontrol/MATLAB_ws/R2025b` |

`dac_daemon.py` is the one in service. Worth confirming which directory
actually holds the `.elf` files, and correcting the other.

## 9. 🟢 `RemoteControl.cpp` has an unconditional early `return`

`dispatch_to_simulink()` returns before its `system()` call, so it prints
parameters and never launches a model. Consistent with the file being a parked
alternative to `dac_daemon.py` rather than live code — but it should be either
finished, or clearly marked dead so nobody deploys it.

## 10. 🟢 Lost management-command sources

`MatlabApp/management/commands/` contains `mqtt_listener.cpython-314.pyc` and
`mqtt_subscriber.cpython-314.pyc` with no matching `.py`. Recovered strings
show they are **legacy** — topics `djangotest` and `matlab/to_django`, broker
port 1883 — superseded by the root `mqtt_subscriber.py` on port 1885. Nothing
of value is lost; the `.pyc` files are now gitignored.

---

## Decisions still needed

1. **Find the `raspi/to_django` publisher** (issue #1) — still the one thing
   blocking the live graph.
2. Lock down MQTT — IP allowlist, passwords, or leave open? (issue #5)
3. Delete `production_settings.py`? (issue #6)
4. Should the camera Pi's `stream.sh` and `autossh-tunnel.service` in
   `picamera/` be updated in git to `ubuntu@`, so the repo matches reality?
