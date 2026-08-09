# Operations Runbook

## Deploying an update

The server runs a checkout of this repo at `/opt/ed2/ED2-Repo`:

```bash
ssh ubuntu@sciencelabtoyou.com
cd /opt/ed2/ED2-Repo
git pull

# Only if requirements.txt changed
/opt/ed2/venv/bin/pip install -r WebInterface/requirements.txt

# Only if models/migrations changed
cd WebInterface
set -a; . /opt/ed2/ED2-Repo/deploy/.env; set +a
/opt/ed2/venv/bin/python manage.py migrate

# Only if templates/static changed
/opt/ed2/venv/bin/python manage.py collectstatic --noinput

sudo systemctl restart egnsite-web egnsite-mqtt
```

The venv lives at `/opt/ed2/venv` — **outside** the repo — so `git pull` can
never collide with it.

If you changed anything in `deploy/`, reinstall it:

```bash
sudo install -m 0644 deploy/nginx/sciencelabtoyou.conf /etc/nginx/sites-available/sciencelabtoyou
sudo nginx -t && sudo systemctl reload nginx

sudo install -m 0644 deploy/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload

sudo install -m 0644 deploy/mediamtx/mediamtx.yml /usr/local/etc/mediamtx.yml
sudo systemctl restart mediamtx
```

## Smoke test

Work down the list — each step depends on the ones above it.

```bash
# 1. All services up
systemctl is-active nginx egnsite-web egnsite-mqtt mediamtx mosquitto redis-server mariadb

# 2. Django responds behind nginx
curl -sI https://sciencelabtoyou.com/login/ | head -1        # expect 200

# 3. TLS valid
echo | openssl s_client -connect sciencelabtoyou.com:443 2>/dev/null \
  | openssl x509 -noout -dates

# 4. Redis reachable (Channels depends on it)
redis-cli ping                                               # expect PONG

# 5. MQTT broker accepting connections
mosquitto_sub -h localhost -p 1885 -t 'pendulum/#' -v -C 1 -W 5

# 6. Control Pi is alive — retained status message appears instantly
mosquitto_sub -h localhost -p 1885 -t 'pendulum/status' -C 1 -W 5

# 7. Telemetry bridge is receiving
journalctl -u egnsite-mqtt -n 30 --no-pager

# 8. Video stream is being published
journalctl -u mediamtx -n 30 --no-pager | grep -i "is publishing"
```

Then in a browser: log in → open an experiment → confirm the video tile shows
**LIVE** and the sensor graph is moving.

## Troubleshooting

### 502 Bad Gateway

gunicorn is down or the socket is missing.

```bash
systemctl status egnsite-web
journalctl -u egnsite-web -n 50 --no-pager
ls -l /run/egnsite/gunicorn.sock
```

Common causes: a bad `deploy/.env` (the unit refuses to start if
`EnvironmentFile` is missing), a database that isn't up, or a Python import
error after a `git pull` without `pip install`.

### Dashboard loads but the sensor graph never moves

This is the most common failure, and it has three possible layers.

```bash
# Layer 1 — is the bridge running and receiving?
journalctl -u egnsite-mqtt -f
#   "Received: ..."           → MQTT is arriving
#   "Pushed to WebSocket ..."  → the channel layer accepted it
#   nothing at all             → the Pi isn't publishing

# Layer 2 — is anything actually publishing telemetry?
mosquitto_sub -h localhost -p 1885 -t 'raspi/to_django' -v

# Layer 3 — is the WebSocket connected?
#   Browser devtools → Network → WS → /ws/sensor/ should be status 101
```

If layer 3 fails but the page works, nginx isn't upgrading the connection —
check the `location /ws/` block, and confirm `egnsite-web` is running
`EGNSite.asgi` and not `EGNSite.wsgi`.

If layer 2 is silent, see the telemetry-publisher gap in
[open-questions.md](open-questions.md).

### Video tile shows OFFLINE

```bash
# Is the Pi's stream arriving?
journalctl -u mediamtx -f          # look for "is publishing to path 'pendulum'"

# On the camera Pi:
systemctl status pistream
journalctl -u pistream -n 50
```

If MediaMTX shows the stream publishing but the browser still won't play, it's
almost always ICE. Check that `webrtcAdditionalHosts` in
`/usr/local/etc/mediamtx.yml` holds the **current public IP**, and that
**8189/udp** is open in *both* the OCI Security List and iptables:

```bash
sudo iptables -L INPUT -n --line-numbers | grep -E '8189|8890'
```

A WHEP POST that returns 201 while video never starts is the signature of a
bad ICE candidate — not a player bug.

### Commands do nothing / "Another user is currently controlling"

The control lock is held by another session. It self-expires after
`LOCK_TIMEOUT = 60` seconds without a heartbeat. An instructor can seize it via
the E-stop button. To clear it manually:

```bash
cd /opt/ed2/ED2-Repo/WebInterface
set -a; . ../deploy/.env; set +a
/opt/ed2/venv/bin/python manage.py shell -c \
  "from MatlabApp.models import ControlLock; ControlLock.objects.all().delete()"
```

### CSRF verification failed on POST

`DJANGO_CSRF_TRUSTED_ORIGINS` in `deploy/.env` doesn't include the origin
you're browsing from, or nginx isn't sending `X-Forwarded-Proto`. Both are
configured in this repo — check they survived any local edits.

### Reaching the camera Pi through the tunnel

```bash
ssh -p 2222 skylar@localhost      # run this ON the server
```

Nothing listening on 2222 means `autossh-tunnel` on the Pi is down, or still
pointing at `opc@` (see [raspberry-pis.md](raspberry-pis.md)).

## Logs

```bash
journalctl -u egnsite-web -f
journalctl -u egnsite-mqtt -f
journalctl -u mediamtx -f
journalctl -u mosquitto -f
sudo tail -f /var/log/nginx/egnsite.error.log
```

## Certificate renewal

certbot installs a systemd timer that renews automatically. Verify:

```bash
systemctl list-timers | grep certbot
sudo certbot renew --dry-run
```

Renewal needs port 80 reachable — don't close it in the Security List.

## Backups

The database holds user accounts, saved experiment parameters, and command
history. Nothing else on the server is stateful (the repo is in git, configs
are in `deploy/`).

```bash
mysqldump -u egnsite -p egnsitedb | gzip > ~/egnsitedb-$(date +%F).sql.gz
```

Worth a weekly cron entry, and worth taking before any migration.
