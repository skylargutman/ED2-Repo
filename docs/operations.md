# Operations Runbook

## Deploying an update

The server runs a checkout of this repo at `/opt/ed2/ED2-Repo`:

> **Run `manage.py` as `ubuntu`, never with `sudo`, and do not source
> `deploy/.env`.** `settings.py` loads that file itself. See
> "Access denied ... after setup-db.sh succeeded" below for why sourcing it
> actively breaks things.

```bash
ssh ubuntu@sciencelabtoyou.com
cd /opt/ed2/ED2-Repo
git pull

# Only if requirements.txt changed
/opt/ed2/venv/bin/pip install -r WebInterface/requirements.txt

# Only if models/migrations changed
cd WebInterface
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
/opt/ed2/venv/bin/python manage.py shell -c \
  "from MatlabApp.models import ControlLock; ControlLock.objects.all().delete()"
```

### `ERROR 1698 (28000): Access denied for user 'ubuntu'@'localhost'`

You ran `mariadb` without `sudo`. MariaDB's root account on Ubuntu uses the
**`unix_socket`** auth plugin — it authenticates by OS user, not password — so
a bare `mariadb` tries to log in as your shell user, which is not a DB user.

```bash
sudo mariadb          # correct
mariadb               # ERROR 1698
```

To connect as the *application* user, name the user and database explicitly:

```bash
mariadb -u egnsite -p egnsitedb
```

(`mariadb egnsite@localhost` doesn't do what it looks like — the argument is
read as a database name.)

### `(1045, "Access denied for user 'egnsite'@'localhost'")` from Django

The DB user doesn't exist, or its password doesn't match `deploy/.env`. Fix
both at once — this is idempotent and re-syncs the password:

```bash
sudo /opt/ed2/ED2-Repo/deploy/setup-db.sh
```

To diagnose by hand, first prove the DB side works independently of Django:

```bash
mariadb -u egnsite -p egnsitedb -e "SELECT 1;"
grep '^DB_' /opt/ed2/ED2-Repo/deploy/.env
```

Two things that produce this error even when the password *looks* right:

- **`#` or `$` in the password.** `#` starts a comment in a systemd
  `EnvironmentFile`, and `$` is expanded when you `. .env` in a shell — so
  Django receives a truncated or altered value. `setup-db.sh` generates
  alphanumeric-only passwords to avoid this.
- **`localhost` vs `127.0.0.1`.** MariaDB treats them as different hosts.
  `.env` sets `DB_HOST=127.0.0.1` (TCP), which only matches a `@'localhost'`
  grant because the server reverse-resolves the address. `setup-db.sh` grants
  to both hosts so this can't bite.

### Browser can't reach the site, but curl works on the server

Read the exact browser error — the two mean different things:

| Error | Meaning |
|---|---|
| `ERR_CONNECTION_REFUSED` | Packets reach the host; nothing is listening on that port |
| `ERR_CONNECTION_TIMED_OUT` | Packets are being dropped — firewall (see the iptables entry below) |

**Before TLS is enabled, only port 80 is listening.** `https://` therefore gets
refused, and Chrome's HTTPS-First mode silently upgrades a typed URL to
`https://`, so it looks like the server is down when it is fine. Type the
scheme explicitly:

```
http://129.153.42.213/login/
```

An incognito window helps if Chrome has cached the upgrade for that host.

Confirm the server side independently before suspecting the network:

```bash
systemctl is-active nginx
sudo nginx -t
ss -lntp | grep -E ':(80|443)\s'      # expect 0.0.0.0:80 (and :443 only after TLS)
curl -sI http://localhost/login/      # expect 200
```

If all four are healthy, the problem is between the browser and the host, not
on the host.

### collectstatic: "Found another file with the destination path ..."

A long list of these for `admin/…` paths means Django's admin assets have been
vendored into `WebInterface/staticfiles/`, duplicating what the installed
Django package already provides.

This is not cosmetic. `STATICFILES_DIRS` is searched by `FileSystemFinder`
**before** `AppDirectoriesFinder`, so the vendored copy **wins** — you end up
serving admin CSS/JS from whatever Django version generated it, against
whatever Django is actually running. The tell is a summary like
`0 static files copied, 127 unmodified` where 127 is suspiciously close to the
number of vendored files.

Fixed in the repo by deleting `WebInterface/staticfiles/admin/` (and
gitignoring it). If a stale `collected_static` is still on the server, clear it
so the correct assets are picked up:

```bash
cd /opt/ed2/ED2-Repo && git pull
rm -rf WebInterface/collected_static
cd WebInterface && /opt/ed2/venv/bin/python manage.py collectstatic --noinput
```

You should then see several hundred files copied and no duplicate warnings.

### `sudo: deploy/<script>.sh: command not found`

The file isn't executable. The scripts are committed mode 755, but a checkout
taken before that was fixed — or any clone where `core.filemode` is off — can
land them `644`. `sudo` then can't exec them and reports "command not found",
which reads like a missing file.

```bash
chmod +x /opt/ed2/ED2-Repo/deploy/*.sh
```

Or bypass it for one run by invoking the interpreter explicitly:

```bash
sudo bash deploy/setup-db.sh --new-password
```

`bootstrap.sh` now asserts the bit after every clone/pull, so this is
self-correcting once you have run it again.

### Access denied (1045) after `setup-db.sh` succeeded — and `sudo` "fixes" it

The tell-tale symptom: `manage.py migrate` fails as `ubuntu` but works under
`sudo`. That is **not** a permissions problem, and `sudo` is not the fix.

`settings.py` calls `load_dotenv(deploy/.env)`, and python-dotenv defaults to
**`override=False`** — it will not replace variables already present in the
environment. So if your shell exported `DB_PASSWORD` earlier (from a
`set -a; . .env; set +a`, back when it still said `replace-me`), that stale
value **shadows the file forever**. `setup-db.sh` writing a fresh password to
`.env` changes nothing, because the environment wins.

`sudo` resets the environment (`env_reset`), so nothing shadows the file and
it connects — which makes `sudo` look like the cure while hiding the cause.

Diagnose:

```bash
env | grep -E '^(DB_|DJANGO_|REDIS_|MQTT_)'      # anything here is shadowing .env
```

Fix — start a clean shell:

```bash
exec bash -l
cd /opt/ed2/ED2-Repo/WebInterface
/opt/ed2/venv/bin/python manage.py migrate       # no sudo
```

**Never source `deploy/.env` into your shell.** `settings.py` reads it, and
systemd reads it via `EnvironmentFile=`. Sourcing it by hand is redundant and
sets this exact trap.

If you did run `manage.py` under `sudo`, fix any root-owned files it left
behind, or the services (which run as `ubuntu`) will fail later:

```bash
sudo chown -R ubuntu:www-data /opt/ed2/ED2-Repo
find /opt/ed2/ED2-Repo -user root
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
