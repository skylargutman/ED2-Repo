# Oracle Cloud Server

## Instance

| | |
|---|---|
| Public IP | **129.153.42.213** (reserved — see below) |
| Hostname | `sciencelabtoyou.com` |
| Shape | VM.Standard.A1.Flex — 1 OCPU, 6 GB (Ampere, aarch64) |
| Image | Ubuntu 24.04 LTS (aarch64) |
| Login | `ssh ubuntu@sciencelabtoyou.com` |
| Deploy root | `/opt/ed2/ED2-Repo`, venv at `/opt/ed2/venv` |

> The previous instance ran Oracle Linux with `opc` as the login user. Ubuntu
> uses **`ubuntu`**. Anything referencing `opc@` — notably the camera Pi's
> `autossh-tunnel.service` — must be updated.

### Why Ubuntu 24.04 over Oracle Linux 9

OL9 enforces SELinux, which blocks nginx from connecting to a gunicorn socket
until `setsebool -P httpd_can_network_connect 1`. It surfaces as a bare 502
that reads like an application fault. For a stack that is mostly reverse
proxies, that's an unnecessary debugging tax. Ubuntu also packages mosquitto,
redis, and certbot more directly, and ships Python 3.12 — supported by
Django 4.2 LTS (since 4.2.8).

### The public IP is ephemeral

Oracle no longer permits reserved public IPs on this tenancy, so the address
stays **ephemeral**. What that actually means:

- ✅ It **survives reboots and stop/starts**. An ephemeral IP is held for the
  lifetime of the instance.
- ❌ It is **released if the instance is terminated** or its VNIC detached. A
  rebuild gets a new address.

So this is not a day-to-day hazard, but a rebuild means the IP changes.

**`deploy/set-public-ip.sh` handles that automatically.** It discovers the
current address from OCI's instance metadata service and rewrites the one
config that genuinely needs a literal IP — MediaMTX's
`webrtcAdditionalHosts` — plus the IP entry in `DJANGO_ALLOWED_HOSTS`. It runs
as `ExecStartPre` of `mediamtx.service`, so a rebuilt instance self-heals on
boot. You can also run it by hand:

```bash
sudo /opt/ed2/ED2-Repo/deploy/set-public-ip.sh
```

Note it reads the IP from the metadata service, **not** from the network
interface — an OCI VNIC only ever carries the private 10.x address.

Everything else in the system refers to the **hostname**, not the IP: the
Pis' `stream.sh`, `autossh-tunnel.service`, and `dac_daemon.py` all target
`sciencelabtoyou.com`. So after a rebuild the only manual step is updating the
DNS A record — the Pis need no changes.

## Two firewalls

This trips up nearly everyone on OCI. A port must be open in **both**:

1. **OCI Security List** (VCN → subnet → Security List → Ingress Rules)
2. **Instance iptables** — Oracle's Ubuntu images ship rules that REJECT
   everything except port 22

`deploy/bootstrap.sh` handles #2. #1 is console-only:

| Port | Proto | Source | For |
|---|---|---|---|
| 22 | TCP | 0.0.0.0/0 | SSH + camera Pi reverse tunnel |
| 80 | TCP | 0.0.0.0/0 | certbot HTTP-01, redirect |
| 443 | TCP | 0.0.0.0/0 | the site |
| 1885 | TCP | 0.0.0.0/0 | MQTT from control Pi |
| 8890 | **UDP** | 0.0.0.0/0 | SRT video ingest |
| 8189 | **UDP** | 0.0.0.0/0 | WebRTC media |

The console defaults new rules to TCP. Creating 8890 and 8189 as TCP is a
common mistake and produces "video connects but never plays".

## Services

| Unit | What it does | Fails as |
|---|---|---|
| `nginx` | TLS, static, reverse proxy | Site unreachable |
| `egnsite-web` | gunicorn + uvicorn worker → `EGNSite.asgi` | 502 Bad Gateway |
| `egnsite-mqtt` | `mqtt_subscriber.py`, MQTT → WebSocket bridge | Dashboard loads, **graph frozen** |
| `mediamtx` | SRT ingest → WebRTC egress | Video tile shows OFFLINE |
| `mosquitto` | MQTT broker on 1885 | Commands do nothing |
| `redis-server` | Channels layer | WebSockets fail to connect |
| `mariadb` | `egnsitedb` | 500 on every page |

`egnsite-web` **must** serve ASGI, not WSGI. Pointing it at `EGNSite.wsgi`
yields a fully working dashboard with a permanently dead live graph, because
Channels never gets wired up.

## Build from scratch

```bash
ssh ubuntu@129.153.42.213
curl -O https://raw.githubusercontent.com/skylargutman/ED2-Repo/main/deploy/bootstrap.sh
chmod +x bootstrap.sh
./bootstrap.sh
```

The script installs packages, opens iptables, clones the repo (shallow +
sparse — the full history carries ~150 MB of KiCad backups and a 63 MB vendor
ISO the server has no use for), builds the venv, installs MediaMTX for arm64,
and drops all service configs into place.

It then prints the manual steps, which are interactive by nature:

1. **DNS** — point `sciencelabtoyou.com` A → the instance IP, confirm with
   `dig +short sciencelabtoyou.com`
2. **Database** — create `egnsitedb` and the `egnsite` user
3. **Secrets** — `cp deploy/env.example deploy/.env`, generate a fresh
   `DJANGO_SECRET_KEY`, set the DB password
4. **Migrate** — `manage.py migrate`, `collectstatic`, `createsuperuser`
5. **Start** — `sudo systemctl enable --now egnsite-web egnsite-mqtt mediamtx`
6. **TLS, later** — `sudo deploy/enable-tls.sh` once DNS resolves here

### HTTP first, TLS second

`bootstrap.sh` installs `nginx/sciencelabtoyou-http.conf`, **not** the TLS
config, because of a bootstrapping cycle: the TLS config references
`/etc/letsencrypt/live/…/fullchain.pem`, and if that file is missing `nginx -t`
fails and nginx won't start — leaving it unable to serve the ACME challenge
that would create the certificate.

The HTTP config is fully functional. The site, the sensor WebSocket, and the
video proxy all work over plain HTTP against the bare IP, so the whole stack
can be validated **before DNS is ready**. Its `server_name _` catch-all means
it answers on any address, which matters given the ephemeral IP.

Once DNS resolves to the instance:

```bash
sudo /opt/ed2/ED2-Repo/deploy/enable-tls.sh
```

That checks DNS actually points here (otherwise the HTTP-01 challenge would be
answered by whatever host it *does* point at), obtains the certificate with
`certbot certonly --webroot`, swaps in the TLS config, and rolls back to HTTP
if `nginx -t` fails. `--webroot` rather than `--nginx` so certbot doesn't
rewrite the hand-written config — what's deployed stays exactly what's in git.

### Accounts to recreate

The database starts empty. Beyond the superuser, the app expects:

- Instructor accounts — set `is_instructor=True` to enable the E-stop
- Student accounts — default
- **`showcase`** — must exist with `is_viewer=True`, or the `/demo/` URL
  redirects to login. It auto-logs in and lands on the `CartControl`
  experiment in read-only mode.

## Configuration that encodes the IP

| File | Setting | Updated by |
|---|---|---|
| `/usr/local/etc/mediamtx.yml` | `webrtcAdditionalHosts` | `set-public-ip.sh` (automatic) |
| `deploy/.env` | `DJANGO_ALLOWED_HOSTS` | `set-public-ip.sh` (automatic) |
| `nginx/sciencelabtoyou.conf` | `server_name` | not needed — hostname-based |
| DNS | the A record | **manual** |

After a rebuild, the DNS A record is the only manual step.

### The WebRTC/OCI gotcha

An OCI instance's NIC only carries its **private** 10.x address — the public IP
is NAT'd by the fabric and never appears on any interface. MediaMTX's default
`webrtcIPsFromInterfaces: true` therefore advertises `10.0.x.x` as its ICE
candidate, which no browser can route to. The WHEP POST still succeeds, so it
presents as a player bug rather than a network one.

`deploy/mediamtx/mediamtx.yml` fixes this with:

```yaml
webrtcAdditionalHosts: [129.153.42.213]
webrtcIPsFromInterfaces: false
```

The stock config in `picamera/mediamtx.yml` does **not** have this set, so
however video worked on the old instance, it wasn't via that file as committed.

## Hardening changes from the old instance

- MediaMTX's RTSP (8554), RTMP (1935) and HLS (8888) listeners are **off** —
  the old stock config had them all public and unused.
- MediaMTX WHEP (8889) is loopback-only, reached through nginx.
- Redis and MariaDB are loopback-only.
- `DEBUG=False`; secrets moved to `deploy/.env`.
- `SECURE_PROXY_SSL_HEADER` set so Django knows requests arrived over TLS.

Still open: mosquitto on 1885 is anonymous and world-reachable, meaning anyone
who can reach that port can publish to `pendulum/cmd` and drive real hardware.
See [open-questions.md](open-questions.md).
