#!/usr/bin/env bash
# ===========================================================================
# EGNSite server bootstrap -- Ubuntu 24.04 on Oracle Cloud Ampere A1 (aarch64)
#
# Run as the `ubuntu` user on a fresh instance:
#     curl -O https://raw.githubusercontent.com/skylargutman/ED2-Repo/main/deploy/bootstrap.sh
#     chmod +x bootstrap.sh
#     ./bootstrap.sh
#
# Safe to re-run. It does NOT obtain the TLS cert or create the database
# password -- those are interactive and called out at the end.
# ===========================================================================
set -euo pipefail

REPO_URL="https://github.com/skylargutman/ED2-Repo.git"
BASE="/opt/ed2"
REPO="${BASE}/ED2-Repo"
VENV="${BASE}/venv"
PUBLIC_IP="129.153.42.213"

log() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------------------
log "Installing packages"
# ---------------------------------------------------------------------------
sudo apt-get update
sudo apt-get install -y \
    git curl ca-certificates \
    python3 python3-venv python3-dev build-essential pkg-config \
    nginx \
    mariadb-server libmariadb-dev \
    redis-server \
    mosquitto mosquitto-clients \
    certbot python3-certbot-nginx \
    ffmpeg \
    iptables-persistent

# ---------------------------------------------------------------------------
log "Opening the instance firewall"
# ---------------------------------------------------------------------------
# Oracle's Ubuntu images ship iptables rules that REJECT everything except 22.
# The cloud-side Security List is a separate, second firewall -- both must
# allow a port or traffic dies silently. This is the #1 cause of "my OCI
# instance won't serve traffic".
#
# Rules are inserted before the catch-all REJECT in the INPUT chain.
open_port() {
    local port="$1" proto="$2"
    if ! sudo iptables -C INPUT -p "$proto" --dport "$port" -j ACCEPT 2>/dev/null; then
        sudo iptables -I INPUT 6 -p "$proto" --dport "$port" -j ACCEPT
        echo "  opened ${port}/${proto}"
    else
        echo "  ${port}/${proto} already open"
    fi
}
open_port 80   tcp   # HTTP  -- certbot challenge + redirect
open_port 443  tcp   # HTTPS -- the site
open_port 1885 tcp   # MQTT  -- control Pi
open_port 8890 udp   # SRT   -- camera Pi video ingest
open_port 8189 udp   # WebRTC media
sudo netfilter-persistent save

# ---------------------------------------------------------------------------
log "Cloning the repo"
# ---------------------------------------------------------------------------
sudo mkdir -p "${BASE}"
sudo chown ubuntu:ubuntu "${BASE}"

if [[ -d "${REPO}/.git" ]]; then
    git -C "${REPO}" pull --ff-only
else
    # Shallow + sparse: the full repo carries ~150 MB of KiCad backups, PDFs
    # and a 63 MB vendor ISO that the server has no use for.
    git clone --depth 1 --filter=blob:none --sparse "${REPO_URL}" "${REPO}"
    git -C "${REPO}" sparse-checkout set WebInterface picamera picontrol deploy docs
fi

# ---------------------------------------------------------------------------
log "Building the virtualenv"
# ---------------------------------------------------------------------------
# Lives OUTSIDE the repo so `git pull` can never collide with it.
[[ -d "${VENV}" ]] || python3 -m venv "${VENV}"
"${VENV}/bin/pip" install --upgrade pip wheel
"${VENV}/bin/pip" install -r "${REPO}/WebInterface/requirements.txt"

# ---------------------------------------------------------------------------
log "Installing MediaMTX (arm64)"
# ---------------------------------------------------------------------------
if ! command -v mediamtx >/dev/null 2>&1; then
    MTX_VER="$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
               | grep -oP '"tag_name":\s*"\K[^"]+')"
    echo "  installing MediaMTX ${MTX_VER}"
    curl -fsSL -o /tmp/mediamtx.tar.gz \
        "https://github.com/bluenviron/mediamtx/releases/download/${MTX_VER}/mediamtx_${MTX_VER}_linux_arm64.tar.gz"
    tar -xzf /tmp/mediamtx.tar.gz -C /tmp mediamtx
    sudo install -m 0755 /tmp/mediamtx /usr/local/bin/mediamtx
    rm -f /tmp/mediamtx.tar.gz /tmp/mediamtx
fi
id -u mediamtx >/dev/null 2>&1 || sudo useradd --system --no-create-home --shell /usr/sbin/nologin mediamtx
sudo install -m 0644 "${REPO}/deploy/mediamtx/mediamtx.yml" /usr/local/etc/mediamtx.yml

# ---------------------------------------------------------------------------
log "Installing service configs"
# ---------------------------------------------------------------------------
sudo install -m 0644 "${REPO}/deploy/mosquitto/pendulum.conf" /etc/mosquitto/conf.d/pendulum.conf
sudo install -m 0644 "${REPO}/deploy/systemd/egnsite-web.service"  /etc/systemd/system/
sudo install -m 0644 "${REPO}/deploy/systemd/egnsite-mqtt.service" /etc/systemd/system/
sudo install -m 0644 "${REPO}/deploy/systemd/mediamtx.service"     /etc/systemd/system/
sudo install -m 0644 "${REPO}/deploy/nginx/sciencelabtoyou.conf" /etc/nginx/sites-available/sciencelabtoyou
sudo ln -sf /etc/nginx/sites-available/sciencelabtoyou /etc/nginx/sites-enabled/sciencelabtoyou
sudo rm -f /etc/nginx/sites-enabled/default
sudo mkdir -p /var/www/certbot

# Redis and MariaDB stay bound to loopback; nothing external should reach them.
sudo systemctl enable --now redis-server mariadb mosquitto
sudo systemctl daemon-reload

# ---------------------------------------------------------------------------
log "Bootstrap complete -- remaining MANUAL steps"
# ---------------------------------------------------------------------------
cat <<EOF

  Public IP baked into configs: ${PUBLIC_IP}

  1. Point DNS:  sciencelabtoyou.com  A  ${PUBLIC_IP}
     Wait for it to resolve:  dig +short sciencelabtoyou.com

  2. Create the database and user:
       sudo mariadb
         CREATE DATABASE egnsitedb CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
         CREATE USER 'egnsite'@'localhost' IDENTIFIED BY '<pick-a-strong-password>';
         GRANT ALL PRIVILEGES ON egnsitedb.* TO 'egnsite'@'localhost';
         FLUSH PRIVILEGES;

  3. Create the secrets file:
       cp ${REPO}/deploy/env.example ${REPO}/deploy/.env
       chmod 600 ${REPO}/deploy/.env
       \$EDITOR ${REPO}/deploy/.env     # new SECRET_KEY + the DB password above

  4. Migrate and collect static:
       cd ${REPO}/WebInterface
       set -a; . ${REPO}/deploy/.env; set +a
       ${VENV}/bin/python manage.py migrate
       ${VENV}/bin/python manage.py collectstatic --noinput
       ${VENV}/bin/python manage.py createsuperuser

  5. Get the TLS cert (needs DNS from step 1 to have propagated):
       sudo certbot --nginx -d sciencelabtoyou.com -d www.sciencelabtoyou.com

  6. Start everything:
       sudo systemctl enable --now egnsite-web egnsite-mqtt mediamtx
       sudo systemctl reload nginx

  7. Verify:  ${REPO}/docs/operations.md  ("Smoke test" section)

EOF
