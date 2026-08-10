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
# Delegated to open-firewall.sh, which locates the catch-all REJECT rather
# than assuming its position, and then VERIFIES each ACCEPT actually precedes
# it. An earlier version here hardcoded `-I INPUT 6`; when the chain differed,
# rules landed after the REJECT, where they are listed, look correct, and do
# nothing.
sudo "${REPO}/deploy/open-firewall.sh"

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

# The scripts are committed mode 755, but a checkout made before that fix (or
# any clone where core.filemode is off) can land them non-executable, which
# shows up as the confusing "sudo: deploy/setup-db.sh: command not found".
# Assert it here rather than trusting the checkout -- and do it BEFORE any of
# them is invoked below.
chmod +x "${REPO}"/deploy/*.sh

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
    # awk, not `grep -oP`: PCRE mode is unavailable under some locales
    # ("grep: -P supports only unibyte and UTF-8 locales").
    MTX_VER="$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
               | tr ',' '\n' \
               | awk -F'"' '/"tag_name"/ { print $4; exit }')"
    [[ -n "${MTX_VER}" ]] || { echo "  could not determine MediaMTX version"; exit 1; }
    echo "  installing MediaMTX ${MTX_VER}"
    curl -fsSL -o /tmp/mediamtx.tar.gz \
        "https://github.com/bluenviron/mediamtx/releases/download/${MTX_VER}/mediamtx_${MTX_VER}_linux_arm64.tar.gz"
    tar -xzf /tmp/mediamtx.tar.gz -C /tmp mediamtx
    sudo install -m 0755 /tmp/mediamtx /usr/local/bin/mediamtx
    rm -f /tmp/mediamtx.tar.gz /tmp/mediamtx
fi
id -u mediamtx >/dev/null 2>&1 || sudo useradd --system --no-create-home --shell /usr/sbin/nologin mediamtx
sudo install -m 0644 "${REPO}/deploy/mediamtx/mediamtx.yml" /usr/local/etc/mediamtx.yml

# The public IP is ephemeral, so discover the real one rather than trusting
# the literal committed in mediamtx.yml.
sudo "${REPO}/deploy/set-public-ip.sh" || echo "  (IP sync failed; check manually)"

# ---------------------------------------------------------------------------
log "Installing service configs"
# ---------------------------------------------------------------------------
sudo install -m 0644 "${REPO}/deploy/mosquitto/pendulum.conf" /etc/mosquitto/conf.d/pendulum.conf
sudo install -m 0644 "${REPO}/deploy/systemd/egnsite-web.service"  /etc/systemd/system/
sudo install -m 0644 "${REPO}/deploy/systemd/egnsite-mqtt.service" /etc/systemd/system/
sudo install -m 0644 "${REPO}/deploy/systemd/mediamtx.service"     /etc/systemd/system/
# HTTP-only config to start with. The TLS config references certificate files
# that do not exist yet, and nginx refuses to start if they are missing --
# which would leave us unable to serve the ACME challenge that creates them.
# deploy/enable-tls.sh swaps in the TLS config once DNS is live.
sudo install -m 0644 "${REPO}/deploy/nginx/sciencelabtoyou-http.conf" /etc/nginx/sites-available/sciencelabtoyou
sudo ln -sf /etc/nginx/sites-available/sciencelabtoyou /etc/nginx/sites-enabled/sciencelabtoyou
sudo rm -f /etc/nginx/sites-enabled/default
sudo mkdir -p /var/www/certbot
chmod +x "${REPO}"/deploy/*.sh

# Redis and MariaDB stay bound to loopback; nothing external should reach them.
sudo systemctl enable --now redis-server mariadb mosquitto
sudo systemctl daemon-reload

# ---------------------------------------------------------------------------
log "Bootstrap complete -- remaining MANUAL steps"
# ---------------------------------------------------------------------------
cat <<EOF

  Public IP (ephemeral, discovered at run time): $(curl -fsS -m 5 \
      -H 'Authorization: Bearer Oracle' http://169.254.169.254/opc/v2/vnics/ 2>/dev/null \
      | tr ',' '\n' \
      | awk '/"publicIp"/ { if (match($0, /[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/)) \
                            { print substr($0, RSTART, RLENGTH); exit } }' \
      || echo "${PUBLIC_IP}")

  nginx is serving HTTP only for now. Steps 1 and 5 need DNS; everything
  else can be done immediately and tested against the bare IP.

  1. Point DNS:  sciencelabtoyou.com  A  <the IP above>
     Wait for it to resolve:  dig +short sciencelabtoyou.com

  2. Create the secrets file:
       cp ${REPO}/deploy/env.example ${REPO}/deploy/.env
       chmod 600 ${REPO}/deploy/.env
       \$EDITOR ${REPO}/deploy/.env     # set a new DJANGO_SECRET_KEY

     Generate a secret key with:
       /opt/ed2/venv/bin/python -c \\
         "from django.core.management.utils import get_random_secret_key as k; print(k())"

  3. Create the database and user:
       sudo ${REPO}/deploy/setup-db.sh

     Generates a DB password, writes it into .env, and verifies the login.
     NOTE the sudo: MariaDB's root account uses unix_socket auth, so a bare
     'mariadb' fails with "Access denied for user 'ubuntu'@'localhost'".

  4. Migrate and collect static:
       cd ${REPO}/WebInterface
       ${VENV}/bin/python manage.py migrate
       ${VENV}/bin/python manage.py collectstatic --noinput
       ${VENV}/bin/python manage.py createsuperuser

  5. Start everything:
       sudo systemctl enable --now egnsite-web egnsite-mqtt mediamtx
       sudo systemctl reload nginx

     The site should now answer over HTTP on the bare IP. Verify with the
     "Smoke test" section of ${REPO}/docs/operations.md.

  6. LATER, once DNS from step 1 resolves -- enable HTTPS:
       sudo ${REPO}/deploy/enable-tls.sh

     This obtains the certificate and swaps in the TLS nginx config. It
     refuses to run (and rolls back) if DNS does not point here yet.

EOF
