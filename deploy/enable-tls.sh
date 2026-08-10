#!/usr/bin/env bash
# ===========================================================================
# Promote the site from the HTTP-only bootstrap config to the full TLS config.
#
# Run this ONCE, after DNS for sciencelabtoyou.com resolves to this instance.
#     sudo /opt/ed2/ED2-Repo/deploy/enable-tls.sh
#
# Order matters: certbot must obtain the certificate while nginx is still
# serving the HTTP-only config, because the TLS config cannot even load until
# the certificate files exist.
# ===========================================================================
set -euo pipefail

REPO="${REPO:-/opt/ed2/ED2-Repo}"
DOMAIN="${DOMAIN:-sciencelabtoyou.com}"
EMAIL="${EMAIL:-}"          # optional: EMAIL=you@example.com sudo -E ./enable-tls.sh

log() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------------------
log "Checking DNS"
# ---------------------------------------------------------------------------
resolved="$(getent hosts "${DOMAIN}" | awk 'NR == 1 { print $1; exit }' || true)"
mine="$(curl -fsS -m 5 -H 'Authorization: Bearer Oracle' \
        http://169.254.169.254/opc/v2/vnics/ 2>/dev/null \
        | tr ',' '\n' \
        | awk '/"publicIp"/ {
                   if (match($0, /[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/)) {
                       print substr($0, RSTART, RLENGTH); exit
                   }
               }' || true)"

if [[ -z "${resolved}" ]]; then
    echo "ERROR: ${DOMAIN} does not resolve yet. Wait for the DNS change."
    exit 1
fi
echo "  ${DOMAIN} resolves to ${resolved}"
echo "  this instance is    ${mine:-unknown}"

if [[ -n "${mine}" && "${resolved}" != "${mine}" ]]; then
    echo
    echo "ERROR: DNS points somewhere else. certbot's HTTP-01 challenge will"
    echo "       fail because the request would land on another host."
    echo "       Fix the A record (or wait for propagation) and re-run."
    exit 1
fi

# ---------------------------------------------------------------------------
log "Obtaining certificate"
# ---------------------------------------------------------------------------
# --webroot rather than --nginx: it does not rewrite our hand-written config,
# so what ends up deployed is exactly what is in git.
certbot_args=(certonly --webroot -w /var/www/certbot
              -d "${DOMAIN}" -d "www.${DOMAIN}"
              --non-interactive --agree-tos --keep-until-expiring)

if [[ -n "${EMAIL}" ]]; then
    certbot_args+=(--email "${EMAIL}")
else
    certbot_args+=(--register-unsafely-without-email)
fi

certbot "${certbot_args[@]}"

# ---------------------------------------------------------------------------
log "Swapping in the TLS config"
# ---------------------------------------------------------------------------
# The TLS server blocks include this snippet rather than certbot's
# options-ssl-nginx.conf, which only exists if the nginx plugin has run.
install -d -m 0755 /etc/nginx/snippets
install -m 0644 "${REPO}/deploy/nginx/egnsite-ssl.conf" \
                /etc/nginx/snippets/egnsite-ssl.conf

install -m 0644 "${REPO}/deploy/nginx/sciencelabtoyou.conf" \
                /etc/nginx/sites-available/sciencelabtoyou

if nginx -t; then
    systemctl reload nginx
    log "TLS enabled -- https://${DOMAIN}"
else
    log "nginx config test FAILED; rolling back to HTTP-only"
    install -m 0644 "${REPO}/deploy/nginx/sciencelabtoyou-http.conf" \
                    /etc/nginx/sites-available/sciencelabtoyou
    nginx -t && systemctl reload nginx
    echo "Rolled back. The site is still up over HTTP."
    exit 1
fi

# ---------------------------------------------------------------------------
log "Renewal"
# ---------------------------------------------------------------------------
systemctl list-timers | grep -i certbot || true
echo "Test renewal with:  sudo certbot renew --dry-run"
echo "Renewal needs port 80 to stay open in the OCI Security List."
