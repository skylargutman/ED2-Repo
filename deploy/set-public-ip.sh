#!/usr/bin/env bash
# ===========================================================================
# Sync the instance's CURRENT public IP into every config that needs it.
#
# Oracle no longer offers reserved public IPs on this tenancy, so the address
# is ephemeral. An ephemeral IP survives reboots and stop/starts -- it is only
# released if the instance is TERMINATED or its VNIC detached. So this is not
# a per-boot hazard, but a rebuild WILL hand you a new address.
#
# Rather than hand-editing files after that, this script discovers the current
# public IP and rewrites the one place that genuinely needs a literal address:
# MediaMTX's webrtcAdditionalHosts.
#
# Run automatically as ExecStartPre of mediamtx.service, so a rebuilt or
# re-IP'd instance self-heals on boot. Also safe to run by hand:
#     sudo /opt/ed2/ED2-Repo/deploy/set-public-ip.sh
# ===========================================================================
set -euo pipefail

MTX_CONF="${MTX_CONF:-/usr/local/etc/mediamtx.yml}"
ENV_FILE="${ENV_FILE:-/opt/ed2/ED2-Repo/deploy/.env}"

log() { printf '[set-public-ip] %s\n' "$*"; }

# ---------------------------------------------------------------------------
# Discover the public IP.
#
# OCI's instance metadata service is authoritative and needs no external
# network, so it is tried first. Note that the VNIC's own interface only ever
# carries the PRIVATE address -- `ip addr` is useless here, which is the whole
# reason webrtcAdditionalHosts has to be set explicitly.
# ---------------------------------------------------------------------------
discover_ip() {
    local ip=""

    # awk rather than `grep -oP`: PCRE mode is unavailable in some locales
    # ("grep: -P supports only unibyte and UTF-8 locales") and this has to
    # work under whatever environment systemd hands us.
    # awk exits after the first match instead of piping into `head`, which
    # would SIGPIPE the producer and trip pipefail.
    #
    # v2 requires the Bearer header; v1 is unauthenticated and still present
    # on some images. Try both before reaching out to the internet.
    local url
    for url in "http://169.254.169.254/opc/v2/vnics/" \
               "http://169.254.169.254/opc/v1/vnics/"; do
        ip="$(curl -fsS -m 5 -H 'Authorization: Bearer Oracle' "${url}" 2>/dev/null \
              | tr ',' '\n' \
              | awk '/"publicIp"/ {
                         if (match($0, /[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/)) {
                             print substr($0, RSTART, RLENGTH); exit
                         }
                     }' || true)"
        if [[ -n "${ip}" ]]; then
            log "discovered via OCI metadata ${url##*169.254.169.254}: ${ip}"
            printf '%s' "${ip}"
            return 0
        fi
    done

    # Fallback for non-OCI hosts or if the metadata service is unreachable.
    for svc in https://api.ipify.org https://ifconfig.me/ip; do
        ip="$(curl -fsS -m 5 "${svc}" 2>/dev/null | tr -d '[:space:]' || true)"
        if [[ "${ip}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
            log "discovered via ${svc}: ${ip}"
            printf '%s' "${ip}"
            return 0
        fi
    done

    return 1
}

if ! PUBLIC_IP="$(discover_ip)"; then
    log "ERROR: could not determine public IP; leaving configs untouched"
    exit 1
fi

# ---------------------------------------------------------------------------
# MediaMTX -- the one file that REQUIRES a literal IP.
#
# A browser's ICE agent needs a routable candidate. With the default
# webrtcIPsFromInterfaces, MediaMTX would advertise the private 10.x address
# and video would never start, even though the WHEP POST returns success.
# ---------------------------------------------------------------------------
if [[ -f "${MTX_CONF}" ]]; then
    # `sed ... | head -1` would SIGPIPE sed once head exits; awk self-terminates.
    current="$(awk '/^webrtcAdditionalHosts:/ {
                        if (match($0, /\[[^]]*\]/)) {
                            print substr($0, RSTART + 1, RLENGTH - 2); exit
                        }
                    }' "${MTX_CONF}")"
    if [[ "${current}" == "${PUBLIC_IP}" ]]; then
        log "mediamtx.yml already correct (${PUBLIC_IP})"
    else
        sed -i "s|^webrtcAdditionalHosts:.*|webrtcAdditionalHosts: [${PUBLIC_IP}]|" "${MTX_CONF}"
        log "mediamtx.yml webrtcAdditionalHosts: ${current:-unset} -> ${PUBLIC_IP}"
    fi
else
    log "WARNING: ${MTX_CONF} not found, skipping"
fi

# ---------------------------------------------------------------------------
# Django ALLOWED_HOSTS -- keeps the bare-IP URL working before DNS is live.
# The hostname entries are left alone; only the IP literal is refreshed.
# ---------------------------------------------------------------------------
#
# The line is rebuilt from its non-IP entries rather than regex-substituted.
# An earlier version tried to swap "any dotted quad" in place; because `.*` is
# greedy it matched the LAST address on the line and destroyed the 127.0.0.1
# entry, producing "localhost,12<newip>". Splitting on commas and dropping
# only IPv4 literals is unambiguous.
#
if [[ -f "${ENV_FILE}" ]]; then
    if grep -q '^DJANGO_ALLOWED_HOSTS=' "${ENV_FILE}"; then
        current_val="$(awk 'index($0, "DJANGO_ALLOWED_HOSTS=") == 1 {
                                print substr($0, length("DJANGO_ALLOWED_HOSTS=") + 1); exit
                            }' "${ENV_FILE}")"

        # Keep every hostname; keep loopback; drop other IPv4 literals.
        kept=""
        IFS=',' read -ra entries <<< "${current_val}"
        for e in "${entries[@]}"; do
            e="$(printf '%s' "${e}" | tr -d '[:space:]')"
            [[ -z "${e}" ]] && continue
            if [[ "${e}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ && "${e}" != "127.0.0.1" ]]; then
                continue    # stale public IP -- drop it
            fi
            kept="${kept:+${kept},}${e}"
        done

        new_val="${kept:+${kept},}${PUBLIC_IP}"

        if [[ "${current_val}" == "${new_val}" ]]; then
            log ".env already correct"
        else
            # Write via a temp file so a failure cannot truncate .env.
            tmp="$(mktemp)"
            sed "s|^DJANGO_ALLOWED_HOSTS=.*|DJANGO_ALLOWED_HOSTS=${new_val}|" \
                "${ENV_FILE}" > "${tmp}"
            cat "${tmp}" > "${ENV_FILE}"   # preserve original owner/permissions
            rm -f "${tmp}"
            log ".env DJANGO_ALLOWED_HOSTS -> ${new_val}"
        fi
    fi
else
    log "note: ${ENV_FILE} not found yet, skipping"
fi

log "done (public IP ${PUBLIC_IP})"
