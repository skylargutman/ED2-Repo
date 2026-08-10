#!/usr/bin/env bash
# ===========================================================================
# Open this system's own iptables for the ports the stack needs.
#
#     sudo /opt/ed2/ED2-Repo/deploy/open-firewall.sh
#
# There are TWO firewalls in front of an OCI instance:
#
#   1. The OCI Security List / NSG   (cloud side, configured in the console)
#   2. The instance's iptables       (this script)
#
# A port must be open in both. Oracle's Ubuntu images ship an INPUT chain that
# ends in `REJECT all -- reject-with icmp-host-prohibited`, so everything
# except SSH is blocked until rules are added ahead of it.
#
# From outside, that REJECT usually looks like a TIMEOUT rather than a
# connection refused, because the ICMP unreachable it generates is commonly
# swallowed before it leaves the cloud. So "cloud rules look right but the
# site times out" is the classic signature of this script not having run.
#
# Idempotent: existing rules are detected and left alone.
# ===========================================================================
set -euo pipefail

[[ ${EUID} -eq 0 ]] || { echo "must run as root (use sudo)" >&2; exit 1; }

log() { printf '[open-firewall] %s\n' "$*"; }

# port:proto:description
PORTS=(
    "80:tcp:HTTP (site + certbot ACME challenge)"
    "443:tcp:HTTPS"
    "1885:tcp:MQTT — control Pi"
    "8890:udp:SRT video ingest — camera Pi"
    "8189:udp:WebRTC media — MediaMTX"
)
# 22/tcp is deliberately absent: Oracle's base rules already allow it, and a
# mistake here could lock you out.

# ---------------------------------------------------------------------------
# Find where the catch-all REJECT/DROP sits, so ACCEPT rules go BEFORE it.
#
# The previous version of this logic hardcoded `-I INPUT 6`, which silently
# placed rules AFTER the REJECT on any image whose chain differed -- the rule
# is then listed, looks correct, and does nothing.
# ---------------------------------------------------------------------------
reject_pos() {
    iptables -L INPUT --line-numbers -n \
        | awk '$2 == "REJECT" || $2 == "DROP" { print $1; exit }'
}

insert_at="$(reject_pos || true)"
if [[ -n "${insert_at}" ]]; then
    log "catch-all REJECT/DROP is rule #${insert_at}; rules will go before it"
else
    log "no catch-all REJECT/DROP found; appending to end of chain"
fi

# Delete-then-insert, rather than "skip if present".
#
# A rule that already exists may be in the WRONG PLACE -- `iptables -C` only
# asks whether the rule exists, not where. Skipping on that basis is how a
# misordered chain stays broken through repeated runs. Removing every copy
# first and re-inserting makes placement correct regardless of prior state,
# and is naturally idempotent.
for entry in "${PORTS[@]}"; do
    port="${entry%%:*}"
    rest="${entry#*:}"
    proto="${rest%%:*}"
    desc="${rest#*:}"

    removed=0
    while iptables -C INPUT -p "${proto}" --dport "${port}" -j ACCEPT 2>/dev/null; do
        iptables -D INPUT -p "${proto}" --dport "${port}" -j ACCEPT
        removed=$((removed + 1))
    done

    # Re-read each time: every insert shifts the REJECT down by one.
    pos="$(reject_pos || true)"
    if [[ -n "${pos}" ]]; then
        iptables -I INPUT "${pos}" -p "${proto}" --dport "${port}" -j ACCEPT
    else
        iptables -A INPUT -p "${proto}" --dport "${port}" -j ACCEPT
    fi

    if [[ ${removed} -gt 0 ]]; then
        log "  ${port}/${proto} repositioned (removed ${removed} misplaced) — ${desc}"
    else
        log "  ${port}/${proto} opened — ${desc}"
    fi
done

# ---------------------------------------------------------------------------
# Verify the rules actually precede the REJECT, rather than trusting the
# insert. This is the check whose absence caused the original bug.
# ---------------------------------------------------------------------------
echo
log "Verifying rule order"
final_reject="$(reject_pos || true)"
problems=0
for entry in "${PORTS[@]}"; do
    port="${entry%%:*}"; rest="${entry#*:}"; proto="${rest%%:*}"

    # Match on the trailing "tcp dpt:80" text, not the prot column: with -n,
    # iptables may print the protocol NUMERICALLY (6, 17) instead of tcp/udp,
    # so testing $3 == "tcp" silently never matches. The trailing extras field
    # always spells the protocol out.
    #
    # The ([^0-9]|$) guard stops "dpt:80" from also matching "dpt:8080".
    line="$(iptables -L INPUT --line-numbers -n \
            | awk -v pat="${proto} dpt:${port}" \
                  '$2 == "ACCEPT" && $0 ~ (pat "([^0-9]|$)") { print $1; exit }')"

    if [[ -z "${line}" ]]; then
        echo "  FAIL ${port}/${proto} — no ACCEPT rule found"
        problems=$((problems + 1))
    elif [[ -n "${final_reject}" && "${line}" -gt "${final_reject}" ]]; then
        echo "  FAIL ${port}/${proto} — rule #${line} is AFTER the REJECT at #${final_reject}"
        problems=$((problems + 1))
    else
        echo "   OK  ${port}/${proto} — rule #${line}"
    fi
done

# ---------------------------------------------------------------------------
if [[ ${changed} -gt 0 ]]; then
    echo
    log "Persisting rules across reboot"
    if command -v netfilter-persistent >/dev/null 2>&1; then
        netfilter-persistent save
    else
        log "WARNING: netfilter-persistent not installed; rules will be LOST on reboot"
        log "         sudo apt-get install -y iptables-persistent"
    fi
fi

echo
if [[ ${problems} -eq 0 ]]; then
    log "All ports open. Remember the OCI Security List / NSG must allow them too."
else
    log "${problems} problem(s) — inspect with:  sudo iptables -L INPUT -n --line-numbers"
    exit 1
fi
