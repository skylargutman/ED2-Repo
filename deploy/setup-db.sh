#!/usr/bin/env bash
# ===========================================================================
# Create the MariaDB database and user, and make deploy/.env agree with it.
#
#     sudo /opt/ed2/ED2-Repo/deploy/setup-db.sh
#
# MUST be run with sudo. On Ubuntu, MariaDB's root account uses the
# unix_socket auth plugin -- it authenticates by OS user, not password. A bare
# `mariadb` therefore tries to log in as your shell user and fails with
#     ERROR 1698 (28000): Access denied for user 'ubuntu'@'localhost'
# which looks like a password problem but is not.
#
# Idempotent: safe to re-run. It resets the password to whatever .env holds,
# so a half-finished earlier attempt cannot leave the two out of sync.
# ===========================================================================
set -euo pipefail

REPO="${REPO:-/opt/ed2/ED2-Repo}"
ENV_FILE="${ENV_FILE:-${REPO}/deploy/.env}"

log() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }
die() { printf '\n\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

[[ ${EUID} -eq 0 ]] || die "must run as root (use sudo) -- MariaDB root uses unix_socket auth"
[[ -f "${ENV_FILE}" ]] || die "${ENV_FILE} not found. Copy it first:
    cp ${REPO}/deploy/env.example ${ENV_FILE}
    chmod 600 ${ENV_FILE}"

# ---------------------------------------------------------------------------
# Read current values without sourcing the file (avoids shell expansion of
# any $ or backticks that happen to be inside a password).
# ---------------------------------------------------------------------------
getenv() {
    sed -n "s/^$1=//p" "${ENV_FILE}" | head -1 | sed 's/^["'\'']//; s/["'\'']$//'
}

DB_NAME="$(getenv DB_NAME)";     DB_NAME="${DB_NAME:-egnsitedb}"
DB_USER="$(getenv DB_USER)";     DB_USER="${DB_USER:-egnsite}"
DB_PASSWORD="$(getenv DB_PASSWORD)"

# ---------------------------------------------------------------------------
# Generate a password if the placeholder is still in place.
#
# Restricted alphabet on purpose: '#' begins a comment in a systemd
# EnvironmentFile, and '$' is expanded when the file is sourced in a shell --
# either would silently corrupt the value between here and Django.
# ---------------------------------------------------------------------------
if [[ -z "${DB_PASSWORD}" || "${DB_PASSWORD}" == "replace-me" ]]; then
    DB_PASSWORD="$(tr -dc 'A-Za-z0-9' </dev/urandom | head -c 32)"
    log "Generated a database password and writing it to .env"

    tmp="$(mktemp)"
    if grep -q '^DB_PASSWORD=' "${ENV_FILE}"; then
        sed "s|^DB_PASSWORD=.*|DB_PASSWORD=${DB_PASSWORD}|" "${ENV_FILE}" > "${tmp}"
    else
        cat "${ENV_FILE}" > "${tmp}"
        printf 'DB_PASSWORD=%s\n' "${DB_PASSWORD}" >> "${tmp}"
    fi
    cat "${tmp}" > "${ENV_FILE}"     # rewrite in place, preserving owner/mode
    rm -f "${tmp}"
else
    log "Using the DB_PASSWORD already in .env"
fi

case "${DB_PASSWORD}" in
    *'#'*|*'$'*|*"'"*|*'"'*)
        echo "WARNING: DB_PASSWORD contains a character that systemd or the"
        echo "         shell may mangle (# \$ ' \"). Consider replacing it." ;;
esac

# ---------------------------------------------------------------------------
log "Creating database '${DB_NAME}' and user '${DB_USER}'"
# ---------------------------------------------------------------------------
# Two accounts, for 'localhost' and '127.0.0.1'.
#
# MariaDB treats these as distinct hosts. Connecting to 127.0.0.1 is a TCP
# connection; it only matches a @'localhost' grant because the server reverse-
# resolves the address. If skip-name-resolve is ever turned on, that stops
# happening and a @'localhost'-only grant breaks with a 1045 that looks
# identical to a wrong password. .env defaults DB_HOST to 127.0.0.1, so cover
# both.
#
# ALTER USER after CREATE USER IF NOT EXISTS is deliberate: if a previous
# partial run created the user with a different password, the CREATE is a
# silent no-op and Django would keep failing with 1045.
mariadb <<SQL
CREATE DATABASE IF NOT EXISTS \`${DB_NAME}\`
  CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;

CREATE USER IF NOT EXISTS '${DB_USER}'@'localhost' IDENTIFIED BY '${DB_PASSWORD}';
ALTER  USER              '${DB_USER}'@'localhost' IDENTIFIED BY '${DB_PASSWORD}';
GRANT ALL PRIVILEGES ON \`${DB_NAME}\`.* TO '${DB_USER}'@'localhost';

CREATE USER IF NOT EXISTS '${DB_USER}'@'127.0.0.1' IDENTIFIED BY '${DB_PASSWORD}';
ALTER  USER              '${DB_USER}'@'127.0.0.1' IDENTIFIED BY '${DB_PASSWORD}';
GRANT ALL PRIVILEGES ON \`${DB_NAME}\`.* TO '${DB_USER}'@'127.0.0.1';

FLUSH PRIVILEGES;
SQL

# ---------------------------------------------------------------------------
log "Verifying"
# ---------------------------------------------------------------------------
# Connect over TCP to the host Django will actually use, so this proves the
# real path rather than the unix socket.
DB_HOST="$(getenv DB_HOST)"; DB_HOST="${DB_HOST:-127.0.0.1}"
DB_PORT="$(getenv DB_PORT)"; DB_PORT="${DB_PORT:-3306}"

if mariadb -u "${DB_USER}" -p"${DB_PASSWORD}" \
     -h "${DB_HOST}" -P "${DB_PORT}" "${DB_NAME}" \
     -e "SELECT 1;" >/dev/null 2>&1; then
    echo "  ${DB_USER}@${DB_HOST}:${DB_PORT} -> ${DB_NAME}  OK"
else
    die "created the user but cannot log in as it over ${DB_HOST}:${DB_PORT}.
Check ${ENV_FILE}, and that mariadb is listening:  ss -lntp | grep 3306"
fi

echo
echo "Database ready. Next:"
echo "    cd ${REPO}/WebInterface"
echo "    set -a; . ${ENV_FILE}; set +a"
echo "    /opt/ed2/venv/bin/python manage.py migrate"
echo "    /opt/ed2/venv/bin/python manage.py collectstatic --noinput"
echo "    /opt/ed2/venv/bin/python manage.py createsuperuser"
