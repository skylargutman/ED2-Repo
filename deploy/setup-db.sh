#!/usr/bin/env bash
# ===========================================================================
# Create the MariaDB database and user, and make deploy/.env agree with it.
#
#     sudo /opt/ed2/ED2-Repo/deploy/setup-db.sh
#     sudo /opt/ed2/ED2-Repo/deploy/setup-db.sh --new-password
#
# --new-password forces a fresh generated password even if .env already has
# one, and applies it to both MariaDB and .env so they cannot drift apart.
# Use it if the current password contains characters that the shell or systemd
# may mangle -- see the alphabet note below.
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

FORCE_NEW_PASSWORD=0
for arg in "$@"; do
    case "${arg}" in
        --new-password) FORCE_NEW_PASSWORD=1 ;;
        -h|--help)
            sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "unknown option: ${arg}" >&2; exit 2 ;;
    esac
done

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
# awk stops at the first match itself rather than piping into `head`. Under
# `set -o pipefail` a consumer that exits early (head) sends SIGPIPE to the
# producer, the pipeline reports 141, and `set -e` aborts the script.
getenv() {
    awk -v k="$1" 'index($0, k "=") == 1 { print substr($0, length(k) + 2); exit }' \
        "${ENV_FILE}" | sed 's/^["'\'']//; s/["'\'']$//'
}

DB_NAME="$(getenv DB_NAME)";     DB_NAME="${DB_NAME:-egnsitedb}"
DB_USER="$(getenv DB_USER)";     DB_USER="${DB_USER:-egnsite}"
DB_PASSWORD="$(getenv DB_PASSWORD)"

# ---------------------------------------------------------------------------
# Generate a password if the placeholder is still in place.
#
# Restricted alphabet on purpose. Measured behaviour of a '$' in the value:
#
#   python-dotenv (Django)   abc$def -> abc$def   OK, only ${...} interpolates
#   shell `. .env`           abc$def -> abc       TRUNCATED at the $
#   shell, "abc$def" quoted  abc$def -> abc       TRUNCATED even so
#
# '#' is worse: it begins a comment in a systemd EnvironmentFile. Sticking to
# alphanumerics means the value survives every consumer identically, which
# matters because a mismatch surfaces only as an opaque 1045 Access denied.
# ---------------------------------------------------------------------------
gen_password() {
    # NOTE: deliberately no `| head -c N`. That makes the upstream producer
    # take SIGPIPE, which under `set -o pipefail` + `set -e` aborts the script
    # with exit 141. Consume the generator's output fully, then slice in bash.
    local raw=""
    if command -v openssl >/dev/null 2>&1; then
        raw="$(openssl rand -base64 48 | tr -dc 'A-Za-z0-9')"
    else
        raw="$(head -c 512 /dev/urandom | tr -dc 'A-Za-z0-9')"
    fi
    printf '%s' "${raw:0:32}"
}

if [[ ${FORCE_NEW_PASSWORD} -eq 1 || -z "${DB_PASSWORD}" || "${DB_PASSWORD}" == "replace-me" ]]; then
    DB_PASSWORD="$(gen_password)"
    if [[ ${FORCE_NEW_PASSWORD} -eq 1 ]]; then
        log "Generating a NEW database password (--new-password)"
    else
        log "Generated a database password and writing it to .env"
    fi

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
    *"'"*)
        die "DB_PASSWORD contains a single quote, which would break the SQL
below. Re-run with:  sudo $0 --new-password" ;;
    *'#'*|*'$'*|*'\'*|*'\"'*)
        echo
        echo "  WARNING: DB_PASSWORD contains one of  # \$ \\ \"  ."
        echo "           Django reads it correctly, but the value is fragile:"
        echo "           '\$' truncates if .env is ever sourced in a shell, and"
        echo "           '#' begins a comment in a systemd EnvironmentFile."
        echo "           Strongly consider:  sudo $0 --new-password"
        echo ;;
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
echo "Database ready. Next (settings.py reads .env itself -- do NOT source it,"
echo "and do NOT use sudo):"
echo "    cd ${REPO}/WebInterface"
echo "    /opt/ed2/venv/bin/python manage.py migrate"
echo "    /opt/ed2/venv/bin/python manage.py collectstatic --noinput"
echo "    /opt/ed2/venv/bin/python manage.py createsuperuser"
echo

# If the invoking shell already exported DB_* (e.g. from a previous
# `set -a; . .env`), those stale values SHADOW the file: python-dotenv's
# load_dotenv() defaults to override=False. The password just written here
# would then be ignored and Django would fail with 1045.
if [[ -n "${SUDO_USER:-}" ]] && sudo -u "${SUDO_USER}" env 2>/dev/null | grep -q '^DB_PASSWORD='; then
    echo "  WARNING: your shell already exports DB_PASSWORD. It will shadow the"
    echo "           value just written to .env and Django will fail with 1045."
    echo "           Start a clean shell before running manage.py:"
    echo "               exec bash -l"
    echo
fi
