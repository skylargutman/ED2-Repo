#!/usr/bin/env bash
# ===========================================================================
# Explain a database "Access denied (1045)" by showing what Django actually
# resolves, versus what deploy/.env contains.
#
#     /opt/ed2/ED2-Repo/deploy/diagnose-db.sh
#
# Run as the SAME user that runs Django (ubuntu) and from the SAME shell that
# reproduced the error -- the environment is usually the culprit, so running
# it elsewhere hides the bug.
#
# Deliberately does not source .env: sourcing is what causes half the
# problems this script diagnoses.
# ===========================================================================
set -uo pipefail

REPO="${REPO:-/opt/ed2/ED2-Repo}"
ENV_FILE="${REPO}/deploy/.env"
VENV="${VENV:-/opt/ed2/venv}"

bold() { printf '\n\033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m OK\033[0m  %s\n' "$*"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$*"; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$*"; }

problems=0

# ---------------------------------------------------------------------------
bold "1. Is .env present and readable by $(id -un)?"
# ---------------------------------------------------------------------------
if [[ ! -f "${ENV_FILE}" ]]; then
    bad "${ENV_FILE} does not exist"
    echo "       cp ${REPO}/deploy/env.example ${ENV_FILE}"
    exit 1
fi

ls -l "${ENV_FILE}" | sed 's/^/       /'

if [[ -r "${ENV_FILE}" ]]; then
    ok "readable"
else
    bad "NOT readable by $(id -un) -- load_dotenv() fails SILENTLY and Django"
    echo "       falls back to the defaults in settings.py (user 'root', no password)."
    echo "       Fix:  sudo chown ubuntu:ubuntu ${ENV_FILE} && sudo chmod 600 ${ENV_FILE}"
    problems=$((problems + 1))
fi

# ---------------------------------------------------------------------------
bold "2. Is anything in the environment shadowing .env?"
# ---------------------------------------------------------------------------
# python-dotenv's load_dotenv() defaults to override=False, so a pre-existing
# environment variable WINS over the file. A shell that once ran
# `set -a; . .env; set +a` keeps serving stale values forever.
shadowed=0
while IFS= read -r var; do
    [[ -z "${var}" ]] && continue
    printf '       %s is exported\n' "${var}"
    shadowed=$((shadowed + 1))
done < <(env | awk -F= '/^(DB_|DJANGO_|REDIS_|MQTT_)/ { print $1 }' | sort)

if [[ ${shadowed} -eq 0 ]]; then
    ok "nothing exported -- .env is authoritative"
else
    bad "${shadowed} variable(s) exported; these OVERRIDE .env"
    echo "       Fix:  exec bash -l      (then re-run without sourcing .env)"
    problems=$((problems + 1))
fi

# ---------------------------------------------------------------------------
bold "3. What does .env say, and what does Django resolve?"
# ---------------------------------------------------------------------------
file_pw="$(awk 'index($0, "DB_PASSWORD=") == 1 { print substr($0, 13); exit }' "${ENV_FILE}" 2>/dev/null)"
file_pw="${file_pw%\"}"; file_pw="${file_pw#\"}"
file_pw="${file_pw%\'}"; file_pw="${file_pw#\'}"

printf '       .env    DB_PASSWORD len=%s first4=%s\n' \
       "${#file_pw}" "$(printf '%.4s' "${file_pw}")"

"${VENV}/bin/python" - "${REPO}" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1] + '/WebInterface')
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'EGNSite.settings')
try:
    import django
    django.setup()
    from django.conf import settings
    d = settings.DATABASES['default']
    p = d['PASSWORD'] or ''
    print(f"       Django  DB_PASSWORD len={len(p)} first4={p[:4]!r}")
    print(f"       Django  USER={d['USER']!r} HOST={d['HOST']!r} "
          f"PORT={d['PORT']!r} NAME={d['NAME']!r}")
    print(f"       Django  DEBUG={settings.DEBUG}")
except Exception as exc:
    print(f"       could not load Django settings: {exc}")
PY

echo
echo "       If the two first4 values differ, the environment is shadowing .env"
echo "       (see step 2). If Django shows USER='root' with an empty password,"
echo "       .env was not readable (see step 1)."

# ---------------------------------------------------------------------------
bold "4. Can we log in with exactly what .env holds?"
# ---------------------------------------------------------------------------
db_name="$(awk 'index($0, "DB_NAME=") == 1 { print substr($0, 9); exit }' "${ENV_FILE}")"
db_user="$(awk 'index($0, "DB_USER=") == 1 { print substr($0, 9); exit }' "${ENV_FILE}")"
db_host="$(awk 'index($0, "DB_HOST=") == 1 { print substr($0, 9); exit }' "${ENV_FILE}")"
db_name="${db_name:-egnsitedb}"; db_user="${db_user:-egnsite}"; db_host="${db_host:-127.0.0.1}"

if command -v mariadb >/dev/null 2>&1; then
    if mariadb -u "${db_user}" -p"${file_pw}" -h "${db_host}" "${db_name}" \
         -e "SELECT 1;" >/dev/null 2>&1; then
        ok "${db_user}@${db_host} -> ${db_name} accepts the .env password"
        echo "       So the credentials are correct and the problem is how Django"
        echo "       is reading them -- see steps 1 and 2."
    else
        bad "${db_user}@${db_host} -> ${db_name} REJECTS the .env password"
        echo "       The database and .env disagree. Fix both at once:"
        echo "           sudo ${REPO}/deploy/setup-db.sh --new-password"
        problems=$((problems + 1))
    fi
else
    warn "mariadb client not found, skipping"
fi

# ---------------------------------------------------------------------------
bold "5. Root-owned files that would break the services"
# ---------------------------------------------------------------------------
# egnsite-web runs as ubuntu; anything root created (usually by running
# manage.py under sudo) will fail for it later.
root_owned="$(find "${REPO}" -user root -not -path '*/.git/*' 2>/dev/null | head -20)"
if [[ -z "${root_owned}" ]]; then
    ok "none"
else
    warn "root-owned paths under ${REPO}:"
    printf '       %s\n' ${root_owned}
    echo "       Fix:  sudo chown -R ubuntu:www-data ${REPO}"
fi

# ---------------------------------------------------------------------------
if [[ ${problems} -eq 0 ]]; then
    bold "No blocking problems found."
else
    bold "${problems} problem(s) found -- see the Fix lines above."
fi
exit 0
