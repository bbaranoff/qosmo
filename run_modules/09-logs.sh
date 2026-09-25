MOD_REGISTER logs "Journaux et espace de travail"
MOD_REQUIRED[logs]=1
MOD_DEPS[logs]="config"
MOD_TIMEOUT[logs]=10

: "${LOG_ARCHIVES_KEEP:=10}"
: "${OSMO_LOG_DIR:=/var/log/osmocom}"
: "${OSMO_LOG_USER:=osmocom}"

mod_logs_check() {
    local d="${LOG_DIR:-}" p
    [ -n "$d" ] || { mod_fail "LOG_DIR non défini"; return $MOD_RC_FAIL; }
    p="$d"
    while [ ! -d "$p" ] && [ "$p" != "/" ] && [ "$p" != "." ]; do p="$(dirname "$p")"; done
    if [ ! -w "$p" ]; then
        mod_hint "posez LOG_DIR sur un chemin inscriptible (environnement/bench.env, LOG_DIR)"
        mod_fail "aucun parent inscriptible pour LOG_DIR=$d (bloqué à $p)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_logs_status() { return $MOD_RC_FAIL; }

mod_logs_start() {
    local _n _cible _lien _faits=0
    if [ "${CALYPSO_LIENS_ROOT:-1}" = 1 ] && [ -w /root ]; then
        for _n in qemu osmocon mobile bts sidecar-mobile; do
            _cible="$LOG_DIR/$_n.log"; _lien="/root/$_n.log"
            if [ -e "$_lien" ] && [ ! -L "$_lien" ]; then
                mod_say "⚠ /root/$_n.log est un vrai fichier — laissé en place"
                continue
            fi
            ln -sfn "$_cible" "$_lien" 2>/dev/null && _faits=$(( _faits + 1 ))
        done
        [ "$_faits" -gt 0 ] && mod_say "$_faits liens de compatibilité dans /root (→ $LOG_DIR)"
    fi
    [ -d "$OSMO_LOG_DIR" ] || mkdir -p "$OSMO_LOG_DIR" 2>/dev/null || true
    if id "$OSMO_LOG_USER" >/dev/null 2>&1; then
        chown "$OSMO_LOG_USER":"$OSMO_LOG_USER" "$OSMO_LOG_DIR" 2>/dev/null || true
    fi
    if [ -d "$OSMO_LOG_DIR" ]; then
        mod_say "journaux des services : $OSMO_LOG_DIR (propriétaire $OSMO_LOG_USER)"
    else
        mod_say "⚠ $OSMO_LOG_DIR absent — les services n'écriront que sur journald"
    fi
    mkdir -p "$LOG_DIR" "$LOG_DIR/mod" "${RUN_DIR:-/tmp/calypso}" "${CAPTURE_DIR:-$RUN_DIR/captures}" 2>/dev/null
    local osmocon_log="$LOG_DIR/osmocon.log"
    if [ -s "$osmocon_log" ]; then
        local arch="${osmocon_log%.log}.$(date +%Y%m%d_%H%M%S).log"
        if mv "$osmocon_log" "$arch" 2>/dev/null; then
            mod_say "osmocon.log du run précédent archivé -> $arch"
        fi
    fi
    ls -1t "${osmocon_log%.log}".[0-9]*.log 2>/dev/null \
        | tail -n +$((LOG_ARCHIVES_KEEP + 1)) | xargs -r rm -f 2>/dev/null || true
    for _n in qemu osmocon mobile bts; do
        : > "$LOG_DIR/$_n.log" 2>/dev/null
    done
    mod_say "LOG_DIR    = $LOG_DIR"
    mod_say "RUN_DIR    = ${RUN_DIR:-/tmp/calypso}"
    mod_ok
}

_logs_writable() {
    local probe="$LOG_DIR/.probe.$$"
    : > "$probe" 2>/dev/null || return 1
    rm -f "$probe" 2>/dev/null
    return 0
}

mod_logs_wait() {
    if ! wait_until "${MOD_TIMEOUT[logs]}" "espace de journaux" _logs_writable; then
        mod_hint "df -h $LOG_DIR — volume plein ou monté en lecture seule ?"
        mod_fail "écriture impossible dans $LOG_DIR (test réel, pas un test de droits)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_logs_stop() { return 0; }
