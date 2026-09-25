MOD_REGISTER osmocon "Passerelle osmocon"
MOD_REQUIRED[osmocon]=1
MOD_DEPS[osmocon]="qemu pty"
MOD_TIMEOUT[osmocon]=30

: "${OSMOCON_MODEL:=romload}"
: "${OSMOCON_INTER_BYTE_DELAY:=100}"
: "${OSMOCON_DEBUG:=tr}"
: "${L1CTL_SOCK_PATH:=/tmp/osmocom_l2}"

_osmocon_log() { printf '%s\n' "${LOG_DIR}/osmocon.log"; }
_osmocon_vivant() { pgrep -f "osmocon .*-p " >/dev/null 2>&1; }

mod_osmocon_check() {
    [ -x "${OSMOCON:-}" ] || {
        mod_hint "réglez OSMOCOM_BB dans environnement/bench.env, ou compilez osmocon"
        mod_fail "osmocon introuvable : ${OSMOCON:-<non défini>}"; return $MOD_RC_FAIL; }
    [ -r "${FIRMWARE_BIN:-}" ] || {
        mod_hint "réglez FIRMWARE_BIN dans environnement/bench.env"
        mod_fail "image firmware illisible : ${FIRMWARE_BIN:-<non défini>}"; return $MOD_RC_FAIL; }
    [ -n "${OSMOCON_PTY:-}" ] && [ -c "$OSMOCON_PTY" ] || {
        mod_hint "le module « pty » doit avoir résolu le port série de QEMU ; regardez son journal"
        mod_fail "PTY série indisponible : ${OSMOCON_PTY:-<non résolu>}"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_osmocon_status() { _osmocon_vivant && have_unix "$L1CTL_SOCK_PATH"; }

mod_osmocon_start() {
    local log; log="$(_osmocon_log)"
    rm -f "$L1CTL_SOCK_PATH" 2>/dev/null
    mod_say "PTY      : $OSMOCON_PTY"
    mod_say "firmware : $FIRMWARE_BIN"
    mod_say "modèle   : $OSMOCON_MODEL"
    mod_say "L1CTL    : $L1CTL_SOCK_PATH"
    setsid stdbuf -oL -eL "$OSMOCON" \
        -m "$OSMOCON_MODEL" \
        -i "$OSMOCON_INTER_BYTE_DELAY" \
        -p "$OSMOCON_PTY" \
        -s "$L1CTL_SOCK_PATH" \
        "$FIRMWARE_BIN" \
        -d "$OSMOCON_DEBUG" >>"$log" 2>&1 </dev/null &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/osmocon.pid"
    mod_ok
}

mod_osmocon_wait() {
    local log; log="$(_osmocon_log)"
    wait_until 5 "processus osmocon" _osmocon_vivant || {
        modb_tail "$log" 15
        mod_hint "PTY $OSMOCON_PTY refusé ? regardez $log"
        mod_fail "osmocon s'est arrêté aussitôt lancé"
        return $MOD_RC_FAIL
    }
    if ! wait_until "${MOD_TIMEOUT[osmocon]}" "socket L1CTL $L1CTL_SOCK_PATH" \
         have_unix "$L1CTL_SOCK_PATH"; then
        modb_tail "$log" 15
        if _osmocon_vivant; then
            mod_hint "osmocon vit mais n'a pas fini son romload : modèle inadapté (OSMOCON_MODEL=$OSMOCON_MODEL — « romload » attendu sous QEMU) ou PTY qui ne mène pas au baseband"
            mod_fail "aucune socket L1CTL : le firmware n'a pas été chargé"
        else
            mod_hint "regardez $log"
            mod_fail "osmocon est mort pendant le chargement"
        fi
        return $MOD_RC_FAIL
    fi
    mod_say "firmware chargé, socket L1CTL ouverte"
    mod_ok
}

mod_osmocon_stop() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/osmocon.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "osmocon .*-p " 2>/dev/null
    rm -f "${RUN_DIR:-/tmp/calypso}/osmocon.pid" "$L1CTL_SOCK_PATH"
    return 0
}
