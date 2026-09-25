MOD_REGISTER l2 "Client de couche 2 (mobile, MS#1)"
MOD_REQUIRED[l2]=0
MOD_DEPS[l2]="osmocon"
MOD_TIMEOUT[l2]=30

: "${L1CTL_SOCK_PATH:=/tmp/osmocom_l2}"
: "${CALYPSO_MOBILE_DEBUG:=DCS:DNB:DPLMN:DRR:DMM:DSIM:DCC:DMNCC:DSS:DLSMS:DPAG:DSUM:DSAP:DGPS:DMOB:DPRIM:DLUA:DGAPK:DLLAPD}"
: "${CALYPSO_PULSE_LATENCY_MSEC:=320}"
: "${CALYPSO_L1_READY_MARKER:=your code is running now}"
: "${CALYPSO_L1_READY_TIMEOUT:=20}"

_l2_log() { printf '%s' "${LOG_DIR}/mobile.log"; }
_l2_cfg() {
    if [ -n "${MOBILE_CFG:-}" ]; then printf '%s' "$MOBILE_CFG"
    elif [ -r "${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg" ]; then
        printf '%s' "${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg"
    else printf '%s' "${QEMU_CFGS:-${QEMU_TREE}/cfgs}/mobile_group1.cfg"; fi
}
_l2_pat() { printf '%s' "mobile -c $(_l2_cfg)"; }

mod_l2_check() {
    command -v mobile >/dev/null 2>&1 || {
        mod_hint "compilez osmocom-bb (layer23) ou ajoutez son répertoire au PATH"
        mod_fail "client L2 introuvable dans le PATH : mobile"
        return $MOD_RC_FAIL; }
    local cfg; cfg="$(_l2_cfg)"
    [ -r "$cfg" ] || {
        mod_hint "le module mobile-cfg doit avoir copié cfgs/mobile_group1.cfg vers \$OSMOCOM_HOME/bb/"
        mod_fail "configuration du mobile illisible : $cfg"
        return $MOD_RC_FAIL; }
    if [ "${DRY:-0}" = 1 ]; then
        mod_say "simulation : attente de la socket L1CTL et du marqueur L1 non jouées"
        mod_ok; return $MOD_RC_OK
    fi
    wait_until "${MOD_TIMEOUT[l2]}" "socket L1CTL $L1CTL_SOCK_PATH" \
               have_unix "$L1CTL_SOCK_PATH" || {
        mod_hint "c'est osmocon qui crée cette socket (option -s) : regardez le journal du module osmocon"
        mod_fail "socket L1CTL absente : $L1CTL_SOCK_PATH"
        return $MOD_RC_FAIL; }
    if ! wait_until "$CALYPSO_L1_READY_TIMEOUT" "L1 prête" \
                    log_has "${LOG_DIR}/osmocon.log" "$CALYPSO_L1_READY_MARKER"; then
        mod_say "AVERTISSEMENT : marqueur « $CALYPSO_L1_READY_MARKER » absent d'osmocon.log — on lance quand même"
    fi
    mod_ok
}

mod_l2_status() {
    local pid; pid="$(cat "${RUN_DIR}/l2.pid" 2>/dev/null || echo 0)"
    if [ "$pid" != 0 ] && kill -0 "$pid" 2>/dev/null; then
        tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qF "$(_l2_cfg)" && return 0
    fi
    have_proc "$(_l2_pat)"
}

mod_l2_start() {
    local log cfg
    log="$(_l2_log)"; cfg="$(_l2_cfg)"
    mkdir -p "$LOG_DIR" "$RUN_DIR" 2>/dev/null
    : > "$log"
    mod_say "journal  : $log"
    mod_say "cfg      : $cfg"
    mod_say "debug    : $CALYPSO_MOBILE_DEBUG"
    mod_say "latence  : PULSE_LATENCY_MSEC=$CALYPSO_PULSE_LATENCY_MSEC"
    PULSE_LATENCY_MSEC="$CALYPSO_PULSE_LATENCY_MSEC" \
        setsid mobile -c "$cfg" -d "$CALYPSO_MOBILE_DEBUG" >>"$log" 2>&1 </dev/null &
    printf '%s\n' "$!" > "${RUN_DIR}/l2.pid"
    mod_ok
}

mod_l2_wait() {
    local pid log; pid="$(cat "${RUN_DIR}/l2.pid" 2>/dev/null || echo 0)"
    log="$(_l2_log)"
    wait_until "${MOD_TIMEOUT[l2]}" "première trace du mobile" test -s "$log" || {
        mod_hint "lisez $log ; « same layer2-socket » = un autre client tient déjà $L1CTL_SOCK_PATH"
        mod_fail "mobile n'a produit aucune trace"
        return $MOD_RC_FAIL; }
    kill -0 "$pid" 2>/dev/null || {
        mod_hint "lisez la fin de $log : la cause est presque toujours dans les 10 dernières lignes"
        mod_fail "mobile a démarré puis s'est arrêté"
        return $MOD_RC_FAIL; }
    if command -v ss >/dev/null 2>&1; then
        if ss -x 2>/dev/null | grep -q "$L1CTL_SOCK_PATH"; then
            mod_say "socket L1CTL $L1CTL_SOCK_PATH : connexion observée"
        else
            mod_say "socket L1CTL $L1CTL_SOCK_PATH : pas de connexion visible via ss (indice, pas verdict)"
        fi
    fi
    mod_ok
}

mod_l2_stop() {
    local pid; pid="$(cat "${RUN_DIR}/l2.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "$(_l2_pat)" 2>/dev/null
    rm -f "${RUN_DIR}/l2.pid"
    return 0
}
