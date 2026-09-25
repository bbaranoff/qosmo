. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-mobile "Second abonné (MS#2)"
MOD_REQUIRED[sidecar-mobile]=0
MOD_DEPS[sidecar-mobile]="sidecar-trxcon"
MOD_TIMEOUT[sidecar-mobile]=30

: "${SC_MOBILE_BIN:=mobile}"
: "${SC_MOBILE_DELAY:=3}"
: "${CALYPSO_PULSE_LATENCY_MSEC:=320}"

mod_sidecar_mobile_check() {
    command -v "$SC_MOBILE_BIN" >/dev/null 2>&1 || {
        mod_fail "binaire introuvable : $SC_MOBILE_BIN"; return $MOD_RC_FAIL; }
    [ -r "$SC_MOBILE_CFG" ] || {
        mod_hint "le module « sidecar-cfg » doit avoir posé ce fichier"
        mod_fail "configuration illisible : $SC_MOBILE_CFG"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_mobile_status() { radio_alive sidecar-mobile; }

mod_sidecar_mobile_start() {
    radio_dirs
    local log; log="$(radio_log sidecar-mobile)"
    wait_until 30 "socket L1CTL $SC_L2_SOCK" have_unix "$SC_L2_SOCK" || {
        mod_fail "socket L1CTL absente : $SC_L2_SOCK"; return $MOD_RC_FAIL; }
    if [ "${SC_MOBILE_DELAY}" != 0 ]; then
        mod_say "attente de ${SC_MOBILE_DELAY}s avant lancement"
        sleep "$SC_MOBILE_DELAY"
    fi
    mod_say "mobile -c $SC_MOBILE_CFG (VTY $SC_MOBILE_VTY_PORT)"
    mod_say "journal : $log"
    mod_say "latence : PULSE_LATENCY_MSEC=${CALYPSO_PULSE_LATENCY_MSEC:-<defaut greffon>}"
    mod_say "debug    : $CALYPSO_MOBILE_DEBUG"
    PULSE_LATENCY_MSEC="$CALYPSO_PULSE_LATENCY_MSEC" \
        setsid stdbuf -oL -eL "$SC_MOBILE_BIN" -c "$SC_MOBILE_CFG" \
        -d "$CALYPSO_MOBILE_DEBUG" >>"$log" 2>&1 </dev/null &
    radio_save_pid sidecar-mobile $!
    mod_ok
}

mod_sidecar_mobile_wait() {
    wait_until "${MOD_TIMEOUT[sidecar-mobile]}" "VTY du MS#2 ($SC_MOBILE_VTY_PORT)" \
        have_port "$SC_MOBILE_VTY_PORT" || {
        modb_tail "$(radio_log sidecar-mobile)" 20
        mod_hint "port déjà pris ? ss -tlnp | grep :$SC_MOBILE_VTY_PORT"
        mod_fail "VTY $SC_MOBILE_VTY_PORT jamais en écoute"; return $MOD_RC_FAIL; }
    mod_say "prêt — telnet 127.0.0.1 $SC_MOBILE_VTY_PORT puis « show ms »"
    mod_ok
}

mod_sidecar_mobile_stop() { radio_kill sidecar-mobile; return 0; }
