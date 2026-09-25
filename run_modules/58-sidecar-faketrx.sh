. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-faketrx "Transceiver simulé du BTS#1"
MOD_REQUIRED[sidecar-faketrx]=1
MOD_DEPS[sidecar-faketrx]="sidecar-cfg"
MOD_TIMEOUT[sidecar-faketrx]=20

: "${FAKETRX_PY:=${OSMOCOM_BB:-${GSM_ROOT}/osmocom-bb}/src/target/trx_toolkit/fake_trx.py}"
: "${FAKETRX_PYTHON:=python3}"

mod_sidecar_faketrx_check() {
    command -v "$FAKETRX_PYTHON" >/dev/null 2>&1 || {
        mod_fail "interpréteur introuvable : $FAKETRX_PYTHON"; return $MOD_RC_FAIL; }
    [ -r "$FAKETRX_PY" ] || {
        mod_hint "posez FAKETRX_PY (trx_toolkit/fake_trx.py d'osmocom-bb)"
        mod_fail "fake_trx.py illisible : $FAKETRX_PY"; return $MOD_RC_FAIL; }
    local p
    for p in "$SC_TRX_PORT" "$SC_BB_PORT"; do
        if radio_udp_bound "$p" && ! radio_alive sidecar-faketrx; then
            mod_hint "ss -uanp | grep :$p — arrêtez le tenant, ou décalez SC_TRX_PORT/SC_BB_PORT"
            mod_fail "port UDP $p déjà pris par un autre processus"; return $MOD_RC_FAIL
        fi
    done
    mod_ok
}

mod_sidecar_faketrx_status() { radio_alive sidecar-faketrx; }

mod_sidecar_faketrx_start() {
    radio_dirs
    local log; log="$(radio_log sidecar-faketrx)"
    mod_say "fake_trx -P $SC_TRX_PORT (BTS#1) -p $SC_BB_PORT (bande de base)"
    mod_say "journal  : $log"
    setsid stdbuf -oL -eL "$FAKETRX_PYTHON" "$FAKETRX_PY" \
        -b "$TRX_BIND_IP" -R "$TRX_BTS_IP" -r "$TRX_BB_IP" \
        -P "$SC_TRX_PORT" -p "$SC_BB_PORT" >>"$log" 2>&1 </dev/null &
    radio_save_pid sidecar-faketrx $!
    mod_ok
}

mod_sidecar_faketrx_wait() {
    local log; log="$(radio_log sidecar-faketrx)"
    wait_until "${MOD_TIMEOUT[sidecar-faketrx]}" "port UDP $SC_TRX_PORT (côté BTS#1)" \
        radio_udp_bound "$SC_TRX_PORT" || {
        modb_tail "$log" 20
        mod_hint "fake_trx a-t-il démarré ? $log"
        mod_fail "port $SC_TRX_PORT jamais lié"; return $MOD_RC_FAIL; }
    wait_until 10 "port UDP $SC_BB_PORT (côté bande de base)" \
        radio_udp_bound "$SC_BB_PORT" || {
        mod_fail "côté BTS lié mais pas le côté bande de base ($SC_BB_PORT)"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_faketrx_stop() { radio_kill sidecar-faketrx; return 0; }
