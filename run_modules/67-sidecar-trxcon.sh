. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-trxcon "Couche 1 du MS#2 (trxcon)"
MOD_REQUIRED[sidecar-trxcon]=1
MOD_DEPS[sidecar-trxcon]="sidecar-faketrx"
MOD_TIMEOUT[sidecar-trxcon]=20

: "${SC_TRXCON_BIN:=trxcon}"
: "${SC_TRXCON_FN_ADVANCE:=100}"

mod_sidecar_trxcon_check() {
    command -v "$SC_TRXCON_BIN" >/dev/null 2>&1 || {
        mod_fail "binaire introuvable : $SC_TRXCON_BIN"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_trxcon_status() { radio_alive sidecar-trxcon && have_unix "$SC_L2_SOCK"; }

mod_sidecar_trxcon_start() {
    radio_dirs
    local log; log="$(radio_log sidecar-trxcon)"
    rm -f "$SC_L2_SOCK" 2>/dev/null
    mod_say "trxcon -p $SC_BB_PORT -s $SC_L2_SOCK (avance de trame $SC_TRXCON_FN_ADVANCE)"
    mod_say "journal : $log"
    setsid stdbuf -oL -eL "$SC_TRXCON_BIN" -i "$TRX_BIND_IP" -b "$TRX_BB_IP" \
        -p "$SC_BB_PORT" -s "$SC_L2_SOCK" -C 1 -F "$SC_TRXCON_FN_ADVANCE" >>"$log" 2>&1 </dev/null &
    radio_save_pid sidecar-trxcon $!
    mod_ok
}

mod_sidecar_trxcon_wait() {
    local log; log="$(radio_log sidecar-trxcon)"
    wait_until 5 "processus trxcon" radio_alive sidecar-trxcon || {
        modb_tail "$log" 20
        mod_fail "trxcon s'est arrêté aussitôt lancé"; return $MOD_RC_FAIL; }
    wait_until "${MOD_TIMEOUT[sidecar-trxcon]}" "socket L1CTL $SC_L2_SOCK" \
        have_unix "$SC_L2_SOCK" || {
        modb_tail "$log" 20
        mod_hint "fake_trx répond-il sur $SC_BB_PORT ? voir $(radio_log sidecar-faketrx)"
        mod_fail "socket L1CTL jamais créée"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_trxcon_stop() { radio_kill sidecar-trxcon; rm -f "$SC_L2_SOCK"; return 0; }
