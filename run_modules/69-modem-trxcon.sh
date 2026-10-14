. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"
. "$(dirname "${BASH_SOURCE[0]}")/_lib/modem.sh"

MOD_REGISTER modem-trxcon "Couche 1 du MS GPRS (trxcon, second MS du fake_trx)"
MOD_REQUIRED[modem-trxcon]=0
MOD_DEPS[modem-trxcon]="sidecar-faketrx"
MOD_TIMEOUT[modem-trxcon]=20
MOD_ENABLED_IF[modem-trxcon]='[ "${MODEM_GPRS:-1}" = 1 ]'

: "${SC_TRXCON_BIN:=trxcon}"

mod_modem_trxcon_check() {
    command -v "$SC_TRXCON_BIN" >/dev/null 2>&1 || {
        mod_fail "binaire introuvable : $SC_TRXCON_BIN"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_modem_trxcon_status() { radio_alive modem-trxcon && have_unix "$MODEM_L2_SOCK"; }

# Meme lancement que le trxcon du MS#2 (67-sidecar-trxcon.sh), sur la base
# TRXD que fake_trx a ouverte pour ce second MS (--trx ...:MODEM_BB_PORT).
mod_modem_trxcon_start() {
    radio_dirs
    local log; log="$(radio_log modem-trxcon)"
    rm -f "$MODEM_L2_SOCK" 2>/dev/null
    # -f 0 (avance de tirage des rafales montantes) : sur un PDCH, trxcon
    # demande au modem le bloc montant X au DECODAGE du bloc descendant
    # (rafale X-1, trxcon_fsm.c TRXCON_EV_RX_DATA_IND) mais, avec l'avance par
    # defaut (2), il TIRE ce bloc a l'arrivee de la rafale X-2 : la primitive
    # arrive toujours apres le tirage et part a la poubelle (« dropping stale
    # Tx prim », un bloc de retard a chaque fois ; N3101 cote PCU). A 0, le
    # tirage a lieu a la rafale X : une trame pour repondre. MS#2 (voix) garde
    # son propre trxcon et son avance.
    mod_say "trxcon -p $MODEM_BB_PORT -s $MODEM_L2_SOCK (fbsb +$MODEM_TRXCON_FN_ADVANCE, avance UL $MODEM_TRXCON_TRX_ADVANCE)"
    mod_say "journal : $log"
    setsid stdbuf -oL -eL "$SC_TRXCON_BIN" -i "$TRX_BIND_IP" -b "$TRX_BB_IP" \
        -p "$MODEM_BB_PORT" -s "$MODEM_L2_SOCK" -C 1 -F "$MODEM_TRXCON_FN_ADVANCE" \
        -f "$MODEM_TRXCON_TRX_ADVANCE" >>"$log" 2>&1 </dev/null &
    radio_save_pid modem-trxcon $!
    mod_ok
}

mod_modem_trxcon_wait() {
    local log; log="$(radio_log modem-trxcon)"
    wait_until 5 "processus trxcon (modem)" radio_alive modem-trxcon || {
        modb_tail "$log" 20
        mod_fail "trxcon (modem) s'est arrete aussitot lance"; return $MOD_RC_FAIL; }
    wait_until "${MOD_TIMEOUT[modem-trxcon]}" "socket L1CTL $MODEM_L2_SOCK" \
        have_unix "$MODEM_L2_SOCK" || {
        modb_tail "$log" 20
        mod_hint "fake_trx a-t-il ete lance avec --trx ...:$MODEM_BB_PORT ? voir $(radio_log sidecar-faketrx)"
        mod_fail "socket L1CTL jamais creee"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_modem_trxcon_stop() { radio_kill modem-trxcon; rm -f "$MODEM_L2_SOCK"; return 0; }
