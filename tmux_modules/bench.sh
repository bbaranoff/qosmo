TMUX_FENETRE_PREMIERE=radio
TMUX_RESUME="6 fenêtres : radio · coeur · ms1 · ms2 (side-car) · voix · shell"

tmux_layout_premiere() {
    printf "tail -n 200 -F '%s/qemu.log' 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "${LOG_DIR}"
}

tmux_layout() {
    local L="${LOG_DIR}"
    tmux select-pane -t "$TMUX_SESSION:radio" -T \
        "QEMU | baseband emule (ARM7, couche 1 gr-gsm) - traces du modele" 2>/dev/null
    _paint radio "$C_RADIO"
    _split radio "osmocon | lien serie firmware - romload puis transport L1CTL" \
        "$(_tail "'$L/osmocon.log'")" "$C_RADIO"
    _split radio "pont | transceiver du BTS#0 - bursts TRXD 5700-5702 <-> QEMU 4730/4731" \
        "$(_tail "'/dev/shm/pont.log'")" "$C_RADIO"
    _fenetre_coeur
    _w     ms1 "MS#1 | pile L2/L3 du telephone emule - camp, LU, appels" \
        "$(_tail "'$L/mobile.log'")" "$C_MS1"
    _split ms1 "BTS#0 | sa station de base - ARFCN ${PONT_ARFCN:-514}, unit-id 6001" \
        "$(_tail "'$L/bts.log'")" "$C_MS1"
    _split ms1 "GSMTAP | capture pcap - a ouvrir dans Wireshark" \
        "$(_tail "'$L/gsmtap.log'")" "$C_MS1"
    _w     ms2 "MS#2 | second abonne - campe sur le BTS#1, appelle le MS#1" \
        "$(_tail "'$L/sidecar-mobile.log'")" "$C_MS2"
    _split ms2 "trxcon | sa couche 1 - TRXD <-> L1CTL (remplace le firmware)" \
        "$(_tail "'$L/sidecar-trxcon.log'")" "$C_MS2"
    _split ms2 "BTS#1 | seconde station - ARFCN ${SC_ARFCN:-516}, unit-id ${SC_UNIT_ID:-6002}" \
        "$(_tail "'$L/sidecar-bts.log'")" "$C_MS2"
    _split ms2 "fake_trx | commutateur de bursts UDP ${SC_TRX_PORT:-5720} <-> ${SC_BB_PORT:-6720}" \
        "$(_tail "'$L/sidecar-faketrx.log'")" "$C_MS2"
    _fenetre_voix
    _fenetre_shell \
        "MS#1  : telnet 127.0.0.1 4247   puis  show ms" \
        "MS#2  : telnet 127.0.0.1 ${SC_MOBILE_VTY_PORT:-4248}   puis  call 1 10001"
}
