TMUX_FENETRE_PREMIERE=radio
TMUX_RESUME="6 fenêtres : radio · coeur · ms1 · ms2 (side-car) · voix · shell"

tmux_layout_premiere() {
    printf "tail -n 200 -F '%s/qemu.log' 2>/dev/null | stdbuf -oL tr -d '\\\\007'%s || sleep infinity" \
           "${LOG_DIR}" "$(_couleurs)"
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
    # ARM asm trace when requested (--asm / --assembly-logs)
    if { [ -n "${QEMU_ASM:-}" ] && [ "${QEMU_ASM}" != 0 ]; } || [ "${ASSEMBLY_LOGS:-0}" = 1 ]; then
        _split radio "asm | blocs ARM (-d ${QEMU_ASM_FLAGS:-in_asm}), 60 dernieres lignes, 2x/s - anneau : tbring-dump" \
            "$(_asm_watch)" "$C_RADIO"
    fi
    _fenetre_coeur
    # MS1_LOG (osmo-operator, 09-mobile-cfg-tmux.sh): rvinterf.log with Magnetite
    _w     ms1 "MS#1 | ${MS1_LOG_NOM:-mobile} du telephone emule - camp, LU, appels" \
        "$(_tail "'${MS1_LOG:-$L/mobile.log}'")" "$C_MS1"
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
