# =============================================================================
#  44-gdb-telnet — console gdb de l'ARM, servie en telnet, non bloquante
# =============================================================================
#  RÔLE      Tenir un serveur (tools/gdb-telnet.py) qui, a chaque connexion
#            `telnet localhost 44444`, attache un gdb-multiarch au gdbstub de
#            QEMU (-gdb tcp::$CALYPSO_GDB_PORT) avec la cible EN MARCHE
#            (`continue &`). Ctrl-C arrete l'ARM, `continue &` repart, quit
#            ferme la session : gdb est tue et QEMU reprend l'ARM.
#            Tant que personne n'est connecte, rien ne touche a la cible :
#            on s'en sert quand on en a envie.
#  PRÉREQUIS QEMU lance avec son gdbstub (module qemu ; le lanceur qosmo-dsp
#            exporte CALYPSO_GDB_PORT et CALYPSO_GDB_ELF).
#  SUCCÈS    Le port telnet ecoute.
#  JOURNAL   $LOG_DIR/gdb-telnet.log
#  RÉGLAGES  CALYPSO_GDB_TELNET_PORT (defaut 44444, 0 = pas de serveur)
#            CALYPSO_GDB_TELNET_BIND (defaut 127.0.0.1 : une console gdb
#            controle la machine, on n'expose pas ca au reseau sans le vouloir)
# -----------------------------------------------------------------------------
MOD_REGISTER gdbtelnet "Console gdb ARM en telnet (non bloquante)"
MOD_REQUIRED[gdbtelnet]=0
MOD_DEPS[gdbtelnet]="qemu"
MOD_TIMEOUT[gdbtelnet]=10
MOD_ENABLED_IF[gdbtelnet]='[ "${CALYPSO_GDB_TELNET_PORT:-44444}" != 0 ] && [ -n "${CALYPSO_GDB_PORT:-}" ]'

# Le gdbstub QEMU est lui-meme non bloquant : QEMU tourne normalement tant que
# personne n'y est connecte. 1234 = le port que le lanceur (qosmo-dsp/qosmo-grgsm)
# passe a -gdb ; le poser ici garantit que 40-qemu et ce module parlent du meme.
: "${CALYPSO_GDB_PORT:=1234}"
: "${CALYPSO_GDB_TELNET_PORT:=44444}"
: "${CALYPSO_GDB_TELNET_BIND:=127.0.0.1}"
: "${GDB_TELNET_PY:=${QEMU_TREE:-.}/tools/gdb-telnet.py}"

_gdbtel_log() { printf '%s\n' "${LOG_DIR:-/root/calypso/logs}/gdb-telnet.log"; }

mod_gdbtelnet_check() {
    [ -r "$GDB_TELNET_PY" ] || { mod_skip "gdb-telnet.py absent ($GDB_TELNET_PY)"; return $MOD_RC_SKIP; }
    command -v gdb-multiarch >/dev/null 2>&1 || {
        mod_hint "apt-get install gdb-multiarch — ou CALYPSO_GDB_TELNET_PORT=0"
        mod_fail "gdb-multiarch introuvable"; return $MOD_RC_FAIL; }
    command -v python3 >/dev/null 2>&1 || { mod_fail "python3 introuvable"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_gdbtelnet_status() { have_proc "gdb-telnet.py" && have_port "$CALYPSO_GDB_TELNET_PORT"; }

mod_gdbtelnet_start() {
    local log; log="$(_gdbtel_log)"
    mod_say "telnet   : $CALYPSO_GDB_TELNET_BIND:$CALYPSO_GDB_TELNET_PORT   (telnet localhost $CALYPSO_GDB_TELNET_PORT)"
    mod_say "gdbstub  : 127.0.0.1:$CALYPSO_GDB_PORT"
    mod_say "elf      : ${CALYPSO_GDB_ELF:-${FIRMWARE_ELF:-}}"
    setsid python3 "$GDB_TELNET_PY" --port "$CALYPSO_GDB_TELNET_PORT" --bind "$CALYPSO_GDB_TELNET_BIND" \
        --stub "$CALYPSO_GDB_PORT" --elf "${CALYPSO_GDB_ELF:-${FIRMWARE_ELF:-}}" \
        </dev/null >>"$log" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/gdb-telnet.pid"
    mod_ok
}

mod_gdbtelnet_wait() {
    wait_until "${MOD_TIMEOUT[gdbtelnet]}" "port telnet $CALYPSO_GDB_TELNET_PORT" have_port "$CALYPSO_GDB_TELNET_PORT" || {
        mod_hint "regardez $(_gdbtel_log) — port deja pris ? CALYPSO_GDB_TELNET_PORT=autre"
        mod_fail "le serveur gdb-telnet n'ecoute pas"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_gdbtelnet_stop() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/gdb-telnet.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    for p in $(pgrep -f 'tools/gdb-telnet.py' 2>/dev/null); do [ "$p" = "$$" ] || kill "$p" 2>/dev/null; done
    rm -f "${RUN_DIR:-/tmp/calypso}/gdb-telnet.pid"
    return 0
}
