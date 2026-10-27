# =============================================================================
#  44-gdb-telnet — gdb console of the ARM, served over telnet, non-blocking
# =============================================================================
#  ROLE      Keep a server (tools/gdb-telnet.py) which, on each
#            `telnet localhost 44444` connection, attaches a gdb-multiarch to
#            the QEMU gdbstub (-gdb tcp::$CALYPSO_GDB_PORT) with the target
#            RUNNING (`continue &`). Ctrl-C stops the ARM, `continue &` resumes,
#            quit closes the session: gdb is killed and QEMU resumes the ARM.
#            As long as nobody is connected, nothing touches the target: it is
#            used on demand.
#  REQUIRES  QEMU started with its gdbstub (qemu module; the qosmo-dsp launcher
#            exports CALYPSO_GDB_PORT and CALYPSO_GDB_ELF).
#  SUCCESS   The telnet port is listening.
#  LOG       $LOG_DIR/gdb-telnet.log
#  SETTINGS  CALYPSO_GDB_TELNET_PORT (default 44444, 0 = no server)
#            CALYPSO_GDB_TELNET_BIND (default 127.0.0.1: a gdb console
#            controls the machine, it is not exposed to the network unwittingly)
# -----------------------------------------------------------------------------
MOD_REGISTER gdbtelnet "Console gdb ARM en telnet (non bloquante)"
MOD_REQUIRED[gdbtelnet]=0
MOD_DEPS[gdbtelnet]="qemu"
MOD_TIMEOUT[gdbtelnet]=10
MOD_ENABLED_IF[gdbtelnet]='[ "${CALYPSO_GDB_TELNET_PORT:-44444}" != 0 ] && [ -n "${CALYPSO_GDB_PORT:-}" ]'

# The QEMU gdbstub is itself non-blocking: QEMU runs normally as long as nobody
# is connected to it. 1234 = the port the launcher (qosmo-dsp/qosmo-grgsm)
# passes to -gdb; setting it here guarantees 40-qemu and this module agree.
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
