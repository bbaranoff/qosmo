MOD_REGISTER pty "PTY série de l'émulateur"
MOD_REQUIRED[pty]=1
MOD_DEPS[pty]="qemu"
MOD_TIMEOUT[pty]=60

: "${QEMU_MON_SOCK:=${RUN_DIR:-/tmp/calypso}/qemu-monitor.sock}"

_pty_from_monitor() {
    local sock="$1" label="$2"
    [ -S "$sock" ] || return 1
    command -v socat >/dev/null 2>&1 || return 1
    printf 'info chardev\n' \
        | timeout 3 socat -t 1 - "UNIX-CONNECT:$sock" 2>/dev/null \
        | grep -E "^${label}:" \
        | grep -oE '/dev/pts/[0-9]+' \
        | head -1
}

_pty_from_log() {
    local log="$1" label="$2"
    [ -r "$log" ] || return 1
    grep -a "redirected to /dev/pts/.* (label ${label})" "$log" 2>/dev/null \
        | grep -oE '/dev/pts/[0-9]+' \
        | head -1
}

_pty_resolve() {
    local log="${LOG_DIR}/qemu.log" p
    if [ -z "${OSMOCON_PTY:-}" ] || [ ! -c "${OSMOCON_PTY:-/nonexistent}" ]; then
        p="$(_pty_from_monitor "$QEMU_MON_SOCK" serial0)"
        [ -n "$p" ] || p="$(_pty_from_log "$log" serial0)"
        [ -n "$p" ] && OSMOCON_PTY="$p"
    fi
    [ -n "${OSMOCON_PTY:-}" ] && [ -c "$OSMOCON_PTY" ]
}

mod_pty_check() {
    if ! command -v socat >/dev/null 2>&1; then
        mod_say "socat absent : lecture du moniteur impossible, secours par le journal seulement"
    fi
    [ -S "$QEMU_MON_SOCK" ] || [ -r "${LOG_DIR}/qemu.log" ] || {
        mod_hint "le module qemu doit avoir démarré : ./run.sh --only qemu"
        mod_fail "ni moniteur ($QEMU_MON_SOCK) ni journal QEMU : rien à interroger"
        return $MOD_RC_FAIL
    }
    mod_ok
}

mod_pty_status() { return 1; }
mod_pty_start() { _pty_resolve; mod_ok; }

mod_pty_wait() {
    local deadline=$(( SECONDS + ${MOD_TIMEOUT[pty]} ))
    local qpid; qpid="$(cat "${RUN_DIR:-/tmp/calypso}/qemu.pid" 2>/dev/null || echo 0)"
    while (( SECONDS < deadline )); do
        if _pty_resolve; then
            export OSMOCON_PTY
            mod_say "serial0=$OSMOCON_PTY"
            mod_ok
            return $MOD_RC_OK
        fi
        if [ "$qpid" != 0 ] && ! kill -0 "$qpid" 2>/dev/null; then
            modb_tail "${LOG_DIR}/qemu.log" 20
            mod_hint "cause typique : machine type ou firmware invalide"
            mod_fail "QEMU (pid $qpid) s'est arrêté avant d'allouer son PTY"
            return $MOD_RC_FAIL
        fi
        sleep 0.5
    done
    mod_hint "vérifiez que QEMU est lancé avec « -serial pty » ; sinon : socat - UNIX-CONNECT:$QEMU_MON_SOCK puis « info chardev »"
    mod_fail "aucun PTY serial0 après ${MOD_TIMEOUT[pty]}s (moniteur $QEMU_MON_SOCK et journal muets)"
    return $MOD_RC_FAIL
}

mod_pty_stop() { return 0; }
