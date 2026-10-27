MOD_REGISTER qemu "Émulateur Calypso (QEMU)"
MOD_REQUIRED[qemu]=1
MOD_DEPS[qemu]="prereqs"
MOD_TIMEOUT[qemu]=30

: "${QEMU_LOG_MAX:=$((64 * 1024 * 1024))}"
: "${QEMU_LOG_HEAD:=$((8 * 1024 * 1024))}"

mod_qemu_check() {
    [ -x "${QEMU_BIN:-}" ] || { mod_fail "binaire QEMU absent : ${QEMU_BIN:-<non défini>}"
                                mod_hint "compilez-le : ./configure --target-list=arm-softmmu && ninja -C build qemu-system-arm"
                                return $MOD_RC_FAIL; }
    [ -r "${FIRMWARE_ELF:-}" ] || { mod_fail "firmware ARM introuvable : ${FIRMWARE_ELF:-<non défini>}"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_qemu_status() { have_proc "qemu-system-arm.*calypso"; }

_qemu_save_head() {
    local src="$1" dst="$2" taille="$3" i=0
    while [ "$i" -lt 120 ]; do
        if [ -s "$src" ] && [ "$(stat -c %s "$src" 2>/dev/null || echo 0)" -ge "$taille" ]; then
            head -c "$taille" "$src" > "$dst" 2>/dev/null
            return 0
        fi
        sleep 1; i=$(( i + 1 ))
    done
    [ -s "$src" ] && head -c "$taille" "$src" > "$dst" 2>/dev/null
    return 0
}

_qemu_log_guard() {
    local f="$1" max="$2" qpid="$3" n=0
    while kill -0 "$qpid" 2>/dev/null; do
        if [ -f "$f" ] && [ "$(stat -c %s "$f" 2>/dev/null || echo 0)" -gt "$max" ]; then
            n=$((n+1))
            printf '\n--- journal remis à zéro (%d fois) : plafond de %s octets atteint.\n--- La tête du journal est conservée dans qemu-tete.log ---\n' \
                   "$n" "$max" > "$f"
        fi
        sleep 5
    done
}

mod_qemu_start() {
    local qlog="${LOG_DIR}/qemu.log" qpid
    mod_say "machine  : calypso"
    mod_say "journal  : $qlog (plafond ${QEMU_LOG_MAX} o)"
    # C LAUNCHER `qosmo-grgsm` (tools/qosmo-launch, installed into
    # /usr/local/bin by `make install`). Same command line as below
    # (-M calypso, -cpu arm946, -serial pty x2, -monitor, -kernel), reads
    # l1s/last_rach from the ELF, relays QEMU stdout+stderr into qemu.log
    # (41-pty keeps reading "redirected to /dev/pts/N"), and two stable links:
    # $RUN_DIR/modem.pty (serial0, the one of osmocon) and $RUN_DIR/irda.pty.
    # It forwards SIGTERM to QEMU and dies with it. Without the launcher: the
    # direct qemu-system-arm line.
    # ARM gdbstub ACTIVE here too (tcp::$CALYPSO_GDB_PORT, 1234): QEMU only
    # stops when a client connects, so no cost as long as it is unused. This
    # is what 44-gdb-telnet.sh attaches on demand (telnet localhost 44444,
    # target running). CALYPSO_GDB_PORT=off to disable.
    : "${CALYPSO_GDB_PORT:=1234}"
    # -display none -parallel none, HERE AS IN THE LAUNCHER.
    # The calypso machine has no screen. Without -display, QEMU takes its
    # graphical default (SDL is compiled in) and wires its default devices to
    # virtual consoles; both serial ports being taken (-serial pty) and the
    # monitor too, only the parallel port is left: an empty SDL window titled
    # "parallel0", which grabs the keyboard, while the boot happens on the
    # ptys. -parallel none also removes the chardev (`info chardev` still shows
    # it as `vc` with -display none alone).
    # icount / in_asm trace, off by default (./run.sh --icount / --asm):
    #   QEMU_ICOUNT=auto|shift=N,…   -> -icount
    #   QEMU_ASM=1|FILE              -> -d in_asm -D FILE (default $LOG_DIR/qemu-asm.log)
    #   QEMU_ASM_FLAGS=…             replaces in_asm (in_asm,exec,nochain: every execution, too slow for attach)
    #   QEMU_ASM_FILTRE=…            adds -dfilter
    # Passed after `--` to the launcher: also works with an older installed launcher.
    local -a qx=()
    case "${QEMU_ICOUNT:-}" in ''|0|off) ;; *) qx+=(-icount "$QEMU_ICOUNT") ;; esac
    case "${QEMU_ASM:-}" in
        ''|0|off) ;;
        1) qx+=(-d "${QEMU_ASM_FLAGS:-in_asm}" -D "${LOG_DIR}/qemu-asm.log") ;;
        *) qx+=(-d "${QEMU_ASM_FLAGS:-in_asm}" -D "$QEMU_ASM") ;;
    esac
    [ -n "${QEMU_ASM:-}" ] && [ -n "${QEMU_ASM_FILTRE:-}" ] && qx+=(-dfilter "$QEMU_ASM_FILTRE")
    [ "${#qx[@]}" -gt 0 ] && mod_say "extra    : ${qx[*]}"
    local launcher="${QOSMO_LAUNCHER:-/usr/local/bin/qosmo}"
    if [ -x "$launcher" ]; then
        mod_say "lanceur  : $launcher (QOSMO_LAUNCHER) ; gdbstub ARM : $CALYPSO_GDB_PORT"
        setsid "$launcher" --qemu "$QEMU_BIN" -k "$FIRMWARE_ELF" --bin "$FIRMWARE_BIN" \
            --cpu arm946 --gdb "$CALYPSO_GDB_PORT" --rundir "$RUN_DIR" \
            --monitor "${RUN_DIR}/qemu-monitor.sock" -- "${qx[@]}" >>"$qlog" 2>&1 </dev/null &
        qpid=$!
    else
        mod_say "lanceur  : absent ($launcher) — ligne qemu-system-arm directe ; installez-le : make -C ${QEMU_TREE:-.}/tools/qosmo-launch install"
        setsid "$QEMU_BIN" -M calypso -cpu arm946 \
            -display none -parallel none \
            -serial pty -serial pty \
            -monitor "unix:${RUN_DIR}/qemu-monitor.sock,server,nowait" \
            -kernel "$FIRMWARE_ELF" "${qx[@]}" >>"$qlog" 2>&1 </dev/null &
        qpid=$!
    fi
    printf '%s\n' "$qpid" > "${RUN_DIR}/qemu.pid"
    ( trap '' HUP; _qemu_save_head "$qlog" "${LOG_DIR}/qemu-tete.log" "${QEMU_LOG_HEAD}" ) >/dev/null 2>&1 </dev/null &
    ( trap '' HUP; _qemu_log_guard "$qlog" "$QEMU_LOG_MAX" "$qpid" ) >/dev/null 2>&1 </dev/null &
    mod_say "tête     : ${LOG_DIR}/qemu-tete.log"
    mod_ok
}

_qemu_running() {
    local sock="$1"
    printf 'info status\n' \
        | timeout 3 socat -t 1 - "UNIX-CONNECT:$sock" 2>/dev/null \
        | grep -q 'VM status: running'
}

mod_qemu_wait() {
    local sock="${RUN_DIR}/qemu-monitor.sock" qlog="${LOG_DIR}/qemu.log"
    wait_until "${MOD_TIMEOUT[qemu]}" "socket du moniteur QEMU" have_unix "$sock" || return $MOD_RC_FAIL
    local qpid; qpid="$(cat "${RUN_DIR}/qemu.pid" 2>/dev/null || echo 0)"
    if ! kill -0 "$qpid" 2>/dev/null; then
        modb_tail "$qlog" 20
        mod_hint "cause typique : machine type ou firmware invalide"
        mod_fail "QEMU a démarré puis s'est arrêté"
        return $MOD_RC_FAIL
    fi
    if command -v socat >/dev/null 2>&1; then
        wait_until "${MOD_TIMEOUT[qemu]}" "machine virtuelle en marche" _qemu_running "$sock" || {
            modb_tail "$qlog" 20
            mod_hint "socat - UNIX-CONNECT:$sock puis « info status » pour voir l'état de la machine"
            return $MOD_RC_FAIL
        }
    else
        wait_until "${MOD_TIMEOUT[qemu]}" "PTY série alloués" grep -aq 'label serial1' "$qlog" || {
            modb_tail "$qlog" 20
            return $MOD_RC_FAIL
        }
    fi
    mod_ok
}

mod_qemu_stop() {
    local qpid; qpid="$(cat "${RUN_DIR}/qemu.pid" 2>/dev/null || echo 0)"
    [ "$qpid" != 0 ] && kill "$qpid" 2>/dev/null
    pkill -f "qemu-system-arm.*calypso" 2>/dev/null
    rm -f "${RUN_DIR}/qemu.pid" "${RUN_DIR}/qemu-monitor.sock"
    return 0
}
