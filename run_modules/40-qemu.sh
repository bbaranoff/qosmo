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
    # [2026-09-03] LANCEUR C `qosmo-grgsm` (tools/qosmo-launch, installe dans
    # /usr/local/bin par `make install`). Meme ligne de commande que ci-dessous
    # (-M calypso, -cpu arm946, -serial pty x2, -monitor, -kernel), lecture de
    # l1s/last_rach dans l'ELF, relais de stdout+stderr de QEMU dans qemu.log
    # (41-pty continue de lire « redirected to /dev/pts/N »), et deux liens
    # stables : $RUN_DIR/modem.pty (serial0, celui d'osmocon) et $RUN_DIR/irda.pty.
    # Il transmet SIGTERM a QEMU et meurt avec lui. Sans lanceur : ligne historique.
    # [2026-09-03] gdbstub ARM ACTIF ici aussi (tcp::$CALYPSO_GDB_PORT, 1234) :
    # QEMU ne s'arrete que si un client s'y connecte, donc aucun cout tant qu'on
    # ne s'en sert pas. C'est ce que 44-gdb-telnet.sh attache a la demande
    # (telnet localhost 44444, cible en marche). CALYPSO_GDB_PORT=off pour couper.
    : "${CALYPSO_GDB_PORT:=1234}"
    # [2026-09-05] -display none -parallel none, ICI COMME DANS LE LANCEUR.
    # La machine calypso n'a pas d'ecran. Sans -display, QEMU prend son defaut
    # graphique (SDL est compile) et cable ses peripheriques par defaut sur des
    # consoles virtuelles ; les deux serie etant prises (-serial pty) et le
    # moniteur aussi, il ne restait que le port parallele : une fenetre SDL vide
    # intitulee « parallel0 », qui capte le clavier, pendant que le boot se passe
    # sur les pty. -parallel none supprime en plus le chardev (`info chardev` le
    # montrait encore en `vc` avec le seul -display none).
    local launcher="${QOSMO_LAUNCHER:-/usr/local/bin/qosmo}"
    if [ -x "$launcher" ]; then
        mod_say "lanceur  : $launcher (QOSMO_LAUNCHER) ; gdbstub ARM : $CALYPSO_GDB_PORT"
        setsid "$launcher" --qemu "$QEMU_BIN" -k "$FIRMWARE_ELF" --bin "$FIRMWARE_BIN" \
            --cpu arm946 --gdb "$CALYPSO_GDB_PORT" --rundir "$RUN_DIR" \
            --monitor "${RUN_DIR}/qemu-monitor.sock" >>"$qlog" 2>&1 </dev/null &
        qpid=$!
    else
        mod_say "lanceur  : absent ($launcher) — ligne qemu-system-arm directe ; installez-le : make -C ${QEMU_TREE:-.}/tools/qosmo-launch install"
        setsid "$QEMU_BIN" -M calypso -cpu arm946 \
            -display none -parallel none \
            -serial pty -serial pty \
            -monitor "unix:${RUN_DIR}/qemu-monitor.sock,server,nowait" \
            -kernel "$FIRMWARE_ELF" >>"$qlog" 2>&1 </dev/null &
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
