MOD_REGISTER teardown "Nettoyage du run précédent"
MOD_REQUIRED[teardown]=1
MOD_DEPS[teardown]="logs"
MOD_TIMEOUT[teardown]=25

_td_patterns() {
    cat <<'EOF'
f:qemu-system-arm
x:osmo-bts-trx
x:osmocon
x:mobile
x:trxcon
f:pont/pont.py
f:pont/pont_dsp.py
EOF
}

_td_port_busy() {
    ss -ln 2>/dev/null | grep -qE "[:.]${1}[[:space:]]"
}

_td_ports() {
    printf '%s\n' "${PORT_TRXD_CLOCK:-5700}" "${PORT_TRXD_CTRL:-5701}" "${PORT_TRXD_DATA:-5702}" \
                  "${PORT_GSMTAP_SI:-4730}" "${PORT_GSMTAP_SCH:-4731}"
}

_td_sockets() {
    printf '%s\n' "${L1CTL_SOCK_PATH:-/tmp/osmocom_l2}" \
                  "${SC_L2_SOCK:-/tmp/ms2_l2}" \
                  "${QEMU_MON_SOCK:-${RUN_DIR:-/tmp/calypso}/qemu-monitor.sock}"
}

_td_live() {
    local _p _st
    for _p in $(pgrep "$1" "$2" 2>/dev/null); do
        _st="$(ps -o stat= -p "$_p" 2>/dev/null | tr -d ' ')"
        case "$_st" in
            ''|Z*) continue ;;
            *)     return 0 ;;
        esac
    done
    return 1
}

_td_leftovers() {
    local line kind pat out="" p
    while IFS= read -r line; do
        kind="${line%%:*}"; pat="${line#*:}"
        case "$kind" in
            x) _td_live -x "$pat" && out="$out $pat" ;;
            f) _td_live -f "$pat" && out="$out $pat" ;;
        esac
    done < <(_td_patterns)
    while IFS= read -r p; do
        _td_port_busy "$p" && out="$out port:$p"
    done < <(_td_ports)
    while IFS= read -r p; do
        [ -e "$p" ] && out="$out socket:$p"
    done < <(_td_sockets)
    printf '%s' "${out# }"
}
_td_clean() { [ -z "$(_td_leftovers)" ]; }

mod_teardown_check() {
    local t
    for t in pgrep pkill; do
        command -v "$t" >/dev/null 2>&1 || {
            mod_hint "installez procps (paquet procps / procps-ng)"
            mod_fail "$t introuvable : impossible de garantir une machine propre"
            return $MOD_RC_FAIL
        }
    done
    command -v ss >/dev/null 2>&1 || mod_say "ss absent : la libération des ports UDP ne sera pas vérifiée"
    mod_ok
}

mod_teardown_status() { _td_clean; }

_td_kill_all() {
    local line kind pat
    while IFS= read -r line; do
        kind="${line%%:*}"; pat="${line#*:}"
        case "$kind" in
            x) _td_live -x "$pat" && { mod_say "kill -x $pat"; pkill -9 -x "$pat" 2>/dev/null; } ;;
            f) _td_live -f "$pat" && { mod_say "kill -f $pat"; pkill -9 -f "$pat" 2>/dev/null; } ;;
        esac
    done < <(_td_patterns)
    return 0
}

_td_kill_ports() {
    command -v ss >/dev/null 2>&1 || { mod_say "ss absent : passe par port impossible"; return 0; }
    local p pid cmd busy=0 named=0
    while IFS= read -r p; do
        _td_port_busy "$p" || continue
        busy=1
        for pid in $(ss -lnp "sport = :$p" 2>/dev/null \
                     | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
            named=1
            [ "$pid" -gt 1 ] 2>/dev/null || continue
            [ "$pid" = "$$" ] && continue
            cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
            mod_say "port $p tenu par PID $pid (${cmd:-cmdline illisible}) — kill -9"
            kill -9 "$pid" 2>/dev/null
        done
    done < <(_td_ports)
    if [ "$busy" = 1 ] && [ "$named" = 0 ]; then
        mod_say "ports occupes mais ss n'a rendu aucun pid= (ss sans -p, ou proprietaire hors de notre namespace) — inspectez : ss -lnp"
    fi
    return 0
}

mod_teardown_start() {
    if [ -n "${TMUX:-}" ]; then
        mod_say "lancé depuis tmux : kill-server évité, on ne touche qu'à la session ${TMUX_SESSION:-calypso}"
        tmux kill-session -t "${TMUX_SESSION:-calypso}" 2>/dev/null
    else
        tmux kill-server 2>/dev/null
    fi
    _td_kill_all
    local p
    while IFS= read -r p; do rm -f "$p" 2>/dev/null; done < <(_td_sockets)
    rm -f /dev/shm/calypso_kc /dev/shm/calypso_kc_l1 2>/dev/null
    rm -f "${RUN_DIR:-/tmp/calypso}/qemu.pid" 2>/dev/null
    mod_ok
}

mod_teardown_wait() {
    local half=$(( ${MOD_TIMEOUT[teardown]} / 2 ))
    if wait_until "$half" "machine propre" _td_clean; then mod_ok; return $MOD_RC_OK; fi
    mod_say "restes après $half s : $(_td_leftovers) — deuxième passe"
    _td_kill_all
    _td_kill_ports
    if wait_until "$half" "machine propre" _td_clean; then mod_ok; return $MOD_RC_OK; fi
    mod_hint "identifiez le tenace : pgrep -a -f <motif> ; ss -lnp | grep <port>"
    mod_fail "restes du run précédent : $(_td_leftovers)"
    return $MOD_RC_FAIL
}

mod_teardown_stop() {
    _td_kill_all
    local p
    while IFS= read -r p; do rm -f "$p" 2>/dev/null; done < <(_td_sockets)
    return 0
}
