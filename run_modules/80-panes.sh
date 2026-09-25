MOD_REGISTER panes "Vue de supervision (tmux)"
MOD_REQUIRED[panes]=0
MOD_DEPS[panes]="tmux"
MOD_TIMEOUT[panes]=15

: "${TMUX_SESSION:=calypso}"
: "${CALYPSO_GSM_SNIFF:=${QEMU_TREE}/opt-gsm-scripts/gsm_sniff.py}"

_panes_count() { tmux list-panes -t "$TMUX_SESSION:all" 2>/dev/null | wc -l; }
_panes_enough() { [ "$(_panes_count)" -ge 2 ]; }

mod_panes_check() {
    command -v tmux >/dev/null 2>&1 || { mod_skip "tmux absent — pas de vue de supervision"; return $MOD_RC_SKIP; }
    if [ "${DRY:-0}" = 1 ]; then
        mod_say "simulation : existence de la session tmux non vérifiée"
        mod_ok; return $MOD_RC_OK
    fi
    tmux has-session -t "$TMUX_SESSION" 2>/dev/null || {
        mod_hint "le module tmux crée la session ; ./run.sh --only tmux"
        mod_fail "session tmux « $TMUX_SESSION » inexistante"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_panes_status() { tmux list-windows -t "$TMUX_SESSION" 2>/dev/null | grep -q '^[0-9]*: all'; }

mod_panes_start() {
    tmux new-window -t "$TMUX_SESSION" -n all \
        "clear; echo '=== osmocon ==='; tail -F ${LOG_DIR}/osmocon.log" || {
        mod_fail "tmux a refusé de créer la fenêtre « all »"
        return $MOD_RC_FAIL; }
    local specs=() spec name what cmd added=0
    specs+=("qemu|${LOG_DIR}/qemu.log")
    [ -r "$CALYPSO_GSM_SNIFF" ] && specs+=("burst|__BURST__")
    specs+=("mobile|${LOG_DIR}/mobile.log")
    specs+=("pont|/dev/shm/pont.log")
    for spec in "${specs[@]}"; do
        name="${spec%%|*}"; what="${spec##*|}"
        case "$what" in
            __BURST__)
                cmd="clear; echo '=== BURST decode (4731 + 5700-5702) ==='; python3 -u $CALYPSO_GSM_SNIFF burst" ;;
            *)  cmd="clear; echo '=== $name ==='; tail -F $what" ;;
        esac
        if tmux split-window -t "$TMUX_SESSION:all" "$cmd" 2>/dev/null; then
            added=$((added + 1))
        else
            mod_say "pane « $name » refusé par tmux (place insuffisante)"
        fi
        tmux select-layout -t "$TMUX_SESSION:all" tiled >/dev/null 2>&1
    done
    mod_say "fenêtre « all » : $((added + 1)) panes demandés"
    mod_ok
}

mod_panes_wait() {
    wait_until "${MOD_TIMEOUT[panes]}" "panes de la fenêtre « all »" _panes_enough || {
        mod_hint "tmux list-panes -t $TMUX_SESSION:all pour voir ce qui a survécu"
        mod_fail "la fenêtre « all » n'a pas gardé ses panes"
        return $MOD_RC_FAIL; }
    mod_say "panes vivants : $(_panes_count)"
    mod_ok
}

mod_panes_stop() {
    tmux kill-window -t "$TMUX_SESSION:all" 2>/dev/null
    return 0
}
