MOD_REGISTER attach "Reprise en main de la session tmux"
MOD_REQUIRED[attach]=0
MOD_DEPS[attach]="tmux"
MOD_TIMEOUT[attach]=10
MOD_ENABLED_IF[attach]='[ "${CALYPSO_NO_ATTACH:-0}" != 1 ] && [ -t 1 ]'

: "${TMUX_SESSION:=calypso}"
: "${CALYPSO_DEFAULT_WINDOW:=all}"

_attach_current_window() {
    tmux display-message -p -t "$TMUX_SESSION" '#W' 2>/dev/null
}

mod_attach_check() {
    command -v tmux >/dev/null 2>&1 || { mod_skip "tmux absent — rien à sélectionner"; return $MOD_RC_SKIP; }
    tmux has-session -t "$TMUX_SESSION" 2>/dev/null || {
        mod_hint "le module tmux (35) crée la session ; sans elle la pile tourne quand même, en arrière-plan"
        mod_skip "session tmux « $TMUX_SESSION » inexistante"
        return $MOD_RC_SKIP; }
    mod_ok
}

mod_attach_status() { [ "$(_attach_current_window)" = "$CALYPSO_DEFAULT_WINDOW" ]; }

mod_attach_start() {
    mkdir -p "$RUN_DIR" 2>/dev/null
    tmux select-window -t "$TMUX_SESSION:$CALYPSO_DEFAULT_WINDOW" 2>/dev/null \
        || tmux select-window -t "$TMUX_SESSION:qemu" 2>/dev/null \
        || mod_say "aucune des fenêtres « $CALYPSO_DEFAULT_WINDOW » / « qemu » n'existe"

    printf 'tmux attach -t %s\n' "$TMUX_SESSION" > "${RUN_DIR}/attach.cmd"
    mod_say "reprise en main : tmux attach -t $TMUX_SESSION"
    mod_say "fin de session  : tmux kill-session -t $TMUX_SESSION"
    mod_ok
}

mod_attach_wait() {
    wait_until "${MOD_TIMEOUT[attach]}" "session tmux « $TMUX_SESSION »" \
               tmux has-session -t "$TMUX_SESSION" || {
        mod_fail "la session tmux a disparu"
        return $MOD_RC_FAIL; }
    local w; w="$(_attach_current_window)"
    [ -n "$w" ] || { mod_fail "aucune fenêtre sélectionnable dans « $TMUX_SESSION »"
                     mod_hint "tmux list-windows -t $TMUX_SESSION"
                     return $MOD_RC_FAIL; }
    mod_say "fenêtre courante : $w"
    mod_ok
}

mod_attach_stop() { return 0; }
