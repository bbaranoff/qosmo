MOD_REGISTER tmux "Session d'observation tmux"
MOD_REQUIRED[tmux]=0
MOD_DEPS[tmux]="logs"
MOD_TIMEOUT[tmux]=10
MOD_ENABLED_IF[tmux]='command -v tmux >/dev/null 2>&1'

: "${TMUX_SESSION:=calypso}"
: "${TMUX_MODULES_DIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")/../tmux_modules" 2>/dev/null && pwd)}"

mod_tmux_check() {
    command -v tmux >/dev/null 2>&1 || {
        mod_hint "apt-get install tmux — ou lancez sans : les services tournent quand même"
        mod_fail "tmux introuvable"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_tmux_status() { tmux has-session -t "$TMUX_SESSION" 2>/dev/null; }

_tmux_charger() {
    local d="${TMUX_MODULES_DIR:-}"
    [ -n "$d" ] && [ -r "$d/_commun.sh" ] && [ -r "$d/bench.sh" ] || {
        mod_hint "attendu : <dépôt>/tmux_modules/_commun.sh et bench.sh"
        mod_fail "disposition tmux introuvable (TMUX_MODULES_DIR=${d:-<vide>})"
        return $MOD_RC_FAIL; }
    . "$d/_commun.sh"
    . "$d/bench.sh"
    mod_say "disposition : bench.sh"
    return 0
}

mod_tmux_start() {
    _tmux_charger || return $MOD_RC_FAIL
    tmux new-session -d -s "$TMUX_SESSION" -n "${TMUX_FENETRE_PREMIERE:-radio}" \
        "$(tmux_layout_premiere)" 2>/dev/null || {
        mod_hint "un serveur tmux périmé peut refuser la session : tmux kill-server puis relancez"
        mod_fail "création de la session « $TMUX_SESSION » refusée"; return $MOD_RC_FAIL; }
    _tmux_env_repropre
    tmux_layout
    _tmux_style
    tmux select-window -t "$TMUX_SESSION:${TMUX_FENETRE_PREMIERE:-radio}" 2>/dev/null
    mod_say "${TMUX_RESUME:-session créée}"
    mod_ok
}

mod_tmux_wait() {
    wait_until "${MOD_TIMEOUT[tmux]}" "session tmux « $TMUX_SESSION »" \
        tmux has-session -t "$TMUX_SESSION" || return $MOD_RC_FAIL
    [ "$(tmux list-windows -t "$TMUX_SESSION" 2>/dev/null | wc -l)" -ge 1 ] || {
        mod_fail "session créée mais aucune fenêtre"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_tmux_stop() { tmux kill-session -t "$TMUX_SESSION" 2>/dev/null; return 0; }
