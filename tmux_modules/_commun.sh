C_RADIO=colour81
C_COEUR=colour114
C_MS1=colour179
C_MS2=colour215
C_VOIX=colour176
SUIVRE="tail -n 100 -F"
: "${OSMO_LOG_DIR:=/var/log/osmocom}"

_paint() {
    [ -n "${2:-}" ] || return 0
    tmux select-pane -t "$TMUX_SESSION:$1" -P "fg=$2" 2>/dev/null || true
}

_w() {
    tmux new-window -t "$TMUX_SESSION" -n "$1" "$3" 2>/dev/null \
        && tmux select-pane -t "$TMUX_SESSION:$1" -T "$2" 2>/dev/null \
        && _paint "$1" "${4:-}"
}

_split() {
    tmux split-window -t "$TMUX_SESSION:$1" "$3" 2>/dev/null \
        && tmux select-pane -t "$TMUX_SESSION:$1" -T "$2" 2>/dev/null \
        && _paint "$1" "${4:-}" \
        && tmux select-layout -t "$TMUX_SESSION:$1" tiled 2>/dev/null
}

_tail() {
    printf "%s %s 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "$SUIVRE" "$*"
}

_fenetre_coeur() {
    local O="${OSMO_LOG_DIR:-/var/log/osmocom}"
    _w     coeur "MSC | commutation circuit - appels, SMS, authentification" \
        "$(_tail "'$O/osmo-msc.log'")" "$C_COEUR"
    _split coeur "BSC | controleur de stations - allocation des canaux" \
        "$(_tail "'$O/osmo-bsc.log'")" "$C_COEUR"
    _split coeur "HLR | registre des abonnes - IMSI, cles, souscriptions" \
        "$(_tail "'$O/osmo-hlr.log'")" "$C_COEUR"
    _split coeur "MGW + STP | media MGCP/RTP et routage SS7" \
        "$(_tail "'$O/osmo-mgw.log' '$O/osmo-stp.log'")" "$C_COEUR"
}

_fenetre_voix() {
    local L="${LOG_DIR}" O="${OSMO_LOG_DIR:-/var/log/osmocom}"
    _w     voix "gapk | transcodage GSM-FR <-> PCM (actif pendant un appel)" \
        "$(_tail "'$L/gapk-auto.log'")" "$C_VOIX"
    _split voix "SIP | passerelle MNCC <-> SIP vers Asterisk" \
        "$(_tail "'$O/osmo-sip-connector.log'")" "$C_VOIX"
}

_fenetre_shell() {
    local aide=""
    local l; for l in "$@"; do aide="${aide}  ${l}\\n"; done
    _w shell "shell | depot qosmo" \
        "cd '${QEMU_TREE:-.}' && printf '\n  %s\n  %s\n${aide}\n' \
             'etat  : ./run.sh --status' 'arret : ./run.sh --stop' \
         && exec ${SHELL:-/bin/bash}"
}

_tmux_env_repropre() {
    command -v tmux >/dev/null 2>&1 || return 0
    tmux show-environment -g 2>/dev/null \
        | sed -n 's/^\(CALYPSO_[A-Za-z0-9_]*\)=.*/\1/p' \
        | while IFS= read -r _v; do
              [ -n "$_v" ] && tmux set-environment -g -u "$_v" 2>/dev/null
          done
    unset _v
}

_tmux_style() {
    local s="$TMUX_SESSION"
    tmux set-environment -t "$s" LANG   "${LANG:-C.UTF-8}"   2>/dev/null
    tmux set-environment -t "$s" LC_ALL "${LC_ALL:-C.UTF-8}" 2>/dev/null
    tmux set -t "$s" -g mouse on            2>/dev/null
    tmux set -t "$s" -g base-index 1        2>/dev/null
    tmux set -t "$s" -g pane-base-index 1   2>/dev/null
    tmux set -t "$s" -g history-limit 20000 2>/dev/null
    tmux set -t "$s" -g renumber-windows on 2>/dev/null
    tmux set -t "$s" -g bell-action none  2>/dev/null
    tmux set -t "$s" -g visual-bell off   2>/dev/null
    tmux set -t "$s" -g monitor-bell off  2>/dev/null
    tmux set -t "$s" -g status-style        "bg=colour234,fg=colour250" 2>/dev/null
    tmux set -t "$s" -g status-left-length  40 2>/dev/null
    tmux set -t "$s" -g status-left \
        "#[bg=colour31,fg=colour15,bold] calypso #[default] " 2>/dev/null
    tmux set -t "$s" -g status-right-length 76 2>/dev/null
    tmux set -t "$s" -g status-right \
        "#[fg=colour215]gr-gsm#[fg=colour244] · #[fg=colour114]pont ARFCN ${PONT_ARFCN:-?} BSIC ${PONT_BSIC:-?}#[fg=colour244] · #[fg=colour81]${ENCRYPTION:-?}#[default]  #[fg=colour250]%H:%M " 2>/dev/null
    tmux set -t "$s" -g window-status-format         " #I #W " 2>/dev/null
    tmux set -t "$s" -g window-status-current-format \
        "#[bg=colour31,fg=colour15,bold] #I #W #[default]" 2>/dev/null
    tmux set -t "$s" -g pane-border-style        "fg=colour238" 2>/dev/null
    tmux set -t "$s" -g pane-active-border-style "fg=colour31"  2>/dev/null
    tmux set -t "$s" -g pane-border-status top 2>/dev/null
    tmux set -t "$s" -g pane-border-format " #{pane_title} " 2>/dev/null
}
