MOD_REGISTER audio "Pont audio (PulseAudio + GAPK)"
MOD_REQUIRED[audio]=0
MOD_TIMEOUT[audio]=25

MOD_ENABLED_IF[audio]='[ "${AUDIO:-1}" = 1 ]'

: "${EGPRS_DIR:=${NITB_ROOT}}"
: "${AUDIO_LIB:=$EGPRS_DIR/lib/audio.sh}"
: "${AUDIO_SINK:=gsm_audio}"
: "${AUDIO_TMUX_SESSION:=gapk}"

_audio_sink_present() {
    pactl list short sinks 2>/dev/null | grep -q "$AUDIO_SINK"
}
_audio_gapk_vivant() {
    pgrep -f "gapk-start.sh" >/dev/null 2>&1 \
        || tmux has-session -t "${AUDIO_TMUX:-gapk}" 2>/dev/null
}

mod_audio_check() {
    if [ ! -r "$AUDIO_LIB" ]; then
        mod_hint "la bibliothèque est l'extraction de start-direct.sh.legacy L566-722 ; elle accompagne osmo_egprs"
        mod_skip "bibliothèque audio absente ($AUDIO_LIB) : pont audio non monté"
        return $MOD_RC_SKIP
    fi
    local manquant=""
    command -v tmux      >/dev/null 2>&1 || manquant="$manquant tmux"
    command -v osmo-gapk >/dev/null 2>&1 || manquant="$manquant osmo-gapk"
    if [ -n "$manquant" ]; then
        mod_hint "installez-les pour avoir le son :$manquant — ou AUDIO=0 pour ne plus l'essayer"
        mod_skip "outillage audio absent :$manquant"
        return $MOD_RC_SKIP
    fi
    mod_ok
}

mod_audio_status() { _audio_gapk_vivant && _audio_sink_present; }

mod_audio_start() {
    local retour="$PWD" rc=0
    cd "$EGPRS_DIR" || { mod_fail "impossible d'entrer dans $EGPRS_DIR"; return $MOD_RC_FAIL; }
    HERE="$EGPRS_DIR"
    . "$AUDIO_LIB" || { cd "$retour"; mod_fail "bibliothèque illisible : $AUDIO_LIB"; return $MOD_RC_FAIL; }

    mod_say "sink visé : $AUDIO_SINK · session tmux : $AUDIO_TMUX_SESSION"
    ensure_gapk || rc=$?
    cd "$retour"
    [ "$rc" -eq 0 ] || { mod_fail "ensure_gapk a rendu $rc"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_audio_wait() {
    wait_until "${MOD_TIMEOUT[audio]}" "démon PulseAudio" pactl info || {
        mod_hint "regardez $LOG_DIR/pulse-system.log : le démon système n'a pas démarré"
        mod_fail "PulseAudio ne répond pas"
        return $MOD_RC_FAIL
    }
    wait_until "${MOD_TIMEOUT[audio]}" "sink $AUDIO_SINK" _audio_sink_present || {
        mod_hint "pactl load-module module-null-sink sink_name=$AUDIO_SINK"
        mod_fail "le sink $AUDIO_SINK n'est pas enregistré"
        return $MOD_RC_FAIL
    }
    wait_until "${MOD_TIMEOUT[audio]}" "superviseur gapk" _audio_gapk_vivant || {
        mod_hint "regardez $LOG_DIR/gapk-auto.log : le superviseur s'est arrêté aussitôt lancé"
        mod_fail "le superviseur gapk ne tourne pas"
        return $MOD_RC_FAIL
    }
    mod_ok
}

mod_audio_stop() {
    tmux kill-session -t "$AUDIO_TMUX_SESSION" 2>/dev/null
    pkill -x osmo-gapk 2>/dev/null
    return 0
}
