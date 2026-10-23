. "$(dirname "${BASH_SOURCE[0]}")/_lib/temps_reel.sh"

MOD_REGISTER bts "Station de base BTS#0 (osmo-bts-trx, pont)"
MOD_REQUIRED[bts]=0
MOD_DEPS[bts]="bsc"
MOD_TIMEOUT[bts]=30

: "${OSMO_BTS_TRX:=}"
: "${BTS_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bts-trx.cfg}"
: "${BTS_LOG:=${LOG_DIR}/bts.log}"
: "${BTS_VTY_PORT:=4241}"

mod_bts_check() {
    if [ -z "$OSMO_BTS_TRX" ] || [ ! -x "$OSMO_BTS_TRX" ]; then
        OSMO_BTS_TRX="$(command -v osmo-bts-trx 2>/dev/null)"
    fi
    [ -n "$OSMO_BTS_TRX" ] && [ -x "$OSMO_BTS_TRX" ] || {
        mod_hint "installez osmo-bts (cible trx) ou posez OSMO_BTS_TRX=<chemin>"
        mod_fail "osmo-bts-trx introuvable"
        return $MOD_RC_FAIL
    }
    [ -r "$BTS_CFG" ] || {
        mod_hint "attendu : <OSMOCOM_CFG>/osmo-bts-trx.cfg — ou posez BTS_CFG=<chemin>"
        mod_fail "configuration de la BTS illisible : $BTS_CFG"
        return $MOD_RC_FAIL
    }
    have_port "${BSC_VTY_PORT:-4242}" || mod_say "AVERTISSEMENT : le VTY du BSC (${BSC_VTY_PORT:-4242}) ne répond pas — l'OML risque de boucler"
    mod_say "binaire=$OSMO_BTS_TRX cfg=$BTS_CFG"
    mod_ok
}

_bts_est_le_notre() {
    tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -qF "$BTS_CFG"
}

mod_bts_status() {
    local pid; pid="$(cat "${RUN_DIR}/bts.pid" 2>/dev/null || echo 0)"
    if [ "$pid" != 0 ] && kill -0 "$pid" 2>/dev/null && _bts_est_le_notre "$pid"; then
        return 0
    fi
    local p
    for p in $(pgrep -x osmo-bts-trx 2>/dev/null); do
        _bts_est_le_notre "$p" && return 0
    done
    return 1
}

mod_bts_start() {
    mkdir -p "${RUN_DIR:-/tmp/calypso}" "$(dirname "$BTS_LOG")" 2>/dev/null || true
    : > "$BTS_LOG" 2>/dev/null || true
    # [2026-10-23] GARDIEN (BTS_GARDIEN=1 par defaut) : osmo-bts-trx SORT (exit 1) sur « PC clock skew too high »
    # quand l'horloge du pont, cadencee par le DSP, cale plus de 50 trames (run du 23/10 17:37, Magnetite : apres
    # une liberation SDCCH, BTS0 partie, plus aucune voie descendante, reseau perdu jusqu'a la relance du banc).
    # La boucle la relance apres BTS_GARDIEN_PAUSE s ; elle attend si une autre BTS de la meme config tourne
    # deja (c54x_exe/run.sh la relance aussi en --bts). Le pid du fichier est celui de la boucle : mod_bts_stop
    # tue la boucle d'abord, puis la BTS.
    if [ "${BTS_GARDIEN:-1}" = 1 ]; then
        setsid bash -c '
            bin="$1" cfg="$2" log="$3" pause="$4"; shift 4
            while :; do
                if pgrep -f -- "osmo-bts-trx -c $cfg\$" >/dev/null 2>&1; then sleep 2; continue; fi
                stdbuf -oL -eL "$@" "$bin" -c "$cfg" </dev/null; rc=$?
                printf "\n[gardien bts] %s : osmo-bts-trx sorti (code %s), relance dans %s s\n" "$(date +%T)" "$rc" "$pause"
                sleep "$pause"
            done' gardien-bts "$OSMO_BTS_TRX" "$BTS_CFG" "$BTS_LOG" "${BTS_GARDIEN_PAUSE:-3}" $(rt_prefixe) \
            >>"$BTS_LOG" 2>&1 </dev/null &
    else
        setsid stdbuf -oL -eL $(rt_prefixe) "$OSMO_BTS_TRX" -c "$BTS_CFG" >>"$BTS_LOG" 2>&1 </dev/null &
    fi
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/bts.pid"
    mod_ok
}

mod_bts_wait() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/bts.pid" 2>/dev/null || echo 0)"
    if ! wait_until "${MOD_TIMEOUT[bts]}" "VTY de la BTS ($BTS_VTY_PORT)" have_port "$BTS_VTY_PORT"; then
        modb_tail "$BTS_LOG" 25
        if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
            mod_hint "causes fréquentes : phy/instance absente de $BTS_CFG, ou unit-id déjà pris par une autre BTS"
            mod_fail "osmo-bts-trx s'est arrêté pendant son initialisation"
        else
            mod_hint "détail : $BTS_LOG"
            mod_fail "osmo-bts-trx tourne mais n'expose pas son VTY après ${MOD_TIMEOUT[bts]}s"
        fi
        return $MOD_RC_FAIL
    fi
    if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
        modb_tail "$BTS_LOG" 25
        mod_hint "une BTS résiduelle (unité systemd ?) tient le VTY : systemctl stop osmo-bts-trx"
        mod_fail "le VTY $BTS_VTY_PORT répond, mais ce n'est PAS notre processus (il est mort)"
        return $MOD_RC_FAIL
    fi
    if grep -qi 'rsl' "$BTS_LOG" 2>/dev/null; then
        mod_say "RSL mentionné dans $BTS_LOG (lien vers le BSC en cours d'établissement)"
    else
        mod_say "aucune mention de RSL dans $BTS_LOG pour l'instant — surveillez l'OML côté BSC"
    fi
    mod_say "ordonnancement : $(rt_etat "$pid")"
    mod_ok
}

mod_bts_stop() {
    local pidf="${RUN_DIR:-/tmp/calypso}/bts.pid" pid _p
    pid="$(cat "$pidf" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    for _p in $(pgrep -f '^bash -c .*gardien-bts' 2>/dev/null) $(pgrep -f "gardien-bts .*$BTS_CFG" 2>/dev/null); do
        _bts_est_le_notre "$_p" && kill "$_p" 2>/dev/null
    done
    for _p in $(pgrep -x osmo-bts-trx 2>/dev/null); do
        _bts_est_le_notre "$_p" && kill "$_p" 2>/dev/null
    done
    rm -f "$pidf"
    return 0
}
