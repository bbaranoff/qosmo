# 38-bruit.sh - noise injector on the DOWNLINK direction. OFF BY DEFAULT.
#
# Inserts tools/injecteur_bruit.py between the bridge and the layer 1 of THIS
# plan's QEMU, when the operator sets BRUIT_MODE (or BRUIT_SNR_DB alone).
# Numbered 38: before 40-qemu, because this module puts into the environment
# variables that QEMU must inherit (CALYPSO_BSP_PORT, CALYPSO_BSP_SNR_DB).
#
#   BRUIT_MODE     ber | souple | iq | relais (empty = module disabled)
#                    ber     flipped bits, rate BRUIT_BER (BRUIT_RAFALES=L:
#                            in bursts of L bits, same average rate)
#                    souple  AWGN at BRUIT_SNR_DB then hard decision (the stream
#                            has no soft values: hard bits or L2 bytes)
#                    iq      EXPERIMENTAL: bits -> GMSK I/Q + AWGN (BRUIT_SNR_DB)
#                    relais  nothing altered: checks the insertion alone
#   BRUIT_BER      flip rate, 0..1                   (ber mode)
#   BRUIT_RAFALES  average burst length, bits (ber mode, optional)
#   BRUIT_SNR_DB   SNR in dB, P/(2 sigma^2) per sample (souple, iq)
#                  SET WITHOUT BRUIT_MODE: no injector; the module exports
#                  CALYPSO_BSP_SNR_DB (AWGN of the C54x core, calypso_c54x.c
#                  c54x_bsp_load), which only acts on a QEMU built with
#                  --enable-l1-dsp (the external DSP, c54x_exe, gets it from
#                  c54x_exe/run.sh).
#   BRUIT_PERTE    probability of erasing a burst (dsp) / a block (grgsm)
#   BRUIT_TN       timeslots to degrade, e.g. 0 or 1,2 (default all)
#   BRUIT_GRAINE   seed: same seed + same frames = same errors; without
#                  BRUIT_MODE, becomes CALYPSO_BSP_BRUIT_GRAINE
#   BRUIT_STATS    statistics period in bruit.log (10 s)
#   BRUIT_OPTS     extra options passed as is
#   BRUIT_CIBLE    auto (default) | dsp | grgsm: QEMU layer 1; auto reads
#                  build/meson-info (l1_dsp / l1_grgsm)
#   BRUIT_PORT_GRGSM  port the injector listens on in grgsm (14730)
#   BRUIT_PORT_DSP    port the BSP is moved to in dsp (16702)
#   BRUIT_PY       path of the injector (default <QEMU_TREE>/tools/injecteur_bruit.py)
#
# WHERE IT IS INSERTED
#   gr-gsm L1 (the build of this bench): QEMU listens for GSMTAP on
#     127.0.0.1:4730, hard-coded (calypso_l1_grgsm.c). The bridge is what
#     changes target: pont/pont.py aims at PONT_GSMTAP_PORT, which it sets
#     itself to BRUIT_PORT_GRGSM (14730) when BRUIT_MODE is ber or relais. The
#     bridge is started by start-native.sh, not by this plan: BRUIT_MODE must
#     therefore be in the environment of start-native.sh
#     (BRUIT_MODE=ber BRUIT_BER=0.01 ./start-native.sh --grgsm).
#     Modes ber and relais only (L2 blocks already decoded, cf. injector).
#   DSP L1 inside QEMU (--enable-l1-dsp): the QEMU BSP listens on BRUIT_PORT_DSP
#     (CALYPSO_BSP_PORT, exported here before 40-qemu), the injector takes
#     127.0.0.1:6702 and receives there what the bridge sends to --dsp-port 6702.
#   DSP setup of start-native.sh (--dsp): qemu is removed from this plan, the
#     module skips itself; c54x_exe/run.sh inserts the injector.
# Injector log: $LOG_DIR/bruit.log. Details:
# osmo-operator/wiki/Injecteur-bruit.md.
MOD_REGISTER bruit "Injecteur de bruit (sens descendant)"
MOD_REQUIRED[bruit]=0
MOD_DEPS[bruit]="logs"
MOD_TIMEOUT[bruit]=10
MOD_ENABLED_IF[bruit]='[ -n "${BRUIT_MODE:-}${BRUIT_SNR_DB:-}" ]'

_TM[fr:bruit]="Injecteur de bruit (sens descendant)"
_TM[en:bruit]="Noise injector (downlink)"

: "${BRUIT_PY:=${QEMU_TREE:-/opt/GSM/qosmo}/tools/injecteur_bruit.py}"
: "${BRUIT_PORT_DSP:=16702}"
: "${BRUIT_PORT_GRGSM:=14730}"
: "${BRUIT_STATS:=10}"

_bruit_log() { printf '%s/bruit.log' "${LOG_DIR:-/tmp/calypso/logs}"; }
_bruit_pidf() { printf '%s/bruit.pid' "${RUN_DIR:-/tmp/calypso}"; }

# The layer 1 of this plan's QEMU: BRUIT_CIBLE, otherwise the build options.
_bruit_l1() {
    case "${BRUIT_CIBLE:-auto}" in dsp|grgsm) printf '%s' "$BRUIT_CIBLE"; return 0 ;; esac
    local j
    j="$(dirname "${QEMU_BIN:-/nonexistent}")/meson-info/intro-buildoptions.json"
    python3 -c 'import json, sys
d = {o["name"]: o["value"] for o in json.load(open(sys.argv[1]))}
print("dsp" if d.get("l1_dsp") == "enabled" else "grgsm", end="")' "$j" 2>/dev/null || printf 'grgsm'
}

# Is qemu in the plan being run? (start-native.sh --dsp removes it: --skip qemu,...)
_bruit_qemu_au_plan() {
    local m
    for m in "${selected[@]:-}"; do [ "$m" = qemu ] && return 0; done
    return 1
}

_bruit_args() {
    BRUIT_ARGS=(--mode "$BRUIT_MODE" --etiquette qosmo --stats "$BRUIT_STATS")
    if [ "$(_bruit_l1)" = dsp ]; then
        BRUIT_ARGS+=(--cible dsp --ecoute 127.0.0.1:6702 --vers "127.0.0.1:$BRUIT_PORT_DSP")
    else
        BRUIT_ARGS+=(--cible grgsm --ecoute "127.0.0.1:${PONT_GSMTAP_PORT:-$BRUIT_PORT_GRGSM}"
                     --vers "127.0.0.1:${PORT_GSMTAP_SI:-4730}")
    fi
    [ -n "${BRUIT_BER:-}" ]     && BRUIT_ARGS+=(--ber "$BRUIT_BER")
    [ -n "${BRUIT_RAFALES:-}" ] && BRUIT_ARGS+=(--rafales "$BRUIT_RAFALES")
    [ -n "${BRUIT_SNR_DB:-}" ]  && BRUIT_ARGS+=(--snr-db "$BRUIT_SNR_DB")
    [ -n "${BRUIT_PERTE:-}" ]   && BRUIT_ARGS+=(--perte "$BRUIT_PERTE")
    [ -n "${BRUIT_TN:-}" ]      && BRUIT_ARGS+=(--tn "$BRUIT_TN")
    [ -n "${BRUIT_GRAINE:-}" ]  && BRUIT_ARGS+=(--graine "$BRUIT_GRAINE")
    # shellcheck disable=SC2206 # BRUIT_OPTS: word splitting intended
    [ -n "${BRUIT_OPTS:-}" ]    && BRUIT_ARGS+=($BRUIT_OPTS)
    return 0
}

_bruit_vivant() {
    [ -n "${BRUIT_MODE:-}" ] || return 1
    local pid; pid="$(cat "$(_bruit_pidf)" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill -0 "$pid" 2>/dev/null \
        && tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q 'injecteur_bruit\.py'
}

mod_bruit_check() {
    if ! _bruit_qemu_au_plan; then
        mod_skip "qemu hors du plan (montage DSP de start-native.sh) : c54x_exe/run.sh intercale l'injecteur"
        return $MOD_RC_SKIP
    fi
    [ -n "${BRUIT_MODE:-}" ] || { mod_ok; return $MOD_RC_OK; }      # BRUIT_SNR_DB alone: nothing to start
    command -v python3 >/dev/null 2>&1 || { mod_fail "python3 introuvable"; return $MOD_RC_FAIL; }
    [ -r "$BRUIT_PY" ] || {
        mod_hint "posez BRUIT_PY=<chemin de injecteur_bruit.py>"
        mod_fail "injecteur introuvable : $BRUIT_PY"; return $MOD_RC_FAIL; }
    local l1; l1="$(_bruit_l1)"
    if [ "$l1" = grgsm ]; then
        case "$BRUIT_MODE" in
            ber|relais) ;;
            *) mod_hint "en gr-gsm, QEMU recoit des blocs L2 deja decodes : BRUIT_MODE=ber (ou relais), BRUIT_PERTE pour des blocs perdus"
               mod_fail "BRUIT_MODE=$BRUIT_MODE impossible avec la L1 gr-gsm (le pont n'est pas redirige, cellule intacte)"
               return $MOD_RC_FAIL ;;
        esac
        if [ "${PONT_GSMTAP_PORT:-$BRUIT_PORT_GRGSM}" = "${PORT_GSMTAP_SI:-4730}" ]; then
            mod_hint "laissez PONT_GSMTAP_PORT vide (14730 par defaut) ou posez BRUIT_PORT_GRGSM"
            mod_fail "l'injecteur ecouterait le port de QEMU (${PORT_GSMTAP_SI:-4730})"
            return $MOD_RC_FAIL
        fi
    fi
    _bruit_args
    mod_say "verification : python3 $BRUIT_PY ${BRUIT_ARGS[*]} --verifier"
    local out
    if ! out="$(python3 "$BRUIT_PY" "${BRUIT_ARGS[@]}" --verifier 2>&1)"; then
        mod_say "$out"
        mod_fail "$(printf '%s\n' "$out" | tail -1)"
        return $MOD_RC_FAIL
    fi
    mod_say "$out"
    mod_ok
}

mod_bruit_status() { _bruit_vivant; }

mod_bruit_start() {
    local l1; l1="$(_bruit_l1)"
    if [ -z "${BRUIT_MODE:-}" ]; then
        # BRUIT_SNR_DB alone: the "physical" noise of the C54x core, in C.
        export CALYPSO_BSP_SNR_DB="${CALYPSO_BSP_SNR_DB:-$BRUIT_SNR_DB}"
        [ -n "${BRUIT_GRAINE:-}" ] && export CALYPSO_BSP_BRUIT_GRAINE="${CALYPSO_BSP_BRUIT_GRAINE:-$BRUIT_GRAINE}"
        if [ "$l1" = dsp ]; then
            mod_say "CALYPSO_BSP_SNR_DB=$CALYPSO_BSP_SNR_DB exporte pour QEMU (L1 DSP) : AWGN sur tous les I/Q descendants"
        else
            mod_say "CALYPSO_BSP_SNR_DB=$CALYPSO_BSP_SNR_DB exporte, mais la L1 de ce QEMU est gr-gsm : SANS EFFET"
            mod_say "  (bruit physique = coeur C54x : montage DSP, c54x_exe/run.sh ; ici : BRUIT_MODE=ber)"
        fi
        mod_ok; return $MOD_RC_OK
    fi
    local log; log="$(_bruit_log)"
    mkdir -p "$(dirname "$log")" "$(dirname "$(_bruit_pidf)")" 2>/dev/null
    : > "$log"
    _bruit_args
    if [ "$l1" = dsp ]; then
        export CALYPSO_BSP_PORT="$BRUIT_PORT_DSP"
        mod_say "CALYPSO_BSP_PORT=$CALYPSO_BSP_PORT exporte : le BSP de QEMU y ecoutera, l'injecteur prend 6702"
    else
        mod_say "L1 gr-gsm : pont -> 127.0.0.1:${PONT_GSMTAP_PORT:-$BRUIT_PORT_GRGSM} (injecteur) -> QEMU ${PORT_GSMTAP_SI:-4730}"
        mod_say "  le pont doit viser ce port : BRUIT_MODE dans l'environnement de start-native.sh (pont/pont.py)"
    fi
    mod_say "commande : python3 -u $BRUIT_PY ${BRUIT_ARGS[*]}"
    mod_say "journal  : $log"
    setsid python3 -u "$BRUIT_PY" "${BRUIT_ARGS[@]}" >>"$log" 2>&1 </dev/null &
    printf '%s\n' "$!" > "$(_bruit_pidf)"
    mod_ok
}

mod_bruit_wait() {
    [ -n "${BRUIT_MODE:-}" ] || { mod_ok; return $MOD_RC_OK; }
    local log; log="$(_bruit_log)"
    wait_until "${MOD_TIMEOUT[bruit]}" "injecteur pret" log_has "$log" "PRET ecoute=" || {
        modb_tail "$log" 15
        mod_hint "lisez $log"
        return $MOD_RC_FAIL; }
    _bruit_vivant || {
        modb_tail "$log" 15
        mod_fail "l'injecteur s'est arrete aussitot lance"
        return $MOD_RC_FAIL; }
    mod_say "$(grep -a 'PRET ecoute=' "$log" | tail -1)"
    mod_ok
}

mod_bruit_stop() {
    local pid; pid="$(cat "$(_bruit_pidf)" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f -- "injecteur_bruit\.py .*--etiquette qosmo" 2>/dev/null
    rm -f "$(_bruit_pidf)"
    return 0
}
