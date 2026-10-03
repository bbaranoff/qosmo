# 38-bruit.sh - injecteur de bruit sur le sens DESCENDANT. INACTIF PAR DEFAUT.
#
# [2026-10-03] Intercale tools/injecteur_bruit.py entre le pont et la couche 1
# du QEMU de CE plan, quand l'operateur pose BRUIT_MODE (ou BRUIT_SNR_DB seul).
# Numerote 38 : avant 40-qemu, parce que ce module pose dans l'environnement
# des variables que QEMU doit heriter (CALYPSO_BSP_PORT, CALYPSO_BSP_SNR_DB).
#
#   BRUIT_MODE     ber | souple | iq | relais (vide = module desactive)
#                    ber     bits inverses, taux BRUIT_BER (BRUIT_RAFALES=L :
#                            en rafales de L bits, meme taux moyen)
#                    souple  AWGN a BRUIT_SNR_DB puis decision dure (le flux
#                            n'a pas de valeurs souples : bits durs ou octets L2)
#                    iq      EXPERIMENTAL : bits -> I/Q GMSK + AWGN (BRUIT_SNR_DB)
#                    relais  rien d'altere : verifie l'intercalation seule
#   BRUIT_BER      taux d'inversion, 0..1            (mode ber)
#   BRUIT_RAFALES  longueur moyenne d'une rafale, bits (mode ber, optionnel)
#   BRUIT_SNR_DB   SNR en dB, P/(2 sigma^2) par echantillon (souple, iq)
#                  POSE SANS BRUIT_MODE : pas d'injecteur ; le module exporte
#                  CALYPSO_BSP_SNR_DB (AWGN du coeur C54x, calypso_c54x.c
#                  c54x_bsp_load), qui n'agit que sur un QEMU construit avec
#                  --enable-l1-dsp (le DSP externe, c54x_exe, le recoit de
#                  c54x_exe/run.sh).
#   BRUIT_PERTE    probabilite d'effacer un burst (dsp) / un bloc (grgsm)
#   BRUIT_TN       intervalles a degrader, ex. 0 ou 1,2 (defaut tous)
#   BRUIT_GRAINE   graine : meme graine + memes trames = memes erreurs ; sans
#                  BRUIT_MODE, devient CALYPSO_BSP_BRUIT_GRAINE
#   BRUIT_STATS    periode des statistiques dans bruit.log (10 s)
#   BRUIT_OPTS     options supplementaires passees telles quelles
#   BRUIT_CIBLE    auto (defaut) | dsp | grgsm : couche 1 du QEMU ; auto lit
#                  build/meson-info (l1_dsp / l1_grgsm)
#   BRUIT_PORT_GRGSM  port ou l'injecteur ecoute en grgsm (14730)
#   BRUIT_PORT_DSP    port du BSP deplace en dsp (16702)
#   BRUIT_PY       chemin de l'injecteur (defaut <QEMU_TREE>/tools/injecteur_bruit.py)
#
# OU IL S'INTERCALE
#   L1 gr-gsm (le build de ce banc) : QEMU ecoute GSMTAP sur 127.0.0.1:4730 en
#     dur (calypso_l1_grgsm.c). C'est le pont qui change de cible : pont/pont.py
#     vise PONT_GSMTAP_PORT, que lui-meme met a BRUIT_PORT_GRGSM (14730) quand
#     BRUIT_MODE vaut ber ou relais. Le pont est lance par start-direct.sh, pas
#     par ce plan : BRUIT_MODE doit donc etre dans l'environnement de
#     start-direct.sh (BRUIT_MODE=ber BRUIT_BER=0.01 ./start-direct.sh --grgsm).
#     Modes ber et relais seulement (blocs L2 deja decodes, cf. injecteur).
#   L1 DSP dans QEMU (--enable-l1-dsp) : le BSP de QEMU ecoute BRUIT_PORT_DSP
#     (CALYPSO_BSP_PORT, exporte ici avant 40-qemu), l'injecteur prend
#     127.0.0.1:6702 et y recoit ce que le pont envoie a --dsp-port 6702.
#   Montage DSP de start-direct.sh (--dsp) : qemu est retire de ce plan, le
#     module se saute ; c'est c54x_exe/run.sh qui intercale l'injecteur.
# Journal de l'injecteur : $LOG_DIR/bruit.log. Detail :
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

# La couche 1 du QEMU de ce plan : BRUIT_CIBLE, sinon les options du build.
_bruit_l1() {
    case "${BRUIT_CIBLE:-auto}" in dsp|grgsm) printf '%s' "$BRUIT_CIBLE"; return 0 ;; esac
    local j
    j="$(dirname "${QEMU_BIN:-/nonexistent}")/meson-info/intro-buildoptions.json"
    python3 -c 'import json, sys
d = {o["name"]: o["value"] for o in json.load(open(sys.argv[1]))}
print("dsp" if d.get("l1_dsp") == "enabled" else "grgsm", end="")' "$j" 2>/dev/null || printf 'grgsm'
}

# qemu est-il dans le plan joue ? (start-direct.sh --dsp le retire : --skip qemu,...)
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
    # shellcheck disable=SC2206 # BRUIT_OPTS : decoupage en mots voulu
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
        mod_skip "qemu hors du plan (montage DSP de start-direct.sh) : c54x_exe/run.sh intercale l'injecteur"
        return $MOD_RC_SKIP
    fi
    [ -n "${BRUIT_MODE:-}" ] || { mod_ok; return $MOD_RC_OK; }      # BRUIT_SNR_DB seul : rien a lancer
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
        # BRUIT_SNR_DB seul : le bruit « physique » du coeur C54x, en C.
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
        mod_say "  le pont doit viser ce port : BRUIT_MODE dans l'environnement de start-direct.sh (pont/pont.py)"
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
