#!/bin/bash
set -u

_pid_qemu="$(pgrep -x qemu-system-arm 2>/dev/null | head -1)"
_live=""
if [ -n "$_pid_qemu" ] && [ -r "/proc/$_pid_qemu/environ" ]; then
    _live="$(tr '\0' '\n' < "/proc/$_pid_qemu/environ" 2>/dev/null | sed -n 's/^LOG_DIR=//p' | head -1)"
fi
LIVE_DIR="${LOG_DIR:-${_live:-/tmp/calypso/logs}}"
L="${1:-$LIVE_DIR}"
SORTIE="${2:-}"
[ -n "$SORTIE" ] && exec > >(tee "$SORTIE")

q="$L/qemu.log"; o="$L/osmocon.log"; m="$L/mobile.log"
b="$L/bts.log";  p="${PONT_LOG:-/dev/shm/pont.log}"

n() { [ -r "${2:-}" ] && grep -icE -- "$1" "$2" 2>/dev/null | head -1 || echo 0; }
temoin() {
    [ -r "${2:-}" ] || return
    grep -iE -- "$1" "$2" 2>/dev/null | tail -1 \
        | sed -E 's/\x1b\[[0-9;]*m//g; s/^[[:space:]]+//' | cut -c1-140
}
mes() {
    local lib="$1" mot="$2" fic="$3" v
    v="$(n "$mot" "$fic")"
    printf '  %-24s %7s' "$lib" "$v"
    if [ "$v" -gt 0 ] 2>/dev/null; then
        local t; t="$(temoin "$mot" "$fic")"; [ -n "$t" ] && printf '  | %s' "$t"
    else
        printf '  | motif jamais vu — absence NON prouvee'
    fi
    printf '\n'
}
titre() { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }
info()  { printf '  %-24s %s\n' "$1" "$2"; }

printf '\033[1m########## RAPPORT DE RUN — %s ##########\033[0m\n' "$(date '+%Y-%m-%d %H:%M:%S')"
[ -r "$q" ] || { echo "ERREUR : $q illisible — mauvais repertoire ?"; exit 2; }
printf 'journaux : %s%s\n' "$L" "$([ "$L" = "$LIVE_DIR" ] && echo '  (run en cours)' || echo '  (photo archivee)')"
printf 'derniere ecriture : %s\n' "$(stat -c %y "$q" 2>/dev/null | cut -d. -f1)"
if [ "$L" = "$LIVE_DIR" ] && [ -n "${_pid_qemu:-}" ]; then
    _t0="$(stat -c %s "$q" 2>/dev/null)"; sleep 2; _t1="$(stat -c %s "$q" 2>/dev/null)"
    if [ "${_t0:-0}" = "${_t1:-0}" ]; then
        printf '\033[1m  ⚠️ qemu.log NE GROSSIT PAS alors que qemu tourne (pid %s).\n' "$_pid_qemu"
        printf '     Journaux probablement perimes — verifier LOG_DIR du processus.\033[0m\n'
    fi
fi

titre "0. IDENTITE DU RUN"
info "couche 1"     "$(temoin '\[l1\] backend' "$q")"
info "pont"         "$(temoin 'pont TRX démarré|pont TRX demarre' "$p")"
if [ "$L" = "$LIVE_DIR" ]; then
    P="$(pgrep -x qemu-system-arm 2>/dev/null | head -1)"
    [ -n "$P" ] && info "qemu vivant" "pid=$P age=$(ps -o etime= -p "$P" 2>/dev/null | tr -d ' ')" \
                || info "qemu vivant" "ARRETE"
    P="$(pgrep -f 'pont/pont.py' 2>/dev/null | head -1)"
    [ -n "$P" ] && info "pont vivant" "pid=$P age=$(ps -o etime= -p "$P" 2>/dev/null | tr -d ' ')" \
                || info "pont vivant" "ARRETE"
fi

titre "1. ACQUISITION L1 (osmocon.log)"
mes "L1CTL_FBSB_REQ"      'L1CTL_FBSB_REQ'                 "$o"
mes "FB detectee"         '^FB[01] \('                      "$o"
mes "Synchronize_TDMA"    'Synchronize_TDMA'                "$o"
mes "SB decodee"          '^SB[0-9] \(|=> SB 0x'            "$o"
mes "  BSIC non nul"      'BSIC=[1-9]'                      "$o"
mes "RESET FULL"          'L1CTL_RESET_REQ: FULL'           "$o"
[ -r "$o" ] && info "TOA les plus vus" "$(grep -oE 'TOA= *[0-9]+' "$o" | tr -s ' ' | sort | uniq -c | sort -rn | head -5 | tr '\n' ' ')"

titre "2. COUCHE 1 gr-gsm (qemu.log)"
mes "reset L1"            '\[l1\] reset L1'                 "$q"
mes "canal dedie"         '\[l1\] canal dedie'              "$q"
mes "chiffrement actif"   '\[l1\] chiffrement A5'           "$q"
mes "canal TCH"           '\[l1\] canal TCH'                "$q"
mes "couche 1 en clair"   '\[l1\] couche 1 en clair'        "$q"

titre "3. PONT TRX (pont.log)"
mes "bursts DL"           'STATS dl_bursts='                "$p"
mes "RACH"                'rach=[1-9]'                      "$p"
mes "erreurs"             'Traceback|error'                 "$p"
[ -r "$p" ] && info "derniere STATS" "$(grep 'STATS' "$p" 2>/dev/null | tail -1 | cut -c1-140)"

titre "4. COUCHES 2/3 (mobile.log) — jusqu ou va l abonne"
mes "sync ARFCN"          'Sync to ARFCN'                   "$m"
mes "SYSTEM INFORMATION"  'SYSTEM INFORMATION [0-9]'        "$m"
mes "  sans sysinfo"      'No sysinfo yet'                  "$m"
mes "camp"                'camping \(normal\)|camped normally' "$m"
mes "CHANNEL REQUEST"     'CHANNEL REQUEST'                 "$m"
mes "IMMEDIATE ASSIGN"    'IMMEDIATE ASSIGNMENT'            "$m"
mes "LAPDm SABM"          'LAPD_STATE_SABM_SENT'            "$m"
mes "cipher negocie"      'cipher [1-9]'                    "$m"
mes "LU REQUEST"          'LOCATION UPDATING REQUEST'       "$m"
mes "LU ACCEPT"           'LOCATION UPDATING ACCEPT'        "$m"
mes "LU REJECT"           'LOCATION UPDATING REJECT'        "$m"
mes "TMSI"                'TMSI'                            "$m"
mes "SMS"                 'SMS-DELIVER|new SMS|CP-DATA'     "$m"
mes "appel"               'CC_SETUP|MNCC_SETUP'             "$m"
[ -r "$m" ] && info "timers MM"  "$(grep -oE 'T3[0-9]{3}' "$m" | sort | uniq -c | sort -rn | head -4 | tr '\n' ' ')"
[ -r "$m" ] && info "ccch mode"  "$(grep -oE 'ccch mode [A-Z]+' "$m" | sort | uniq -c | tr '\n' ' ')"

titre "5. RESEAU (bts / side-car)"
mes "BTS erreurs"         'ERROR'                           "$b"
[ -r "$b" ] && info "erreur dominante" "$(grep -oE 'send\(\) failed on TRXD[^(]*|ERROR [a-z0-9.]+:' "$b" | sort | uniq -c | sort -rn | head -2 | tr '\n' ' ')"
mes "sidecar mobile err"  'error|fail'                      "$L/sidecar-mobile.log"

titre "6. SERVICES"
mes "SMSC"                'sms|deliver'                     "$L/smsc-op1.log"
mes "asterisk warn/err"   'error|warning'                   "$L/asterisk.log"

titre "7. VERDICT — jusqu ou va la chaine"
etape() {
    if [ "${2:-0}" -gt 0 ] 2>/dev/null; then printf '  [ OK ] %-22s (%s)\n' "$1" "$2"
    else printf '  [ .. ] %-22s (0)\n' "$1"; fi
}
etape "synchro FB"        "$(n '^FB[01] \(' "$o")"
etape "synchro SB"        "$(n '^SB[0-9] \(|=> SB 0x' "$o")"
etape "sysinfo"           "$(n 'SYSTEM INFORMATION [0-9]' "$m")"
etape "camp"              "$(n 'camping \(normal\)|camped normally' "$m")"
etape "acces RACH"        "$(n 'CHANNEL REQUEST' "$m")"
etape "canal dedie"       "$(n 'IMMEDIATE ASSIGNMENT' "$m")"
etape "LU demande"        "$(n 'LOCATION UPDATING REQUEST' "$m")"
etape "LU accepte"        "$(n 'LOCATION UPDATING ACCEPT' "$m")"
etape "SMS"               "$(n 'SMS-DELIVER|new SMS|CP-DATA' "$m")"
printf '\n  Methode : un 0 dit que le MOTIF n a pas ete vu. Verifier le motif\n'
printf '  avant de conclure a une absence.\n'
