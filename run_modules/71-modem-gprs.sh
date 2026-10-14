: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"
. "$MODDIR/_lib/radio.sh"
. "$MODDIR/_lib/modem.sh"

MOD_REGISTER modem-gprs "MS GPRS (modem osmocom-bb : attache, PDP, tun)"
MOD_REQUIRED[modem-gprs]=0
MOD_DEPS[modem-gprs]="modem-trxcon sidecar-pcu abonnes-hlr"
MOD_TIMEOUT[modem-gprs]=90
MOD_ENABLED_IF[modem-gprs]='[ "${MODEM_GPRS:-1}" = 1 ]'

: "${HLR_VTY_PORT:=4258}"
: "${OPERATOR_ID:=1}"
: "${BSC_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bsc.cfg}"

# Memes formules que 21-abonnes-hlr.sh, pour le 3e abonne (MODEM_MS_IDX).
_mg_mcc() { printf '%s\n' "${MCC:-$(core_cfg_field "$(core_cfg osmo-msc)" '^[[:space:]]*network country code[[:space:]]' 4 001)}"; }
_mg_mnc() { printf '%s\n' "${MNC:-$(core_cfg_field "$(core_cfg osmo-msc)" '^[[:space:]]*mobile network code[[:space:]]' 4 01)}"; }
_mg_imsi() { printf '%s%s%04d%06d\n' "$(_mg_mcc)" "$(_mg_mnc)" "$OPERATOR_ID" "$MODEM_MS_IDX"; }
_mg_ki()   { printf '00112233445566778899aabbccdd%02x%02x\n' "$MODEM_MS_IDX" "$OPERATOR_ID"; }
_mg_ki_sp() { _mg_ki | sed 's/../& /g; s/ $//'; }
_mg_node() {
    local n="${OSMO_WAN_NODE:-${WAN_NODE_ID:-}}"
    [ -n "$n" ] || n="$(sed -n 's/^PLAN_NODE=//p' "${OSMOCOM_CFG:-/etc/osmocom}/radio-plan.env" 2>/dev/null | tail -1)"
    case "$n" in [1-9]) ;; *) n=1 ;; esac
    printf '%s' "$n"
}
_mg_msisdn() { echo $(( $(_mg_node) * 100000 + OPERATOR_ID * 100 + MODEM_MS_IDX )); }
# L'ARFCN du BTS#1 tel que le BSC le declare (le radio-plan peut l'avoir change).
_mg_arfcn() {
    local a; a="$(awk '/^ bts 1/{f=1} f && /^ *arfcn /{print $2; exit}' "$BSC_CFG" 2>/dev/null)"
    printf '%s' "${a:-$SC_ARFCN}"
}

_mg_write_cfg() {
    cat > "$MODEM_CFG.tmp" <<EOF
! modem-gprs.cfg - MS GPRS du banc (osmocom-bb modem), ecrit par 71-modem-gprs.sh
! IMSI $(_mg_imsi) · MSISDN $(_mg_msisdn) · APN $MODEM_APN -> $MODEM_TUN · ARFCN $(_mg_arfcn)
line vty
 no login
 bind 127.0.0.1 $MODEM_VTY_PORT
log stderr
 logging filter all 1
 logging color 0
 logging timestamp 1
 logging print category 1
 logging print level 1
 logging level all notice
ms 1
 layer2-socket $MODEM_L2_SOCK
 sim test
 imei 35892500865948$MODEM_MS_IDX 0
 imei-fixed
 test-sim
  imsi $(_mg_imsi)
  ki comp128 $(_mg_ki_sp)
  no barred-access
  rplmn $(_mg_mcc) $(_mg_mnc)
 apn $MODEM_APN
  tun-device $MODEM_TUN
  type-support v4
  no shutdown
 no shutdown
EOF
    mv -f "$MODEM_CFG.tmp" "$MODEM_CFG"
}

mod_modem_gprs_check() {
    [ -x "$MODEM_BIN" ] || {
        mod_hint "compilez layer23 d'osmocom-bb (src/host/layer23/src/modem/modem), ou posez MODEM_BIN"
        mod_fail "binaire modem introuvable : $MODEM_BIN"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_modem_gprs_status() { radio_alive modem-gprs; }

mod_modem_gprs_start() {
    radio_dirs
    local log imsi; log="$(radio_log modem-gprs)"; imsi="$(_mg_imsi)"

    # 1. L'abonne dans le HLR (idempotent : « create » d'un IMSI existant est refuse sans effet).
    core_vty_ask "$HLR_VTY_PORT" "enable" \
        "subscriber imsi $imsi create" \
        "subscriber imsi $imsi update msisdn $(_mg_msisdn)" \
        "subscriber imsi $imsi update aud2g comp128v1 ki $(_mg_ki)" "exit" >/dev/null 2>&1
    # Presence : la base d'abord (comme 21-abonnes-hlr.sh), le VTY en repli -
    # le dialogue VTY peut ne rien rendre (socat absent, delais) alors que
    # l'abonne est bien la.
    _mg_present() {
        local n=""
        if command -v sqlite3 >/dev/null 2>&1; then
            n="$(sqlite3 -readonly "${HLR_DB:-/var/lib/osmocom/hlr.db}" "select count(*) from subscriber where imsi='$1';" 2>/dev/null)"
            [ "${n:-0}" -ge 1 ] 2>/dev/null && return 0
        fi
        core_vty_ask "$HLR_VTY_PORT" "enable" "subscriber imsi $1 show" "exit" 2>/dev/null | grep -q "IMSI: $1"
    }
    _mg_present "$imsi" \
        || { mod_fail "abonne $imsi absent du HLR apres provisionnement (VTY $HLR_VTY_PORT)"; return $MOD_RC_FAIL; }
    mod_say "abonne GPRS : IMSI $imsi, MSISDN $(_mg_msisdn)"

    # 2. La config du modem, reecrite a chaque lancement (elle suit le plan radio).
    _mg_write_cfg || { mod_fail "ecriture de $MODEM_CFG impossible"; return $MOD_RC_FAIL; }

    # 3. Le modem. Il cree lui-meme $MODEM_TUN (root) une fois le PDP actif.
    wait_until 30 "socket L1CTL $MODEM_L2_SOCK" have_unix "$MODEM_L2_SOCK" || {
        mod_fail "socket L1CTL absente : $MODEM_L2_SOCK"; return $MOD_RC_FAIL; }
    ip link del "$MODEM_TUN" 2>/dev/null
    mod_say "modem -c $MODEM_CFG -a $(_mg_arfcn) (VTY $MODEM_VTY_PORT, tun $MODEM_TUN)"
    mod_say "journal : $log"
    setsid stdbuf -oL -eL "$MODEM_BIN" -c "$MODEM_CFG" -s "$MODEM_L2_SOCK" -a "$(_mg_arfcn)" >>"$log" 2>&1 </dev/null &
    radio_save_pid modem-gprs $!
    mod_ok
}

mod_modem_gprs_wait() {
    local log; log="$(radio_log modem-gprs)"
    wait_until 5 "processus modem" radio_alive modem-gprs || {
        modb_tail "$log" 20
        mod_fail "le modem s'est arrete aussitot lance"; return $MOD_RC_FAIL; }
    wait_until 20 "VTY du modem ($MODEM_VTY_PORT)" have_port "$MODEM_VTY_PORT" || {
        modb_tail "$log" 20
        mod_fail "VTY $MODEM_VTY_PORT jamais en ecoute"; return $MOD_RC_FAIL; }
    # L'attache GPRS puis le PDP : visibles quand le tun porte une adresse (celle du GGSN, 176.16.32.x).
    if wait_until "${MOD_TIMEOUT[modem-gprs]}" "PDP actif ($MODEM_TUN avec adresse)" \
            sh -c "ip -4 -o addr show '$MODEM_TUN' 2>/dev/null | grep -q inet"; then
        mod_say "pret - $(ip -4 -o addr show "$MODEM_TUN" | awk '{print $4}' | head -1) sur $MODEM_TUN (APN $MODEM_APN)"
    else
        modb_tail "$log" 25
        mod_hint "attache GMM / PDP non abouti : journal du modem, puis osmo-sgsn.log (attach, PDP) et ${SC_PCU_UNIT}.log (TBF)"
        mod_say "modem lance mais pas de PDP dans le delai - la voix n'est pas concernee"
    fi
    mod_ok
}

mod_modem_gprs_stop() { radio_kill modem-gprs; ip link del "$MODEM_TUN" 2>/dev/null; return 0; }
