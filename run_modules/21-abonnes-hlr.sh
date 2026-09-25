: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER abonnes-hlr "Cœur — abonnés provisionnés dans le HLR"
MOD_REQUIRED[abonnes-hlr]=0
MOD_DEPS[abonnes-hlr]="hlr"
MOD_TIMEOUT[abonnes-hlr]=30
MOD_ENABLED_IF[abonnes-hlr]='[ "${NO_OSMO_START:-0}" != 1 ]'

: "${HLR_VTY_PORT:=4258}"
: "${HLR_DB:=/var/lib/osmocom/hlr.db}"
: "${OPERATOR_ID:=1}"
: "${N_MS:=1}"

_abo_mcc() { printf '%s\n' "${MCC:-$(core_cfg_field "$(core_cfg osmo-msc)" '^[[:space:]]*network country code[[:space:]]' 4 001)}"; }
_abo_mnc() { printf '%s\n' "${MNC:-$(core_cfg_field "$(core_cfg osmo-msc)" '^[[:space:]]*mobile network code[[:space:]]' 4 01)}"; }

_abo_imsi() { printf '%s%s%04d%06d\n' "$(_abo_mcc)" "$(_abo_mnc)" "$OPERATOR_ID" "$1"; }
_abo_ki()   { printf '00112233445566778899aabbccdd%02x%02x\n' "$1" "$OPERATOR_ID"; }

_abo_node() {
    local n="${OSMO_WAN_NODE:-${WAN_NODE_ID:-}}"
    [ -n "$n" ] || n="$(sed -n 's/^PLAN_NODE=//p' "${OSMOCOM_CFG:-/etc/osmocom}/radio-plan.env" 2>/dev/null | tail -1)"
    case "$n" in [1-9]) ;; *) n=1 ;; esac
    printf '%s' "$n"
}
_abo_msisdn() { echo $(( $(_abo_node) * 100000 + OPERATOR_ID * 100 + $1 )); }

_abo_present() {
    local imsi="$1" n=""
    if [ -r "$HLR_DB" ] && command -v sqlite3 >/dev/null 2>&1; then
        n="$(sqlite3 -readonly "$HLR_DB" "select count(*) from subscriber where imsi='$imsi';" 2>/dev/null)"
        case "$n" in ''|*[!0-9]*) n="" ;; esac
        if [ -n "$n" ]; then [ "$n" -ge 1 ]; return $?; fi
    fi
    core_vty_ask "$HLR_VTY_PORT" "enable" "subscriber imsi $imsi show" 2>/dev/null | grep -q "IMSI: $imsi"
}

mod_abonnes_hlr_check() {
    case "$N_MS" in ''|*[!0-9]*) mod_fail "N_MS invalide : « $N_MS »"; return $MOD_RC_FAIL ;; esac
    [ "$N_MS" -ge 1 ] || { mod_skip "N_MS=$N_MS : aucun abonné à créer"; return $MOD_RC_SKIP; }
    core_vty_listen "$HLR_VTY_PORT" || {
        mod_hint "le HLR doit être prêt : ./run.sh --only hlr"
        mod_fail "VTY HLR ($HLR_VTY_PORT) injoignable"; return $MOD_RC_FAIL; }
    command -v socat >/dev/null 2>&1 || command -v telnet >/dev/null 2>&1 || {
        mod_hint "installez socat (ou telnet) : le provisionnement passe par le VTY"
        mod_fail "ni socat ni telnet — impossible d'écrire dans le HLR"; return $MOD_RC_FAIL; }
    mod_ok
}

_abo_msisdn_now() {
    local imsi="$1" v=""
    if [ -r "$HLR_DB" ] && command -v sqlite3 >/dev/null 2>&1; then
        v="$(sqlite3 -readonly "$HLR_DB" "select msisdn from subscriber where imsi='$imsi';" 2>/dev/null)"
        [ -n "$v" ] && { printf '%s' "$v"; return 0; }
    fi
    core_vty_ask "$HLR_VTY_PORT" "enable" "subscriber imsi $imsi show" 2>/dev/null \
        | sed -n 's/.*MSISDN: *\([0-9][0-9]*\).*/\1/p' | head -1
}

mod_abonnes_hlr_status() {
    local imsi; imsi="$(_abo_imsi "$N_MS")"
    _abo_present "$imsi" || return 1
    [ "$(_abo_msisdn_now "$imsi")" = "$(_abo_msisdn "$N_MS")" ]
}

mod_abonnes_hlr_start() {
    local m cmds=() imsi msisdn
    cmds+=("enable")
    for m in $(seq 1 "$N_MS"); do
        imsi="$(_abo_imsi "$m")"
        msisdn="$(_abo_msisdn "$m")"
        cmds+=("subscriber imsi $imsi create")
        cmds+=("subscriber imsi $imsi update msisdn $msisdn")
        cmds+=("subscriber imsi $imsi update aud2g comp128v1 ki $(_abo_ki "$m")")
    done
    cmds+=("exit")
    mod_say "provisionnement de $N_MS abonné(s), nœud $(_abo_node), opérateur $OPERATOR_ID, PLMN $(_abo_mcc)-$(_abo_mnc) — MSISDN $(_abo_msisdn 1)..$(_abo_msisdn "$N_MS")"
    core_vty_ask "$HLR_VTY_PORT" "${cmds[@]}" || {
        mod_hint "vérifiez le VTY : socat STDIO TCP:127.0.0.1:$HLR_VTY_PORT,crlf"
        mod_fail "le dialogue VTY avec le HLR n'a rien retourné"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_abonnes_hlr_wait() {
    local imsi; imsi="$(_abo_imsi "$N_MS")"
    if ! wait_until "${MOD_TIMEOUT[abonnes-hlr]}" "abonné $imsi présent dans le HLR" _abo_present "$imsi"; then
        mod_hint "essayez à la main : subscriber imsi $imsi show (VTY $HLR_VTY_PORT) ; base = $HLR_DB"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_abonnes_hlr_stop() { return 0; }
