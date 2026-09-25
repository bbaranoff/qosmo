: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER hlr "Cœur — OsmoHLR (registre GSUP)"
MOD_REQUIRED[hlr]=1
MOD_JOURNAL[hlr]="osmo-hlr"
MOD_TIMEOUT[hlr]=30
MOD_ENABLED_IF[hlr]='[ "${NO_OSMO_START:-0}" != 1 ]'

: "${HLR_UNIT:=osmo-hlr}"
: "${HLR_VTY_PORT:=4258}"
: "${HLR_GSUP_PORT:=4222}"
: "${HLR_DB:=/var/lib/osmocom/hlr.db}"

_hlr_cfg()     { core_cfg osmo-hlr; }
_hlr_gsup_ip() { core_cfg_field "$(_hlr_cfg)" '^[[:space:]]*bind[[:space:]]+ip[[:space:]]' 3 127.0.0.2; }

mod_hlr_check() {
    core_bin osmo-hlr >/dev/null || {
        mod_fail "binaire osmo-hlr introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(_hlr_cfg)" ] || {
        mod_hint "déployez la configuration : cp configs/osmo-hlr.cfg $OSMOCOM_CFG/"
        mod_fail "configuration illisible : $(_hlr_cfg)"; return $MOD_RC_FAIL; }
    local dbdir; dbdir="$(dirname "$HLR_DB")"
    [ -d "$dbdir" ] || {
        mod_hint "mkdir -p $dbdir && chown osmocom:osmocom $dbdir"
        mod_fail "répertoire de base absent : $dbdir"; return $MOD_RC_FAIL; }
    core_ip_local "$(_hlr_gsup_ip)" || {
        mod_hint "ip addr add $(_hlr_gsup_ip)/24 dev lo, ou changez « gsup / bind ip » dans $(_hlr_cfg)"
        mod_fail "adresse GSUP $(_hlr_gsup_ip) portée par aucune interface"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_hlr_status() { core_unit_active "$HLR_UNIT" || core_vty_listen "$HLR_VTY_PORT"; }

mod_hlr_start() {
    core_svc_start "$HLR_UNIT" "$(core_bin osmo-hlr)" -c "$(_hlr_cfg)" -l "$HLR_DB" \
        || { mod_fail "systemctl start $HLR_UNIT a échoué"
             mod_hint "journalctl -u $HLR_UNIT -n 30"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_hlr_wait() {
    local to="${MOD_TIMEOUT[hlr]}" gip; gip="$(_hlr_gsup_ip)"

    if ! wait_until "$to" "VTY OsmoHLR ($HLR_VTY_PORT)" core_vty_listen "$HLR_VTY_PORT"; then
        mod_hint "journalctl -u $HLR_UNIT -n 30"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "$to" "GSUP ($gip:$HLR_GSUP_PORT)" core_tcp_listen "$HLR_GSUP_PORT" "$gip"; then
        mod_hint "base verrouillée ou illisible ? ls -l $HLR_DB ; journalctl -u $HLR_UNIT -n 30"
        return $MOD_RC_FAIL
    fi
    if core_restarted_since "$HLR_UNIT"; then
        mod_hint "journalctl -u $HLR_UNIT -n 50 : le service redémarre en boucle"
        mod_fail "OsmoHLR a redémarré depuis le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_hlr_stop() { core_svc_stop "$HLR_UNIT" "osmo-hlr -c"; }
