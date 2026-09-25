: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER sgsn "Cœur — OsmoSGSN (rattachement paquet)"
MOD_REQUIRED[sgsn]=0
MOD_DEPS[sgsn]="hlr"
MOD_JOURNAL[sgsn]="osmo-sgsn"
MOD_TIMEOUT[sgsn]=25
MOD_ENABLED_IF[sgsn]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_GPRS:-1}" = 1 ]'

: "${SGSN_UNIT:=osmo-sgsn}"
: "${SGSN_VTY_PORT:=4245}"

_sgsn_cfg()  { core_cfg osmo-sgsn; }
_sgsn_ns_ip()   { core_cfg_field "$(_sgsn_cfg)" '^[[:space:]]*listen[[:space:]]+[0-9]' 2 ""; }
_sgsn_ns_port() { core_cfg_field "$(_sgsn_cfg)" '^[[:space:]]*listen[[:space:]]+[0-9]' 3 23000; }

mod_sgsn_check() {
    core_bin osmo-sgsn >/dev/null || { mod_fail "binaire osmo-sgsn introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(_sgsn_cfg)" ] || {
        mod_hint "déployez la configuration : cp configs/osmo-sgsn.cfg $OSMOCOM_CFG/"
        mod_fail "configuration illisible : $(_sgsn_cfg)"; return $MOD_RC_FAIL; }
    local ip; ip="$(_sgsn_ns_ip)"
    if [ -n "$ip" ] && ! core_ip_local "$ip"; then
        mod_hint "corrigez « ns / bind udp local / listen » dans $(_sgsn_cfg)"
        mod_fail "adresse NS $ip portée par aucune interface"; return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_sgsn_status() { core_unit_active "$SGSN_UNIT" || core_vty_listen "$SGSN_VTY_PORT"; }

mod_sgsn_start() {
    core_svc_start "$SGSN_UNIT" "$(core_bin osmo-sgsn)" -c "$(_sgsn_cfg)" \
        || { mod_fail "systemctl start $SGSN_UNIT a échoué"
             mod_hint "journalctl -u $SGSN_UNIT -n 30"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sgsn_wait() {
    local to="${MOD_TIMEOUT[sgsn]}" ip port
    ip="$(_sgsn_ns_ip)"; port="$(_sgsn_ns_port)"

    if ! wait_until "$to" "VTY OsmoSGSN ($SGSN_VTY_PORT)" core_vty_listen "$SGSN_VTY_PORT"; then
        mod_hint "journalctl -u $SGSN_UNIT -n 30"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "$to" "NS UDP ($ip:$port)" core_udp_listen "$port" "$ip"; then
        mod_hint "adresse $ip absente ou port $port déjà pris : ss -lun | grep $port"
        return $MOD_RC_FAIL
    fi
    if core_restarted_since "$SGSN_UNIT"; then
        mod_hint "journalctl -u $SGSN_UNIT -n 50 : le service redémarre en boucle"
        mod_fail "OsmoSGSN a redémarré depuis le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_sgsn_stop() { core_svc_stop "$SGSN_UNIT" "osmo-sgsn -c"; }
