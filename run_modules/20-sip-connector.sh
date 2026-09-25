: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER sip-connector "Cœur — passerelle MNCC↔SIP"
MOD_REQUIRED[sip-connector]=0
MOD_DEPS[sip-connector]="msc asterisk"
MOD_JOURNAL[sip-connector]="osmo-sip-connector"
MOD_TIMEOUT[sip-connector]=25
MOD_ENABLED_IF[sip-connector]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_VOICE:-1}" = 1 ]'

: "${SIPC_UNIT:=osmo-sip-connector}"

_sipc_cfg()  { core_cfg osmo-sip-connector; }
_sipc_ip()   { core_cfg_field "$(_sipc_cfg)" '^[[:space:]]*local[[:space:]]+[0-9]' 2 127.0.0.1; }
_sipc_port() { core_cfg_field "$(_sipc_cfg)" '^[[:space:]]*local[[:space:]]+[0-9]' 3 5060; }
_sipc_mncc() { core_cfg_field "$(_sipc_cfg)" '^[[:space:]]*socket-path[[:space:]]' 2 /tmp/msc_mncc; }

_sipc_mncc_connected() {
    local p; p="$(_sipc_mncc)"
    ss -xH 2>/dev/null | grep -F -- "$p" | grep -q 'ESTAB\|CONNECTED'
}

mod_sip_connector_check() {
    core_bin osmo-sip-connector >/dev/null || {
        mod_hint "installez osmo-sip-connector, ou désactivez la voix : CORE_VOICE=0"
        mod_fail "binaire osmo-sip-connector introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(_sipc_cfg)" ] || {
        mod_fail "configuration illisible : $(_sipc_cfg)"; return $MOD_RC_FAIL; }
    core_ip_local "$(_sipc_ip)" || {
        mod_hint "corrigez « sip / local » dans $(_sipc_cfg)"
        mod_fail "adresse SIP $(_sipc_ip) portée par aucune interface"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sip_connector_status() { core_unit_active "$SIPC_UNIT" || have_proc "osmo-sip-connector -c"; }

mod_sip_connector_start() {
    core_svc_start "$SIPC_UNIT" "$(core_bin osmo-sip-connector)" -c "$(_sipc_cfg)" \
        || { mod_fail "systemctl start $SIPC_UNIT a échoué"
             mod_hint "journalctl -u $SIPC_UNIT -n 30"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sip_connector_wait() {
    local to="${MOD_TIMEOUT[sip-connector]}" ip port
    ip="$(_sipc_ip)"; port="$(_sipc_port)"

    if ! wait_until "$to" "bind SIP ($ip:$port)" core_udp_listen "$port" "$ip"; then
        mod_hint "port $port déjà pris (Asterisk ?) : ss -lun | grep $port"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "$to" "connexion MNCC ($(_sipc_mncc))" _sipc_mncc_connected; then
        mod_hint "le MSC expose-t-il « mncc external $(_sipc_mncc) » ? ss -x | grep $(_sipc_mncc)"
        return $MOD_RC_FAIL
    fi
    if core_restarted_since "$SIPC_UNIT"; then
        mod_hint "journalctl -u $SIPC_UNIT -n 50 : le service redémarre en boucle"
        mod_fail "osmo-sip-connector a redémarré depuis le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_sip_connector_stop() { core_svc_stop "$SIPC_UNIT" "osmo-sip-connector -c"; }
