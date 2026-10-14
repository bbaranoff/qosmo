: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"
. "$MODDIR/_lib/radio.sh"
. "$MODDIR/_lib/modem.sh"

MOD_REGISTER sidecar-pcu "Second OsmoPCU (BTS#1, GPRS du modem)"
MOD_REQUIRED[sidecar-pcu]=0
MOD_DEPS[sidecar-pcu]="sgsn sidecar-bts"
MOD_TIMEOUT[sidecar-pcu]=30
MOD_ENABLED_IF[sidecar-pcu]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_GPRS:-1}" = 1 ] && [ "${MODEM_GPRS:-1}" = 1 ]'

# Un osmo-pcu ne sert qu'UN socket BTS. Le premier (18-pcu.sh) est sur
# /tmp/pcu_bts, celui du BTS principal ; celui-ci prend /tmp/pcu_bts2, le socket
# que le BTS#1 declare (osmo-bts-trx-bts1.cfg). Sa config est derivee de celle
# du premier : meme politique, autre socket, autre journal, VTY sur 127.0.0.2.
_scpcu_gen_cfg() {
    local src; src="$(core_cfg osmo-pcu)"
    [ -r "$src" ] || return 1
    {
        sed -e "s|^log file .*|log file /var/log/osmocom/${SC_PCU_UNIT}.log|" \
            -e "/^ *pcu-socket /d" "$src"
        printf ' pcu-socket %s\n' "$SC_PCU_SOCKET"
        printf 'line vty\n no login\n bind %s\n' "$SC_PCU_VTY_HOST"
    } > "$SC_PCU_CFG.tmp" && mv -f "$SC_PCU_CFG.tmp" "$SC_PCU_CFG"
}

mod_sidecar_pcu_check() {
    core_bin osmo-pcu >/dev/null || { mod_fail "binaire osmo-pcu introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(core_cfg osmo-pcu)" ] || { mod_fail "configuration du premier PCU illisible : $(core_cfg osmo-pcu)"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_pcu_status() { core_alive "$SC_PCU_UNIT"; }

mod_sidecar_pcu_start() {
    _scpcu_gen_cfg || { mod_fail "generation de $SC_PCU_CFG impossible"; return $MOD_RC_FAIL; }
    # Le bloc « pcu » doit finir par pcu-socket : sed l'a retire s'il y etait,
    # et on l'a reecrit en fin du bloc pcu (dernier noeud du fichier source).
    grep -q "^ pcu-socket $SC_PCU_SOCKET" "$SC_PCU_CFG" || {
        mod_fail "pcu-socket absent de $SC_PCU_CFG"; return $MOD_RC_FAIL; }
    mod_say "osmo-pcu -c $SC_PCU_CFG (socket $SC_PCU_SOCKET, VTY $SC_PCU_VTY_HOST:$SC_PCU_VTY_PORT, nsei $SC_GPRS_NSEI)"
    core_svc_start "$SC_PCU_UNIT" "$(core_bin osmo-pcu)" -c "$SC_PCU_CFG" \
        || { mod_fail "lancement de $SC_PCU_UNIT impossible"; return $MOD_RC_FAIL; }
    [ -e "$SC_PCU_SOCKET" ] && chmod 777 "$SC_PCU_SOCKET" 2>/dev/null
    mod_ok
}

mod_sidecar_pcu_wait() {
    wait_until 5 "processus $SC_PCU_UNIT" core_alive "$SC_PCU_UNIT" || {
        modb_tail "$LOG_DIR/$SC_PCU_UNIT.log" 20
        mod_fail "le second osmo-pcu s'est arrete aussitot lance"; return $MOD_RC_FAIL; }
    wait_until "${MOD_TIMEOUT[sidecar-pcu]}" "liaison PCU <-> BTS#1 ($SC_PCU_SOCKET)" \
        grep -aq "PCU socket $SC_PCU_SOCKET has been connected" "/var/log/osmocom/${SC_PCU_UNIT}.log" || {
        modb_tail "/var/log/osmocom/${SC_PCU_UNIT}.log" 15
        mod_hint "le BTS#1 cree $SC_PCU_SOCKET (osmo-bts-trx-bts1.cfg : pcu-socket) ; bts 1 en « gprs mode egprs » ?"
        mod_fail "le PCU ne s'est pas raccorde au BTS#1"; return $MOD_RC_FAIL; }
    mod_say "pret - Gb : nsei $SC_GPRS_NSEI, bvci $SC_GPRS_BVCI, udp $SC_GPRS_NS_PORT -> $SC_SGSN_IP:$SC_SGSN_PORT"
    mod_ok
}

# Par pid seulement : « osmo-pcu -c » en motif tuerait aussi le premier PCU.
mod_sidecar_pcu_stop() { core_svc_stop "$SC_PCU_UNIT" "osmo-pcu -c $SC_PCU_CFG"; }
