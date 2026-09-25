MOD_REGISTER reseau "Interfaces et plan d'adressage"
MOD_REQUIRED[reseau]=1
MOD_DEPS[reseau]="rundir"
MOD_TIMEOUT[reseau]=15

: "${RESEAU_PORTS_LIBRES:=${PORT_GSMTAP_SI:-4730} ${PORT_GSMTAP_SCH:-4731}}"
: "${NEED_TUN:=0}"

_net_tenant() {
    local port="$1" out=""
    if command -v ss >/dev/null 2>&1; then
        out="$(ss -lntp "sport = :$port" 2>/dev/null | awk 'NR>1{print $0}')"
    fi
    [ -n "$out" ] && printf '%s' "$out" || printf 'tenant inconnu (ss absent ou sans droits)'
}
_net_lo_up() { ip link show lo 2>/dev/null | grep -q 'state UNKNOWN\|state UP\|<.*UP'; }
_net_route_loopback() { ip route get 127.0.0.2 >/dev/null 2>&1; }
_net_port_occupe() {
    if command -v ss >/dev/null 2>&1; then
        ss -ln 2>/dev/null | grep -qE "[:.]${1}[[:space:]]" && return 0
        return 1
    fi
    have_port "$1"
}
_net_ports_libres() {
    local p
    for p in $RESEAU_PORTS_LIBRES; do
        _net_port_occupe "$p" && return 1
    done
    return 0
}
_net_tun_ok() { [ -c /dev/net/tun ]; }

mod_reseau_check() {
    command -v ip >/dev/null 2>&1 || {
        mod_hint "apt-get install -y iproute2"
        mod_fail "commande « ip » absente : le réseau ne peut pas être constaté"
        return $MOD_RC_FAIL; }
    local p occupes=""
    for p in $RESEAU_PORTS_LIBRES; do
        _net_port_occupe "$p" && occupes="$occupes $p"
    done
    if [ -n "$occupes" ]; then
        for p in $occupes; do mod_say "port $p occupé : $(_net_tenant "$p")"; done
        mod_hint "arrêtez le run précédent :  ./run.sh --stop   (le module teardown balaie au démarrage)"
        mod_fail "ports déjà occupés :$occupes — le plan ne pourra pas les ouvrir"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_reseau_status() {
    _net_lo_up || return $MOD_RC_FAIL
    _net_route_loopback || return $MOD_RC_FAIL
    if [ "${NEED_TUN}" = 1 ]; then _net_tun_ok || return $MOD_RC_FAIL; fi
    return $MOD_RC_OK
}

mod_reseau_start() {
    _net_lo_up || ip link set lo up 2>/dev/null
    if ! _net_tun_ok; then
        if [ "$(id -u)" != 0 ]; then
            mod_say "/dev/net/tun absent et nous ne sommes pas root : création impossible"
        else
            modprobe tun 2>/dev/null
            if ! _net_tun_ok; then
                mkdir -p /dev/net 2>/dev/null
                mknod /dev/net/tun c 10 200 2>/dev/null && chmod 0666 /dev/net/tun 2>/dev/null
            fi
        fi
        if _net_tun_ok; then
            mod_say "/dev/net/tun : créé"
        elif [ "${NEED_TUN}" = 1 ]; then
            mod_hint "relancez le conteneur avec --device /dev/net/tun (ou --privileged), ou posez NEED_TUN=0"
            mod_fail "/dev/net/tun indisponible alors que NEED_TUN=1"
            return $MOD_RC_FAIL
        else
            mod_say "/dev/net/tun indisponible — sans effet (NEED_TUN=0)"
        fi
    fi
    mod_say "lo        : $(ip -o -4 addr show dev lo 2>/dev/null | awk '{print $4}' | tr '\n' ' ')"
    mod_say "ports     : $RESEAU_PORTS_LIBRES (attendus libres)"
    mod_ok
}

mod_reseau_wait() {
    wait_until "${MOD_TIMEOUT[reseau]}" "interface lo active" _net_lo_up || {
        mod_hint "ip link set lo up"
        mod_fail "la boucle locale n'est pas active"
        return $MOD_RC_FAIL; }
    wait_until "${MOD_TIMEOUT[reseau]}" "route vers 127.0.0.2" _net_route_loopback || {
        mod_hint "la pile écrit en dur sur 127.0.0.2 (HLR, contrôle BTS) : vérifiez  ip route get 127.0.0.2"
        mod_fail "127.0.0.2 n'est pas routable"
        return $MOD_RC_FAIL; }
    wait_until "${MOD_TIMEOUT[reseau]}" "libération des ports" _net_ports_libres || {
        mod_hint "un processus tient encore un port :  ss -lnp | grep -E '$(printf '%s' "$RESEAU_PORTS_LIBRES" | tr ' ' '|')'"
        mod_fail "ports toujours occupés parmi : $RESEAU_PORTS_LIBRES"
        return $MOD_RC_FAIL; }
    if [ "${NEED_TUN}" = 1 ]; then
        wait_until "${MOD_TIMEOUT[reseau]}" "périphérique TUN" _net_tun_ok || {
            mod_hint "--device /dev/net/tun au lancement du conteneur"
            mod_fail "/dev/net/tun absent (NEED_TUN=1)"
            return $MOD_RC_FAIL; }
    fi
    mod_ok
}

mod_reseau_stop() { return 0; }
