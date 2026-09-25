MOD_REGISTER gsmtap "Capture GSMTAP (pcap)"
MOD_REQUIRED[gsmtap]=0
MOD_DEPS[gsmtap]="l2"
MOD_TIMEOUT[gsmtap]=15

: "${PORT_GSMTAP:=4729}"
: "${CALYPSO_GSMTAP_IFACE:=any}"
: "${CAPTURE_DIR:=${RUN_DIR:-/tmp/calypso}/captures}"

_gsmtap_pat()  { printf '%s' "tcpdump .*udp port ${PORT_GSMTAP}"; }
_gsmtap_file() { cat "${RUN_DIR}/gsmtap.pcap.path" 2>/dev/null; }

mod_gsmtap_check() {
    command -v tcpdump >/dev/null 2>&1 || {
        mod_hint "installez-le vous-même : apt-get install tcpdump — ou lancez avec --skip gsmtap"
        mod_fail "tcpdump absent : aucune capture GSMTAP possible"
        return $MOD_RC_FAIL; }
    local d="$CAPTURE_DIR"
    [ -d "$d" ] || d="$(dirname "$CAPTURE_DIR")"
    [ -w "$d" ] || {
        mod_hint "posez CAPTURE_DIR vers un répertoire inscriptible"
        mod_fail "répertoire de capture non inscriptible : $d"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gsmtap_status() { have_proc "$(_gsmtap_pat)"; }

mod_gsmtap_start() {
    local pcap log
    pcap="$CAPTURE_DIR/mobile-gsmtap-$(date +%Y%m%d_%H%M%S).pcap"
    log="${LOG_DIR}/gsmtap.log"
    mkdir -p "$LOG_DIR" "$RUN_DIR" "$CAPTURE_DIR" 2>/dev/null
    printf '%s\n' "$pcap" > "${RUN_DIR}/gsmtap.pcap.path"
    mod_say "capture  : $pcap"
    mod_say "filtre   : udp port $PORT_GSMTAP sur $CALYPSO_GSMTAP_IFACE"
    setsid tcpdump -i "$CALYPSO_GSMTAP_IFACE" -U \
            -w "$pcap" "udp port $PORT_GSMTAP" >>"$log" 2>&1 </dev/null &
    printf '%s\n' "$!" > "${RUN_DIR}/gsmtap.pid"
    mod_ok
}

mod_gsmtap_wait() {
    local pid pcap; pid="$(cat "${RUN_DIR}/gsmtap.pid" 2>/dev/null || echo 0)"
    pcap="$(_gsmtap_file)"
    wait_until "${MOD_TIMEOUT[gsmtap]}" "création du pcap" test -s "$pcap" || {
        mod_hint "lisez ${LOG_DIR}/gsmtap.log ; « permission denied » = capture sans droits (CAP_NET_RAW)"
        mod_fail "tcpdump n'a pas créé $pcap"
        return $MOD_RC_FAIL; }
    kill -0 "$pid" 2>/dev/null || {
        mod_fail "tcpdump a démarré puis s'est arrêté"
        mod_hint "lisez ${LOG_DIR}/gsmtap.log"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gsmtap_stop() {
    local pid; pid="$(cat "${RUN_DIR}/gsmtap.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "$(_gsmtap_pat)" 2>/dev/null
    rm -f "${RUN_DIR}/gsmtap.pid"
    return 0
}
