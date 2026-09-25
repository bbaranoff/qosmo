[ -n "${_RADIO_LIB_LOADED:-}" ] && return 0
_RADIO_LIB_LOADED=1

: "${SC_TRX_PORT:=5720}"
: "${SC_BB_PORT:=6720}"
: "${SC_BTS_VTY_PORT:=4250}"
: "${SC_MOBILE_VTY_PORT:=4248}"
: "${SC_UNIT_ID:=6002}"
: "${SC_ARFCN:=516}"
: "${SC_BTS_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bts-trx-bts1.cfg}"
: "${SC_MOBILE_CFG:=/root/.osmocom/bb/mobile_faketrx_bts1.cfg}"
: "${SC_L2_SOCK:=/tmp/ms2_l2}"
: "${TRX_BIND_IP:=127.0.0.1}"
: "${TRX_BTS_IP:=127.0.0.1}"
: "${TRX_BB_IP:=127.0.0.1}"
: "${RUN_DIR:=/tmp/calypso}"
: "${LOG_DIR:=$RUN_DIR/logs}"

radio_dirs() { mkdir -p "$RUN_DIR" "$LOG_DIR" 2>/dev/null || true; }
radio_log() { printf '%s/%s.log\n' "$LOG_DIR" "$1"; }
radio_pidfile() { printf '%s/%s.pid\n' "$RUN_DIR" "$1"; }
radio_save_pid() { radio_dirs; printf '%s\n' "$2" > "$(radio_pidfile "$1")"; }
radio_pid()   { cat "$(radio_pidfile "$1")" 2>/dev/null || echo 0; }
radio_alive() { local p; p="$(radio_pid "$1")"; [ "$p" != 0 ] && kill -0 "$p" 2>/dev/null; }

radio_kill() {
    local slug="$1" pattern="${2:-}" p i
    p="$(radio_pid "$slug")"
    [ "$p" != 0 ] && kill "$p" 2>/dev/null
    [ -n "$pattern" ] && pkill -f "$pattern" 2>/dev/null
    for i in 1 2 3 4 5 6 7 8 9 10; do
        if [ "$p" = 0 ] || ! kill -0 "$p" 2>/dev/null; then break; fi
        sleep 0.2
    done
    if [ "$p" != 0 ] && kill -0 "$p" 2>/dev/null; then
        kill -9 "$p" 2>/dev/null
        [ -n "$pattern" ] && pkill -9 -f "$pattern" 2>/dev/null
    fi
    rm -f "$(radio_pidfile "$slug")"
    return 0
}

radio_udp_bound() {
    ss -uan 2>/dev/null | grep -qE "[:.]$1[[:space:]]"
}

radio_cfg_val() {
    local f="$1"; shift
    [ -r "$f" ] || return 1
    awk -v k="$*" '
        {
            line = $0
            sub(/^[[:space:]]+/, "", line); sub(/[[:space:]]+$/, "", line)
            n = split(k, kk, /[[:space:]]+/)
            split(line, ff, /[[:space:]]+/)
            for (i = 1; i <= n; i++) if (ff[i] != kk[i]) next
            if (ff[n+1] != "") { print ff[n+1]; exit }
        }' "$f"
}
