MOD_REGISTER config "Résolution des chemins et des ports"
MOD_REQUIRED[config]=1
MOD_DEPS[config]="prereqs"
MOD_PURE[config]=1
MOD_TIMEOUT[config]=5

mod_config_check() {
    if [ ! -d "${QEMU_TREE:-}/run_modules" ]; then
        mod_hint "posez QEMU_TREE sur la racine du dépôt, ou lancez ./run.sh depuis cette racine"
        mod_fail "QEMU_TREE ne désigne pas ce dépôt : ${QEMU_TREE:-<non défini>}"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_config_status() { return $MOD_RC_FAIL; }

mod_config_start() {
    : "${BTS_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bts-trx.cfg}"
    : "${MOBILE_CFG_SRC:=${QEMU_CFGS:-$QEMU_TREE/cfgs}/mobile_group1.cfg}"
    : "${MOBILE_CFG:=${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg}"
    : "${QEMU_MON_SOCK:=${RUN_DIR:-/tmp/calypso}/qemu-monitor.sock}"
    : "${L1CTL_SOCK_PATH:=/tmp/osmocom_l2}"
    : "${TMUX_SESSION:=calypso}"
    : "${PORT_TRXD_CLOCK:=5700}"
    : "${PORT_TRXD_CTRL:=5701}"
    : "${PORT_TRXD_DATA:=5702}"
    : "${PORT_GSMTAP:=4729}"
    : "${PORT_GSMTAP_SI:=4730}"
    : "${PORT_GSMTAP_SCH:=4731}"
    mod_say "BTS_CFG          = $BTS_CFG"
    mod_say "MOBILE_CFG_SRC   = $MOBILE_CFG_SRC"
    mod_say "MOBILE_CFG       = $MOBILE_CFG"
    mod_say "QEMU_MON_SOCK    = $QEMU_MON_SOCK"
    mod_say "L1CTL_SOCK_PATH  = $L1CTL_SOCK_PATH (non exportée)"
    mod_say "ports            : TRXD $PORT_TRXD_CLOCK-$PORT_TRXD_DATA (pont) · GSMTAP $PORT_GSMTAP · QEMU $PORT_GSMTAP_SI/$PORT_GSMTAP_SCH"
    local f
    for f in "$BTS_CFG" "$MOBILE_CFG_SRC"; do
        [ -r "$f" ] && mod_say "lisible  : $f" || mod_say "ABSENT   : $f (le module qui l'utilise le signalera)"
    done
    mod_ok
}

_config_vars() {
    printf '%s\n' BTS_CFG MOBILE_CFG MOBILE_CFG_SRC QEMU_MON_SOCK L1CTL_SOCK_PATH TMUX_SESSION \
                  PORT_TRXD_CLOCK PORT_GSMTAP PORT_GSMTAP_SI PORT_GSMTAP_SCH
}
_config_missing() {
    local v out=""
    while read -r v; do [ -n "${!v:-}" ] || out="$out $v"; done < <(_config_vars)
    printf '%s' "${out# }"
}
_config_all_set() { [ -z "$(_config_missing)" ]; }

mod_config_wait() {
    if ! wait_until "${MOD_TIMEOUT[config]}" "variables de configuration" _config_all_set; then
        mod_hint "vérifiez environnement/bench.env (QEMU_TREE, OSMOCOM_CFG, OSMOCOM_HOME, RUN_DIR)"
        mod_fail "variables de configuration vides : $(_config_missing)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_config_stop() { return 0; }
