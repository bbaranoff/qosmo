MOD_REGISTER mobile-cfg "Configuration du mobile"
MOD_REQUIRED[mobile-cfg]=0
MOD_DEPS[mobile-cfg]="config"
MOD_TIMEOUT[mobile-cfg]=10

_mcfg_imsi() { grep -oE '^[[:space:]]*imsi [0-9]{15}' "${MOBILE_CFG:-}" 2>/dev/null | head -1 | grep -oE '[0-9]{15}'; }
_mcfg_ki()   { sed -n 's/^[[:space:]]*ki comp128 \([0-9a-fA-F ]*\)$/\1/p' "${MOBILE_CFG:-}" 2>/dev/null | head -1 | tr -d ' '; }
_mcfg_stick(){ sed -n 's/^[[:space:]]*stick \([0-9]\+\).*/\1/p' "${MOBILE_CFG:-}" 2>/dev/null | head -1; }

mod_mobile_cfg_check() {
    if [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" != "1" ]; then
        [ -r "${MOBILE_CFG:-}" ] && { mod_say "synchronisation désactivée, fichier existant conservé"; mod_ok; return $MOD_RC_OK; }
        mod_hint "CALYPSO_SYNC_MOBILE_CFG=1 pour réinstaller la version du dépôt"
        mod_fail "synchronisation désactivée et ${MOBILE_CFG:-<non défini>} absent"
        return $MOD_RC_FAIL
    fi
    if [ ! -r "${MOBILE_CFG_SRC:-}" ]; then
        mod_hint "attendu : \$QEMU_TREE/cfgs/mobile_group1.cfg — vérifiez QEMU_CFGS dans paths.env"
        mod_skip "configuration versionnée absente : ${MOBILE_CFG_SRC:-<non défini>}"
        return $MOD_RC_SKIP
    fi
    mod_ok
}

mod_mobile_cfg_status() {
    [ -r "${MOBILE_CFG:-}" ] || return 1
    [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" = "1" ] || return 0
    cmp -s "${MOBILE_CFG_SRC:-}" "$MOBILE_CFG" 2>/dev/null || return 1
    [ -z "${CALYPSO_STICK_ARFCN:-}" ] || [ "${CALYPSO_STICK_ARFCN}" = "0" ]
}

mod_mobile_cfg_start() {
    if [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" = "1" ]; then
        mkdir -p "$(dirname "$MOBILE_CFG")" 2>/dev/null
        if ! cp "$MOBILE_CFG_SRC" "$MOBILE_CFG" 2>/dev/null; then
            mod_hint "vérifiez les droits sur $(dirname "$MOBILE_CFG")"
            mod_fail "copie impossible : $MOBILE_CFG_SRC -> $MOBILE_CFG"
            return $MOD_RC_FAIL
        fi
        mod_say "installée : $MOBILE_CFG_SRC -> $MOBILE_CFG"
    fi

    local a="${CALYPSO_STICK_ARFCN:-}"
    if [ -n "$a" ] && [ "$a" != "0" ]; then
        if grep -qE '^[[:space:]]*stick [0-9]+' "$MOBILE_CFG" 2>/dev/null; then
            sed -i "s/^\([[:space:]]*\)stick [0-9]\+/\1stick $a/" "$MOBILE_CFG"
        elif grep -qE '^[[:space:]]*no stick' "$MOBILE_CFG" 2>/dev/null; then
            sed -i "s/^\([[:space:]]*\)no stick/\1stick $a/" "$MOBILE_CFG"
        else
            mod_say "aucune directive stick/no stick dans $MOBILE_CFG : rien à surcharger"
        fi
        mod_say "stick demandé = $a, lu = $(_mcfg_stick)"
    fi

    MOBILE_IMSI="$(_mcfg_imsi)"
    MOBILE_KI="$(_mcfg_ki)"
    mod_say "IMSI = ${MOBILE_IMSI:-<non trouvé>}"
    mod_say "Ki   = ${MOBILE_KI:+<présent, ${#MOBILE_KI} caractères>}${MOBILE_KI:-<non trouvé>}"
    mod_ok
}

_mcfg_ready() {
    [ -s "${MOBILE_CFG:-}" ] || return 1
    [ -n "$(_mcfg_imsi)" ] || return 1
    [ -n "$(_mcfg_ki)" ]   || return 1
    local a="${CALYPSO_STICK_ARFCN:-}"
    if [ -n "$a" ] && [ "$a" != "0" ] && grep -qE '^[[:space:]]*stick ' "$MOBILE_CFG" 2>/dev/null; then
        [ "$(_mcfg_stick)" = "$a" ] || return 1
    fi
    return 0
}

mod_mobile_cfg_wait() {
    if ! wait_until "${MOD_TIMEOUT[mobile-cfg]}" "configuration du mobile" _mcfg_ready; then
        if [ ! -s "${MOBILE_CFG:-}" ]; then
            mod_fail "$MOBILE_CFG absent ou vide après la copie"
        elif [ -n "${CALYPSO_STICK_ARFCN:-}" ] && [ "$(_mcfg_stick)" != "${CALYPSO_STICK_ARFCN}" ]; then
            mod_hint "vérifiez la ligne 'stick' de $MOBILE_CFG_SRC"
            mod_fail "ARFCN non appliqué : demandé ${CALYPSO_STICK_ARFCN}, lu $(_mcfg_stick)"
        else
            mod_hint "attendu dans $MOBILE_CFG : une ligne 'imsi <15 chiffres>' et une ligne 'ki comp128 <16 octets>'"
            mod_fail "IMSI ou Ki introuvable dans $MOBILE_CFG — le HLR ne pourra pas être alimenté"
        fi
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_mobile_cfg_stop() { return 0; }
