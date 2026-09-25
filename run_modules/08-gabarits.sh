MOD_REGISTER gabarits "Gabarits de configuration du cœur"
MOD_REQUIRED[gabarits]=0
MOD_DEPS[gabarits]="config"
MOD_TIMEOUT[gabarits]=20

MOD_ENABLED_IF[gabarits]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${NO_GABARITS:-0}" != 1 ]'

: "${EGPRS_DIR:=${NITB_ROOT}}"
: "${EGPRS_LIB:=$EGPRS_DIR/lib/gabarits.sh}"
: "${OPERATOR_ID:=1}"
: "${MCC:=001}"
: "${MNC:=01}"
: "${OPERATOR_NAME:=OsmoDirect}"
: "${ENCRYPTION:=a5 0}"

_gab_installes() {
    printf '%s\n' "${OSMOCOM_CFG:-/etc/osmocom}/osmo-bsc.cfg" \
                  "${OSMOCOM_CFG:-/etc/osmocom}/osmo-msc.cfg"
}

mod_gabarits_check() {
    if [ ! -d "$EGPRS_DIR/configs" ]; then
        mod_hint "posez EGPRS_DIR=<racine osmo_egprs> si l'arborescence a bougé"
        mod_skip "osmo_egprs introuvable ($EGPRS_DIR) : les gabarits ne sont pas de ce dépôt"
        return $MOD_RC_SKIP
    fi
    if [ ! -r "$EGPRS_LIB" ]; then
        mod_hint "la bibliothèque est l'extraction de start-direct.sh.legacy L174-258 ; elle doit accompagner osmo_egprs"
        mod_fail "bibliothèque de gabarits absente : $EGPRS_LIB"
        return $MOD_RC_FAIL
    fi
    if [ "$(id -u)" -ne 0 ]; then
        mod_fail "root requis pour écrire dans ${OSMOCOM_CFG:-/etc/osmocom} et /etc/asterisk"
        return $MOD_RC_FAIL
    fi
    if [ "${N_OPERATORS:-1}" -gt 1 ]; then
        mod_hint "le multi-opérateur pose une configuration par netns ; aucun module du cœur ne démarre encore dans un netns"
        mod_fail "N_OPERATORS=${N_OPERATORS} : ce module ne sait poser que le mono-opérateur (cf. POURQUOI 4)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_gabarits_status() { return $MOD_RC_FAIL; }

mod_gabarits_start() {
    local retour="$PWD" rc=0
    cd "$EGPRS_DIR" || { mod_fail "impossible d'entrer dans $EGPRS_DIR"; return $MOD_RC_FAIL; }

    . "$EGPRS_LIB" || { cd "$retour"; mod_fail "bibliothèque illisible : $EGPRS_LIB"; return $MOD_RC_FAIL; }

    local tmpdir; tmpdir="$(mktemp -d)" || { cd "$retour"; mod_fail "mktemp -d a échoué"; return $MOD_RC_FAIL; }

    local cip gw
    cip="$(op_private_ip "$OPERATOR_ID")"
    gw="$(op_private_gw "$OPERATOR_ID")"

    mod_say "opérateur $OPERATOR_ID · MCC/MNC ${MCC}/${MNC} · chiffrement « ${ENCRYPTION} »"
    mod_say "gabarits  : $EGPRS_DIR/configs -> ${OSMOCOM_CFG:-/etc/osmocom}, /etc/asterisk, $HOME/.osmocom/bb"

    local _inter_shutdown="shutdown" _multi="/etc/osmocom/osmo-multi.conf"
    if [ -r "$_multi" ] && grep -q '^MULTI_HUB_IP=' "$_multi" 2>/dev/null; then
        _inter_shutdown=""
        mod_say "inter-STP déclaré (osmo-multi.conf) : ASP as-inter laissé ACTIF"
    fi
    if apply_config_templates "$tmpdir" "$cip" "$gw" "$OPERATOR_ID" \
            "1.1.1" "1.1.2" "1.1.3" "$MCC" "$MNC" "$OPERATOR_NAME" \
            "${INTER_STP_IP:-127.0.0.1}" "$_inter_shutdown" "1"; then
        install_configs_native "$tmpdir" "" || rc=1
    else
        rc=1
    fi

    rm -rf "$tmpdir"
    cd "$retour"
    [ $rc -eq 0 ] || { mod_fail "la pose des gabarits a échoué (détail dans le journal du module)"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_gabarits_wait() {
    local f restants=""
    while IFS= read -r f; do
        if [ ! -f "$f" ]; then
            mod_hint "vérifiez que $EGPRS_DIR/configs contient bien le gabarit correspondant"
            mod_fail "configuration non installée : $f"
            return $MOD_RC_FAIL
        fi
        if grep -qE '__[A-Z0-9_]+__' "$f" 2>/dev/null; then
            restants="$restants $(basename "$f"):$(grep -oE '__[A-Z0-9_]+__' "$f" | sort -u | tr '\n' ',' )"
        fi
    done < <(_gab_installes)

    if [ -n "$restants" ]; then
        mod_hint "ces jetons n'ont pas de valeur dans apply_config_templates (lib/gabarits.sh)"
        mod_fail "jetons non substitués :$restants"
        return $MOD_RC_FAIL
    fi

    local cfg
    for cfg in "${OSMOCOM_CFG:-/etc/osmocom}/osmo-bsc.cfg" \
               "${OSMOCOM_CFG:-/etc/osmocom}/osmo-msc.cfg"; do
        if grep -q '__ENCRYPTION__' "$cfg" 2>/dev/null || ! grep -qF "$ENCRYPTION" "$cfg" 2>/dev/null; then
            mod_hint "attendu « $ENCRYPTION » dans $cfg ; vérifiez que le gabarit porte bien __ENCRYPTION__"
            mod_fail "le chiffrement demandé n'est pas dans la configuration installée ($cfg)"
            return $MOD_RC_FAIL
        fi
        mod_say "chiffrement « $ENCRYPTION » confirmé dans $cfg"
    done
    mod_ok
}

mod_gabarits_stop() { return 0; }
