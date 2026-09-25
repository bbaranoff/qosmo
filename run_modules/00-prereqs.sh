MOD_REGISTER prereqs "Vérification des prérequis"
MOD_REQUIRED[prereqs]=1
MOD_PURE[prereqs]=1

mod_prereqs_check() {
    local missing="" v p
    for v in QEMU_BIN FIRMWARE_ELF FIRMWARE_BIN OSMOCON; do
        p="${!v:-}"
        [ -n "$p" ] && [ -e "$p" ] || missing="$missing $v"
    done
    if [ -n "$missing" ]; then
        mod_hint "réglez les chemins dans environnement/bench.env, puis ./run.sh --check-paths"
        mod_fail "dépendances introuvables :$missing"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_prereqs_start() {
    mod_say "arbre         : ${QEMU_TREE}"
    mod_say "binaire QEMU  : ${QEMU_BIN}"
    mod_say "firmware      : ${FIRMWARE_ELF}"
    mod_say "osmocon       : ${OSMOCON}"
    mod_say "exécution     : ${RUN_DIR}"
    mod_say "journaux      : ${LOG_DIR}"
    mod_ok
}

mod_prereqs_status() { return $MOD_RC_FAIL; }
