MOD_REGISTER summary "Identité du build et résumé"
MOD_REQUIRED[summary]=0
MOD_PURE[summary]=1
MOD_TIMEOUT[summary]=10

mod_summary_check() { mod_ok; }
mod_summary_status() { return $MOD_RC_FAIL; }

mod_summary_start() {
    mod_say "===== IDENTITÉ DU BUILD ====="
    if command -v git >/dev/null 2>&1 && [ -d "${QEMU_TREE}/.git" ]; then
        mod_say "  révision git    : $(git -C "$QEMU_TREE" rev-parse --short HEAD 2>/dev/null || echo inconnue)"
        mod_say "  arbre modifié   : $(git -C "$QEMU_TREE" diff --stat 2>/dev/null | tail -1)"
    fi
    mod_say "  binaire QEMU    : ${QEMU_BIN:-<non défini>}"
    mod_say "  date du binaire : $(stat -c %y "${QEMU_BIN:-/nonexistent}" 2>/dev/null | cut -d. -f1)"
    mod_say "  firmware        : ${FIRMWARE_ELF:-<non défini>}"
    mod_say "===== RÉSUMÉ D'ENVIRONNEMENT ====="
    mod_say "  couche 1        : gr-gsm (GSMTAP ${PORT_GSMTAP_SI:-4730}, SCH ${PORT_GSMTAP_SCH:-4731}) · pont TRXD ${PORT_TRXD_CLOCK:-5700}-${PORT_TRXD_DATA:-5702}"
    mod_say "  cellule du pont : ARFCN ${PONT_ARFCN:-?} BSIC ${PONT_BSIC:-?}"
    mod_say "  chiffrement     : ${ENCRYPTION:-?}"
    mod_say "  MS#1 / MS#2     : ${MOBILE_CFG:-?} / ${SC_MOBILE_CFG:-?}"
    mod_say "  LOG_DIR / RUN_DIR : ${LOG_DIR:-} / ${RUN_DIR:-}"
    mod_ok
}
