MOD_REGISTER rundir "Répertoire d'exécution"
MOD_REQUIRED[rundir]=1
MOD_DEPS[rundir]="profil"
MOD_TIMEOUT[rundir]=15

mod_rundir_check() {
    local d="${RUN_DIR:-}" parent
    [ -n "$d" ] || { mod_hint "RUN_DIR est posé par environnement/paths.env"
                     mod_fail "RUN_DIR non défini"
                     return $MOD_RC_FAIL; }
    if [ -e "$d" ] && [ ! -d "$d" ]; then
        mod_hint "rm -f $d   (ou choisissez un autre RUN_DIR)"
        mod_fail "$d existe et n'est pas un répertoire"
        return $MOD_RC_FAIL
    fi
    if [ ! -d "$d" ]; then
        parent="$(dirname "$d")"
        while [ ! -d "$parent" ] && [ "$parent" != "/" ]; do parent="$(dirname "$parent")"; done
        if [ ! -w "$parent" ]; then
            mod_hint "choisissez un emplacement inscriptible :  RUN_DIR=\$HOME/calypso ./start-oqc.sh"
            mod_fail "impossible de créer $d : $parent n'est pas inscriptible"
            return $MOD_RC_FAIL
        fi
    fi
    mod_ok
}

mod_rundir_status() { [ -d "${RUN_DIR:-}" ] && [ -w "${RUN_DIR:-}" ]; }

mod_rundir_start() {
    mkdir -p "${RUN_DIR}" 2>/dev/null || { mod_fail "création impossible : ${RUN_DIR}"; return $MOD_RC_FAIL; }
    chmod 0755 "${RUN_DIR}" 2>/dev/null
    mod_say "répertoire d'exécution : ${RUN_DIR}"
    local f n=0
    for f in "${RUN_DIR}"/*.pid "${RUN_DIR}"/*.sock; do
        [ -e "$f" ] && { mod_say "reste du run précédent : $f"; n=$((n+1)); }
    done
    [ "$n" -gt 0 ] && mod_say "$n reste(s) — 04-restes puis 10-teardown s'en chargent"
    mod_ok
}

_rundir_ecriture_ok() {
    local t="${RUN_DIR}/.temoin.$$"
    : > "$t" 2>/dev/null || return 1
    rm -f "$t" 2>/dev/null
    return 0
}

mod_rundir_wait() {
    if ! wait_until "${MOD_TIMEOUT[rundir]}" "écriture dans ${RUN_DIR}" _rundir_ecriture_ok; then
        mod_hint "montage en lecture seule ou volume plein :  df -h ${RUN_DIR}"
        mod_fail "${RUN_DIR} existe mais l'écriture y échoue"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_rundir_stop() {
    local f pid
    for f in "${RUN_DIR:-/nonexistent}"/*.pid; do
        [ -f "$f" ] || continue
        pid="$(cat "$f" 2>/dev/null || echo 0)"
        kill -0 "$pid" 2>/dev/null || rm -f "$f"
    done
    return 0
}
