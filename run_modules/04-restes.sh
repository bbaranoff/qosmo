MOD_REGISTER restes "Extinction des processus enregistrés"
MOD_REQUIRED[restes]=0
MOD_DEPS[restes]="rundir"
MOD_TIMEOUT[restes]=20
MOD_ENABLED_IF[restes]='[ "${NO_STARTUP_STOP:-0}" != 1 ]'

_res_pids() {
    local f pid
    for f in "${RUN_DIR:-/nonexistent}"/*.pid; do
        [ -f "$f" ] || continue
        pid="$(cat "$f" 2>/dev/null || echo 0)"
        [ "$pid" -gt 0 ] 2>/dev/null || continue
        [ "$pid" = "$$" ] && continue
        kill -0 "$pid" 2>/dev/null && printf '%s\n' "$pid"
    done
}
_res_vide() { [ -z "$(_res_pids)" ]; }

_res_cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | cut -c1-140; }

_res_signal() {
    local sig="$1" p
    for p in $(_res_pids); do
        kill "-$sig" -- "-$p" 2>/dev/null || kill "-$sig" "$p" 2>/dev/null
    done
    return 0
}

_res_menage() {
    local f pid
    for f in "${RUN_DIR:-/nonexistent}"/*.pid; do
        [ -f "$f" ] || continue
        pid="$(cat "$f" 2>/dev/null || echo 0)"
        kill -0 "$pid" 2>/dev/null || { rm -f "$f"; mod_say "fichier PID retiré : $f"; }
    done
    return 0
}

mod_restes_check() {
    command -v kill >/dev/null 2>&1 || { mod_fail "commande kill introuvable"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_restes_status() { _res_vide; }

mod_restes_start() {
    local liste p
    liste="$(_res_pids | tr '\n' ' ')"
    if [ -z "${liste// /}" ]; then
        _res_menage
        mod_already "aucun processus enregistré n'a survécu au run précédent"
        return $MOD_RC_ALREADY
    fi
    for p in $liste; do mod_say "encore vivant : $p  $(_res_cmdline "$p")"; done
    _res_signal TERM
    mod_ok
}

mod_restes_wait() {
    local moitie=$(( ${MOD_TIMEOUT[restes]} / 2 ))
    [ "$moitie" -lt 2 ] && moitie=2

    if ! wait_until "$moitie" "extinction douce" _res_vide; then
        mod_say "toujours vivants après TERM : $(_res_pids | tr '\n' ' ') — passage en KILL"
        _res_signal KILL
        if ! wait_until "$moitie" "extinction forcée" _res_vide; then
            mod_hint "identifiez le tenace :  ps -o pid,ppid,stat,cmd -p \$(cat ${RUN_DIR:-/tmp/calypso}/*.pid | tr '\\n' ',')"
            mod_fail "processus enregistrés impossibles à arrêter : $(_res_pids | tr '\n' ' ')"
            return $MOD_RC_FAIL
        fi
    fi
    _res_menage
    mod_ok
}

mod_restes_stop() {
    _res_vide && { _res_menage; return 0; }
    _res_signal TERM
    wait_until 5 "extinction douce" _res_vide || { _res_signal KILL; wait_until 5 "extinction forcée" _res_vide; }
    _res_menage
    return 0
}
