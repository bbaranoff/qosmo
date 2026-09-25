. "$(dirname "${BASH_SOURCE[0]}")/i18n.sh"

readonly MOD_RC_OK=0
readonly MOD_RC_FAIL=1
readonly MOD_RC_UNKNOWN=2
readonly MOD_RC_ALREADY=3
readonly MOD_RC_SKIP=4

declare -a MOD_ORDER=()
declare -A MOD_DESC=()
declare -A MOD_DEPS=()
declare -A MOD_REQUIRED=()
declare -A MOD_LOG=()
declare -A MOD_JOURNAL=()
declare -A MOD_PURE=()
declare -A MOD_ENABLED_IF=()
declare -A MOD_TIMEOUT=()

MOD_REGISTER() {
    local slug="$1" desc="$2"
    MOD_ORDER+=("$slug")
    MOD_DESC[$slug]="$desc"
    MOD_DEPS[$slug]=""
    MOD_REQUIRED[$slug]=1
    MOD_LOG[$slug]=""
    MOD_JOURNAL[$slug]=""
    MOD_PURE[$slug]=0
    MOD_ENABLED_IF[$slug]='true'
    MOD_TIMEOUT[$slug]=30
}

mod_prefix() { printf 'mod_%s' "${1//-/_}"; }

_MOD_REASON=""
_MOD_HINT=""

mod_ok()      { _MOD_REASON=""   ; return $MOD_RC_OK; }
mod_fail()    { _MOD_REASON="$*" ; return $MOD_RC_FAIL; }
mod_already() { _MOD_REASON="$*" ; return $MOD_RC_ALREADY; }
mod_skip()    { _MOD_REASON="$*" ; return $MOD_RC_SKIP; }
mod_hint()    { _MOD_HINT="$*"; }
mod_say()     { printf '%s\n' "$*"; }

wait_until() {
    local timeout="$1" what="$2"; shift 2
    local deadline=$(( SECONDS + timeout ))
    while (( SECONDS < deadline )); do
        if "$@" >/dev/null 2>&1; then return $MOD_RC_OK; fi
        sleep 0.2
    done
    mod_fail "$what : toujours pas prêt après ${timeout}s"
}

have_proc() {
    local motif="$1" p ligne ns
    ns="$(readlink /proc/self/ns/pid 2>/dev/null)"
    for p in $(pgrep -f "$motif" 2>/dev/null); do
        [ "$p" = "$$" ] || [ "$p" = "$PPID" ] && continue
        [ -z "$ns" ] || [ "$(readlink "/proc/$p/ns/pid" 2>/dev/null)" = "$ns" ] || continue
        ligne="$( { tr '\0' ' ' < "/proc/$p/cmdline"; } 2>/dev/null )"
        [ -n "$ligne" ] || continue
        case "$ligne" in
            *"tail "*|*"tail -"*|*"less "*|*"multitail "*|*"watch "*|\
            *"sleep infinity"*|*"journalctl "*) continue ;;
        esac
        return 0
    done
    return 1
}
have_port()  { (exec 3<>"/dev/tcp/127.0.0.1/$1") >/dev/null 2>&1; }
have_unix()  { [ -S "$1" ]; }
log_has()    { [ -f "$1" ] && grep -q "$2" "$1" 2>/dev/null; }

modb_tail() {
    local f="$1" n="${2:-20}"
    if [ ! -r "$f" ]; then mod_say "journal absent ou illisible : $f"; return 0; fi
    mod_say "--- $n dernières lignes de $f ---"
    tail -n "$n" "$f" 2>/dev/null
    mod_say "--- fin de $f ---"
}
