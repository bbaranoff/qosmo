#!/bin/bash
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODDIR="$HERE/run_modules"

DRY=0 ONLY="" SKIP="" ACTION=start FORCE=0
NO_ATTACH="${CALYPSO_NO_ATTACH:-0}"

usage() {
    cat <<'USAGE'
Usage : ./run.sh [options]
  --list              affiche le plan et sort, sans rien lancer
  --dry-run           déroule le plan sans effet de bord
  --only  <slugs>     ne joue que ces modules (séparés par des virgules)
  --skip  <slugs>     saute ces modules
  --stop              arrête la pile (plan en ordre inverse)
  --status            interroge l'état de chaque module
  --force             relance même les modules déjà démarrés
  --no-attach         ne pas s'attacher à tmux à la fin
  --check-paths       vérifie que les dépendances déclarées existent
  --restart           = --reset puis démarrage — la relance sûre au quotidien
  --reset             repart d'un état propre : arrête la pile, tue le serveur
                      tmux, archive les journaux, purge sockets et fichiers partagés
  --lang <code>       langue de l'affichage (fr de en es pt it hi zh ar)
  --profile <nom>     accepté pour compatibilité, sans effet (un seul banc)
  --verbose           accepté pour compatibilité, sans effet
  -h, --help          cette aide
USAGE
}

_reset() {
    local horodat rep n=0
    printf '\nRemise à zéro — rien ne sera relancé.\n\n' >&2
    printf '  arrêt de la pile…\n' >&2
    "$0" --stop >/dev/null 2>&1 || true
    local logdir="$LOG_DIR"
    if [ -d "$logdir" ] && [ -n "$(ls -A "$logdir" 2>/dev/null)" ]; then
        horodat="$(date +%Y%m%d-%H%M%S)"
        rep="$logdir/../archives/$horodat"
        mkdir -p "$rep" 2>/dev/null && \
        if cp -a "$logdir/." "$rep/" 2>/dev/null; then
            printf '  journaux archivés     %s\n' "$rep" >&2
        fi
        local arcdir="$logdir/../archives"
        local keep="${CALYPSO_ARCHIVES_KEEP:-3}"
        local maxmb="${CALYPSO_ARCHIVE_MAX_MB:-32}"
        local f taille avant apres
        for f in "$rep"/*; do
            [ -f "$f" ] || continue
            taille=$(stat -c %s "$f" 2>/dev/null || echo 0)
            if [ "$taille" -gt $(( maxmb * 1024 * 1024 )) ]; then
                head -c $(( maxmb * 1024 * 1024 )) "$f" > "$f.tete" 2>/dev/null &&
                mv -f "$f.tete" "$f" 2>/dev/null &&
                printf '  archive tronquée      %s (%s Mo -> %s Mo, tête conservée)\n' \
                       "$(basename "$f")" "$(( taille / 1048576 ))" "$maxmb" >&2
            fi
        done
        if [ -d "$arcdir" ]; then
            avant=$(du -sm "$arcdir" 2>/dev/null | cut -f1)
            ls -1t "$arcdir" 2>/dev/null | tail -n +$(( keep + 1 )) | while read -r vieux; do
                [ -n "$vieux" ] && rm -rf -- "$arcdir/$vieux"
            done
            apres=$(du -sm "$arcdir" 2>/dev/null | cut -f1)
            [ "${avant:-0}" -gt "${apres:-0}" ] 2>/dev/null &&
                printf '  archives purgées      %s gardées (%s Mo -> %s Mo)\n' \
                       "$keep" "$avant" "$apres" >&2
        fi
        rm -f "$logdir"/*.log 2>/dev/null
        printf '  journaux remis à zéro %s\n' "$logdir" >&2
        df -h "$logdir" 2>/dev/null | tail -1 | \
            awk '{printf "  espace disponible     %s (%s utilisé)\n", $4, $5}' >&2
    fi
    if command -v tmux >/dev/null 2>&1 && tmux ls >/dev/null 2>&1; then
        tmux kill-server 2>/dev/null || true
        printf '  serveur tmux tué      (environnement global purgé)\n' >&2
    fi
    local motifs="qemu-system-arm osmocon osmo-bts-trx fake_trx trxcon"
    local m p pids
    for m in $motifs; do
        pids="$(pgrep -f -- "$m" 2>/dev/null || true)"
        for p in $pids; do
            [ "$p" = "$$" ] && continue
            [ "$p" = "${PPID:-0}" ] && continue
            kill -TERM "$p" 2>/dev/null && n=$((n + 1))
        done
    done
    [ "$n" -gt 0 ] && printf '  processus arrêtés     %d\n' "$n" >&2
    local art
    for art in /tmp/osmocom_l2 /tmp/ms2_l2 /dev/shm/calypso_*; do
        [ -e "$art" ] && rm -f "$art" 2>/dev/null && printf '  supprimé              %s\n' "$art" >&2
    done
    printf '\nÉtat propre. Relancez :\n' >&2
    printf '   ./run.sh\n\n' >&2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --list)        ACTION=list ;;
        --dry-run)     DRY=1 ;;
        --only)        ONLY="${2:-}"; shift ;;
        --skip)        SKIP="${2:-}"; shift ;;
        --profile)     shift ;;
        --stop)        ACTION=stop ;;
        --status)      ACTION=status ;;
        --force)       FORCE=1 ;;
        --no-attach)   NO_ATTACH=1 ;;
        --verbose)     ;;
        --check-paths) ACTION=checkpaths ;;
        --reset)       ACTION=reset ;;
        --restart)     ACTION=restart ;;
        --lang)        CALYPSO_LANG="${2:-}"; export CALYPSO_LANG; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) printf 'option inconnue : %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [ -f "$HERE/environnement/bench.env" ]; then
    set -a; . "$HERE/environnement/bench.env"; set +a
else
    printf 'configuration introuvable : %s\n' "$HERE/environnement/bench.env" >&2
    exit 2
fi

if [ "$ACTION" = reset ]; then _reset; exit 0; fi
if [ "$ACTION" = restart ]; then _reset; ACTION=start; fi

LOGDIR="$LOG_DIR"
mkdir -p "$LOGDIR/mod" 2>/dev/null || true

if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
    TTY=1; C_OK=$'\033[32m'; C_KO=$'\033[31m'; C_SK=$'\033[33m'; C_DIM=$'\033[2m'; C_Z=$'\033[0m'
else
    TTY=0; C_OK=""; C_KO=""; C_SK=""; C_DIM=""; C_Z=""
fi
say_begin() { if [ $TTY -eq 1 ]; then printf '[ %s.. %s] %s' "$C_DIM" "$C_Z" "$1"; else printf '[ .. ] %s\n' "$1"; fi; }
say_end()   {
    if [ $TTY -eq 1 ]; then printf '\r\033[K'; fi
    printf '[%s%-4s%s] %s' "$2" "$1" "$C_Z" "$3"
    [ -n "${4:-}" ] && printf ' %s(%s)%s' "$C_DIM" "$4" "$C_Z"
    printf '\n'
}

. "$MODDIR/_lib/mod.sh"
shopt -s nullglob
for f in "$MODDIR"/[0-9][0-9]-*.sh; do . "$f"; done
shopt -u nullglob

if [ ${#MOD_ORDER[@]} -eq 0 ]; then
    printf '%s\n' "$(t aucun_mod "$MODDIR")" >&2
    exit 2
fi

in_csv() { case ",$1," in *",$2,"*) return 0;; esac; return 1; }
selected=()
for m in "${MOD_ORDER[@]}"; do
    [ -n "$ONLY" ] && { in_csv "$ONLY" "$m" || continue; }
    [ -n "$SKIP" ] && { in_csv "$SKIP" "$m" && continue; }
    selected+=("$m")
done

if [ "$ACTION" = list ]; then
    printf '%s\n\n' "$(t plan "${#selected[@]}")"
    for m in "${selected[@]}"; do
        printf '  %-18s %-42s %s%s\n' "$m" "$(t_mod "$m")" \
            "$([ "${MOD_REQUIRED[$m]}" = 1 ] && t obligatoire || t optionnel)" \
            "$([ -n "${MOD_DEPS[$m]}" ] && printf '  %s' "$(t apres "${MOD_DEPS[$m]}")")"
    done
    exit 0
fi

if [ "$ACTION" = checkpaths ]; then
    rc=0
    for v in QEMU_BIN FIRMWARE_ELF FIRMWARE_BIN OSMOCON OSMOCOM_CFG; do
        p="${!v:-}"
        if [ -z "$p" ];      then say_end "$(t fail)" "$C_KO" "$v" "non défini"; rc=1
        elif [ -e "$p" ];    then say_end " $(t ok) " "$C_OK" "$v" "$p"
        else                      say_end "$(t fail)" "$C_KO" "$v" "introuvable : $p"; rc=1; fi
    done
    exit $rc
fi

declare -A STATE=()
nb_ok=0 nb_fail=0 nb_skip=0 nb_warn=0
[ "$ACTION" = stop ] && { tmp=(); for ((i=${#selected[@]}-1;i>=0;i--)); do tmp+=("${selected[$i]}"); done; selected=("${tmp[@]}"); }

for m in "${selected[@]}"; do
    p="$(mod_prefix "$m")"
    log="${MOD_LOG[$m]:-$LOGDIR/mod/$m.log}"
    _MOD_REASON=""; _MOD_HINT=""

    if [ "$ACTION" = stop ]; then
        say_begin "$(t arret "$(t_mod "$m")")"
        declare -F "${p}_stop" >/dev/null && "${p}_stop" >>"$log" 2>&1
        say_end " $(t ok) " "$C_OK" "$(t arret "$(t_mod "$m")")"
        continue
    fi

    if [ "$ACTION" = status ]; then
        if declare -F "${p}_status" >/dev/null && "${p}_status" >>"$log" 2>&1; then
            say_end "$(t up)" "$C_OK" "$(t_mod "$m")"
        else
            say_end "$(t down)" "$C_SK" "$(t_mod "$m")"
        fi
        continue
    fi

    if ! eval "${MOD_ENABLED_IF[$m]}" 2>/dev/null; then
        say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "$(t desactive "${MOD_ENABLED_IF[$m]}")"
        STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
    fi

    dep_ko=""
    for d in ${MOD_DEPS[$m]}; do
        case "${STATE[$d]:-absent}" in ok|skip) ;; *) dep_ko="$d"; break;; esac
    done
    if [ -n "$dep_ko" ]; then
        say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "$(t dep_ko "$dep_ko")"
        STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
    fi

    say_begin "$(t_mod "$m")"

    if [ $FORCE -eq 0 ] && declare -F "${p}_status" >/dev/null && "${p}_status" >>"$log" 2>&1; then
        say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "$(t deja)"
        STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
    fi

    if declare -F "${p}_check" >/dev/null; then
        "${p}_check" >>"$log" 2>&1; rc=$?
        if [ $rc -eq $MOD_RC_FAIL ] && [ $DRY -eq 1 ]; then
            say_end "$(t dry)" "$C_DIM" "$(t_mod "$m")" \
                    "$(t prereq_sim "${_MOD_REASON:-}")"
            STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
        fi
        if [ $rc -eq $MOD_RC_FAIL ]; then
            say_end "$(t fail)" "$C_KO" "$(t_mod "$m")" "${_MOD_REASON:-$(t sans_raison)}"
            [ -n "$_MOD_HINT" ] && printf '       %s→ %s%s\n' "$C_DIM" "$_MOD_HINT" "$C_Z"
            printf '       %s%s%s\n' "$C_DIM" "$(t journal "$log")" "$C_Z"
            STATE[$m]=fail; nb_fail=$((nb_fail+1))
            [ "${MOD_REQUIRED[$m]}" = 1 ] && { printf '\n%s\n' "$(t interrompu)"; exit 1; }
            nb_warn=$((nb_warn+1)); continue
        elif [ $rc -eq $MOD_RC_SKIP ]; then
            say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "${_MOD_REASON:-}"
            STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
        fi
    fi

    if [ $DRY -eq 1 ] && [ "${MOD_PURE[$m]}" != 1 ]; then
        say_end " -- " "$C_DIM" "$(t_mod "$m")" "$(t simule)"
        STATE[$m]=ok; continue
    fi

    "${p}_start" >>"$log" 2>&1; rc=$?
    case $rc in
        $MOD_RC_ALREADY) say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "${_MOD_REASON:-$(t deja)}"
                         STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue ;;
        $MOD_RC_SKIP)    say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "${_MOD_REASON:-}"
                         STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue ;;
        $MOD_RC_FAIL)    say_end "$(t fail)" "$C_KO" "$(t_mod "$m")" "${_MOD_REASON:-$(t sans_raison)}"
                         [ -n "$_MOD_HINT" ] && printf '       %s→ %s%s\n' "$C_DIM" "$_MOD_HINT" "$C_Z"
                         printf '       %s%s%s\n' "$C_DIM" "$(t journal "$log")" "$C_Z"
                         STATE[$m]=fail; nb_fail=$((nb_fail+1))
                         [ "${MOD_REQUIRED[$m]}" = 1 ] && { printf '\n%s\n' "$(t interrompu)"; exit 1; }
                         nb_warn=$((nb_warn+1)); continue ;;
    esac

    if declare -F "${p}_wait" >/dev/null; then
        if ! "${p}_wait" >>"$log" 2>&1; then
            say_end "$(t fail)" "$C_KO" "$(t_mod "$m")" "$(t pas_pret "${_MOD_REASON:-délai dépassé}")"
            printf '       %s%s%s\n' "$C_DIM" "$(t journal "$log")" "$C_Z"
            STATE[$m]=fail; nb_fail=$((nb_fail+1))
            [ "${MOD_REQUIRED[$m]}" = 1 ] && { printf '\n%s\n' "$(t interrompu)"; exit 1; }
            nb_warn=$((nb_warn+1)); continue
        fi
    fi

    say_end " $(t ok) " "$C_OK" "$(t_mod "$m")"
    STATE[$m]=ok; nb_ok=$((nb_ok+1))
done

printf '\n%s\n' "$(t bilan "$nb_ok" "$nb_skip" "$nb_fail")"

if [ "$ACTION" = start ] && [ $DRY -eq 0 ] && [ $nb_fail -eq 0 ]; then
    _sess="${TMUX_SESSION:-calypso}"
    printf '\n'
    if tmux has-session -t "$_sess" 2>/dev/null; then
        printf '  %s%s%s   tmux attach -t %s\n' "$C_OK" "$(t connecter)" "$C_Z" "$_sess"
    else
        printf '  %s%s%s   %s\n' "$C_DIM" "$(t connecter)" "$C_Z" "$(t pas_session "$_sess")"
    fi
    printf '  %s%s%s       %s\n' "$C_DIM" "$(t journaux)" "$C_Z" "$LOGDIR"
    printf '  %s%s%s           ./run.sh --status\n' "$C_DIM" "$(t etat)" "$C_Z"
    printf '  %s%s%s        ./run.sh --stop\n' "$C_DIM" "$(t arreter)" "$C_Z"
    printf '\n'
    if [ $TTY -eq 1 ] && [ "$NO_ATTACH" != 1 ] && [ -z "${TMUX:-}" ] \
       && tmux has-session -t "$_sess" 2>/dev/null; then
        printf '  %sconnexion a la session…  (Ctrl-b d pour detacher)%s\n\n' "$C_DIM" "$C_Z"
        exec tmux attach -t "$_sess"
    fi
fi

[ $nb_fail -gt 0 ] && exit 1
exit 0
