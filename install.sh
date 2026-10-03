#!/bin/bash
# =============================================================================
#  install.sh - construit et installe qosmo (QEMU, machine Calypso)
# =============================================================================
#
#  Autonome : ce depot et quelques paquets apt suffisent, osmo-operator n est
#  pas necessaire. C est aussi CE script qu appellent le Dockerfile
#  d osmo-operator (stage qemu), son Dockerfile.run, start.sh et l installation
#  native (install_modules/45-calypso.sh) : une seule liste de commandes.
#
#      ./install.sh                 tout, dans l ordre (voir --list)
#      ./install.sh --check         prerequis et etat, NE MODIFIE RIEN
#      ./install.sh --list          les etapes et les valeurs retenues
#      ./install.sh --print-deps    les paquets apt, sur une ligne
#      sudo ./install.sh --deps     installe les paquets apt manquants, rien d autre
#      sudo ./install.sh --with-deps   les paquets manquants, puis tout
#      ./install.sh --only build    une ou des etapes (liste a virgules)
#      ./install.sh --skip bin      toutes sauf celles-la
#      ./install.sh --reinstall     rejoue meme ce qui est deja fait (reconfigure)
#      ./install.sh -v              la sortie des commandes a l ecran (sinon : journal)
#
#  Etapes : venv configure build install bin verify
#
#  Options, et la variable qui fait la meme chose (l option l emporte) :
#      --prefix DIR     PREFIX     $GSM_ROOT/qemu-install  ou `make install` pose QEMU
#      --build-dir DIR  BUILD_DIR  $QOSMO/build            construire hors de l arbre
#      --venv DIR       VENV       $HOME/.venv-qemu        venv de configure (tomli) ; none = python du systeme
#      --bin-dir DIR    BIN_DIR    /usr/local/bin          copie de qemu-system-arm et qosmo ; none = aucune
#      --destdir DIR    DESTDIR    (vide)                  installer sous une autre racine (rootfs d ISO)
#      --cc NOM         QOSMO_CC   (vide)                  compilateur impose, ex. gcc-13 (build/ configure avec lui)
#      -j N             JOBS       nproc
#                       GSM_ROOT   /opt/GSM     QOSMO = l arbre a construire, ce depot par defaut
#                       LOG_DIR    /tmp/qosmo-install (un journal par etape)
#
#  Les lanceurs lisent $QOSMO/build/qemu-system-arm (c54x_exe/run.sh,
#  environnement/bench.env) et c54x_exe / grgsm_exe compilent les sources de cet
#  arbre : gardez l arbre ET build/.
#
#  Correspondance avec osmo-operator/Dockerfile : chaque etape cite ses lignes,
#  celles du Dockerfile au commit 4266c8b, quand il portait encore ces commandes
#  (stage qemu, 691-705 ; paquets 156-205). Il appelle desormais ce script.
# -----------------------------------------------------------------------------
set -uo pipefail
ICI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ═════════════════════════════════════════════════════════════════════════════
#  LE CONTRAT - celui de osmo-operator/install_modules/_lib/inst.sh
# ═════════════════════════════════════════════════════════════════════════════
#  Copie IDENTIQUE dans qosmo/install.sh, c54x_exe/install.sh et
#  grgsm_exe/install.sh : chaque depot doit pouvoir s installer seul, sans
#  osmo-operator. Une correction ici se reporte dans les deux autres.
#
#  Quatre fonctions par etape, `run` seule obligatoire :
#     inst_<etape>_check    prerequis, LECTURE SEULE        0 ok · 1 manque · 4 sans objet
#     inst_<etape>_done     deja fait ?                     0 oui · 1 non
#     inst_<etape>_run      fait le travail                 0 ok · 1 echec · 3 deja fait
#     inst_<etape>_verify   controle APRES coup             0 ok · 1 echec
# -----------------------------------------------------------------------------
readonly INST_RC_OK=0 INST_RC_FAIL=1 INST_RC_DONE=3 INST_RC_NA=4
declare -a INST_ORDER=()
declare -A INST_DESC=()
INST_REGISTER() { INST_ORDER+=("$1"); INST_DESC[$1]="$2"; }

_INST_REASON=""
_INST_HINT=""
inst_ok()   { _INST_REASON=""   ; return $INST_RC_OK; }
inst_fail() { _INST_REASON="$*" ; return $INST_RC_FAIL; }
inst_done() { _INST_REASON="$*" ; return $INST_RC_DONE; }
inst_na()   { _INST_REASON="$*" ; return $INST_RC_NA; }
inst_hint() { _INST_HINT="$*"; }
inst_say()  { printf '%s\n' "$*"; }

have_cmd()  { command -v "$1" >/dev/null 2>&1; }
have_file() { [ -f "$1" ]; }
have_pkg()  { dpkg -s "$1" >/dev/null 2>&1; }
# Un chemin est inscriptible s il l est, ou si son plus proche parent existant
# l est (on pourra le creer).
ecrivable() { local d="$1"; while [ ! -e "$d" ] && [ "$d" != / ]; do d="$(dirname "$d")"; done; [ -w "$d" ]; }
# Chemin absolu sans exiger qu il existe (configure, make -C et les --dest
# changent de dossier : un chemin relatif n y voudrait plus rien dire).
absolu() { case "$1" in /*) printf '%s\n' "$1" ;; *) printf '%s\n' "$PWD/$1" ;; esac; }

# exige "ce qu il faut" "conseil si absent" commande... : une condition de
# prerequis. En --check elle est imprimee (ok / MANQUE + conseil) ; sinon elle
# est memorisee, et la premiere qui manque donne la raison de l echec.
_MANQUE=()
exige() {
    local quoi="$1" conseil="$2"; shift 2
    if "$@" >/dev/null 2>&1; then
        [ "$ACTION" = check ] && printf '      ok      %s\n' "$quoi"
        return 0
    fi
    if [ "$ACTION" = check ]; then
        printf '      %sMANQUE%s  %s\n' "$C_KO" "$C_Z" "$quoi"
        [ -n "$conseil" ] && printf '              → %s\n' "$conseil"
    fi
    [ ${#_MANQUE[@]} -eq 0 ] && [ -n "$conseil" ] && inst_hint "$conseil"
    _MANQUE+=("$quoi")
    return 1
}
# Verdict d une fonction check qui a appele exige.
bilan_exige() {
    [ ${#_MANQUE[@]} -eq 0 ] && { inst_ok; return $INST_RC_OK; }
    local plus=""; [ ${#_MANQUE[@]} -gt 1 ] && plus=" (+$(( ${#_MANQUE[@]} - 1 )) autre(s), --check)"
    inst_fail "manque : ${_MANQUE[0]}$plus"
}
# ═════════════════════════════════════════════════════════════════════════════
#  LE COMPOSANT - qosmo
# ═════════════════════════════════════════════════════════════════════════════
COMPOSANT=qosmo
: "${GSM_ROOT:=/opt/GSM}"
: "${QOSMO:=$ICI}"
: "${PREFIX:=$GSM_ROOT/qemu-install}"
: "${BUILD_DIR:=}"
: "${VENV:=${HOME:-/root}/.venv-qemu}"
: "${BIN_DIR:=/usr/local/bin}"
: "${DESTDIR:=}"
: "${QOSMO_CC:=}"
: "${JOBS:=$(nproc 2>/dev/null || echo 2)}"
: "${LOG_DIR:=/tmp/qosmo-install}"

# Paquets apt, repris de la liste unique d osmo-operator/Dockerfile :
#   :159      build-essential git pkg-config        outils de build
#   :166      python3
#   :189      python3-venv python3-pip              le venv de configure (tomli)
#   :190      libglib2.0-dev libpixman-1-dev libslirp-dev ninja-build
#                                                   la cible arm-softmmu, meson
#   :190      socat                                 run.sh / 40-qemu.sh parlent au moniteur QEMU
PAQUETS=(build-essential git pkg-config python3 python3-venv python3-pip
         libglib2.0-dev libpixman-1-dev libslirp-dev ninja-build socat)

options_composant() {
    case "$1" in
        --prefix)    _val "$@"; PREFIX="$2";    _n=2 ;;
        --build-dir) _val "$@"; BUILD_DIR="$2"; _n=2 ;;
        --venv)      _val "$@"; VENV="$2";      _n=2 ;;
        --bin-dir)   _val "$@"; BIN_DIR="$2";   _n=2 ;;
        --destdir)   _val "$@"; DESTDIR="$2";   _n=2 ;;
        --cc)        _val "$@"; QOSMO_CC="$2";  _n=2 ;;
        -j)          _val "$@"; JOBS="$2";      _n=2 ;;
        -j[0-9]*)    JOBS="${1#-j}";            _n=1 ;;
    esac
}

apres_options() {
    QOSMO="$(absolu "$QOSMO")"
    [ -n "$BUILD_DIR" ] || BUILD_DIR="$QOSMO/build"
    BUILD_DIR="$(absolu "$BUILD_DIR")"; PREFIX="$(absolu "$PREFIX")"
    [ "$VENV" = none ]    || VENV="$(absolu "$VENV")"
    [ "$BIN_DIR" = none ] || BIN_DIR="$(absolu "$BIN_DIR")"
    [ -z "$DESTDIR" ]     || DESTDIR="$(absolu "$DESTDIR")"
    # Dockerfile:698-699 - les options de configure, a l identique.
    CONFIGURE_ARGS=(--target-list=arm-softmmu --enable-l1-grgsm "--prefix=$PREFIX"
                    --disable-werror --disable-docs)
}

resume_composant() {
    printf 'QOSMO=%s\nBUILD_DIR=%s\nPREFIX=%s%s\nVENV=%s\nBIN_DIR=%s\nJOBS=%s%s\n' \
        "$QOSMO" "$BUILD_DIR" "$PREFIX" "${DESTDIR:+   (sous DESTDIR=$DESTDIR)}" "$VENV" "$BIN_DIR" \
        "$JOBS" "${QOSMO_CC:+   QOSMO_CC=$QOSMO_CC}"
    printf 'configure %s\n' "${CONFIGURE_ARGS[*]}"
}

# --cc : le PATH local de Dockerfile.run:183-185. build/ garde le compilateur
# du configure (meson memorise « cc ») ; si ce cc n est plus le meme, ninja
# echoue des la premiere unite. On place donc devant le PATH un dossier ou
# cc/gcc (et c++/g++) designent le compilateur demande.
_CCDIR=""
avant_etapes() {
    [ -n "$QOSMO_CC" ] || return 0
    local cc cxx
    cc="$(command -v "$QOSMO_CC")" || { printf -- '--cc %s : introuvable\n' "$QOSMO_CC" >&2; exit 1; }
    cxx="$(command -v "${QOSMO_CC/gcc/g++}" 2>/dev/null || true)"
    _CCDIR="$(mktemp -d)" || exit 1
    trap 'rm -rf "$_CCDIR"' EXIT
    ln -s "$cc" "$_CCDIR/cc"; ln -s "$cc" "$_CCDIR/gcc"
    [ -n "$cxx" ] && { ln -s "$cxx" "$_CCDIR/c++"; ln -s "$cxx" "$_CCDIR/g++"; }
    return 0
}
# avec_env COMMANDE... : le venv active (Dockerfile:695) et le --cc eventuel.
avec_env() {
    local p="$PATH"
    [ "$VENV" != none ] && [ -d "$VENV/bin" ] && p="$VENV/bin:$p"
    [ -n "$_CCDIR" ] && p="$_CCDIR:$p"
    if [ "$VENV" != none ] && [ -d "$VENV/bin" ]; then
        PATH="$p" VIRTUAL_ENV="$VENV" "$@"
    else
        PATH="$p" "$@"
    fi
}

apres_etapes() {
    [ -x "$BUILD_DIR/qemu-system-arm" ] && printf '  qemu-system-arm : %s\n' "$BUILD_DIR/qemu-system-arm"
    printf '  ensuite         : c54x_exe/install.sh (le DSP) - wiki osmo-operator, Telephone-emule\n'
}

# ── venv : python de configure ────────────────────────────────────────────────
INST_REGISTER venv "Venv python de configure (tomli)"
inst_venv_done() { [ "$VENV" != none ] && "$VENV/bin/python3" -c 'import tomli' 2>/dev/null; }
inst_venv_check() {
    [ "$VENV" = none ] && { inst_na "VENV=none : python du systeme"; return $INST_RC_NA; }
    _MANQUE=()
    exige "python3" "apt-get install python3" have_cmd python3
    exige "module venv + ensurepip (python3-venv)" "apt-get install python3-venv" python3 -c 'import venv, ensurepip'
    exige "$VENV inscriptible" "--venv DIR vers un dossier a vous, ou --venv none" ecrivable "$VENV"
    bilan_exige
}
inst_venv_run() {
    # Dockerfile:694-696 : python3 -m venv /root/.venv-qemu, activate, pip install tomli
    python3 -m venv "$VENV" \
        && "$VENV/bin/pip" install --no-cache-dir tomli \
        || { inst_hint "pip a besoin du reseau (PyPI)"; inst_fail "venv $VENV ou pip install tomli en echec"; return $INST_RC_FAIL; }
    inst_ok
}
inst_venv_verify() { inst_venv_done && inst_ok || inst_fail "tomli introuvable dans $VENV"; }

# ── configure ─────────────────────────────────────────────────────────────────
INST_REGISTER configure "configure --target-list=arm-softmmu --enable-l1-grgsm"
# config.log garde la ligne d appel : « # Configured with: '../configure' '--...' ».
# Deja fait = arbre configure AVEC les memes options (le 1er mot, le chemin de
# configure, ne compte pas).
_cfg_attendu() { printf "'%s' " "${CONFIGURE_ARGS[@]}" | sed 's/ $//'; }
_cfg_actuel()  { sed -n "s/^# Configured with: '[^']*' //p" "$BUILD_DIR/config.log" 2>/dev/null | head -1; }
inst_configure_done() { [ -f "$BUILD_DIR/build.ninja" ] && [ "$(_cfg_actuel)" = "$(_cfg_attendu)" ]; }
inst_configure_check() {
    _MANQUE=()
    exige "sources qosmo ($QOSMO/configure, hw/arm/calypso/)" "QOSMO=/chemin/vers/qosmo, ou git clone https://github.com/bbaranoff/qosmO" \
        test -x "$QOSMO/configure" -a -d "$QOSMO/hw/arm/calypso"
    exige "compilateur C (${QOSMO_CC:-cc})" "apt-get install build-essential" have_cmd "${QOSMO_CC:-cc}"
    exige "make" "apt-get install build-essential" have_cmd make
    exige "ninja (ninja-build)" "apt-get install ninja-build" have_cmd ninja
    exige "pkg-config" "apt-get install pkg-config" have_cmd pkg-config
    exige "glib-2.0 vu de pkg-config (libglib2.0-dev)" "apt-get install libglib2.0-dev" pkg-config --exists glib-2.0
    exige "pixman-1 vu de pkg-config (libpixman-1-dev)" "apt-get install libpixman-1-dev" pkg-config --exists pixman-1
    exige "$BUILD_DIR inscriptible" "--build-dir DIR, ou les droits sur $QOSMO" ecrivable "$BUILD_DIR"
    # configure lance DEPUIS l arbre source cree ./build et EFFACE un build/ qu il
    # a cree lui-meme (configure:16-45) : jamais de build dans l arbre source.
    exige "BUILD_DIR distinct de l arbre source" "--build-dir $QOSMO/build" test "$BUILD_DIR" != "$QOSMO"
    bilan_exige
}
inst_configure_run() {
    # Dockerfile:697-699 : mkdir -p build && cd build && ../configure <options>
    local conf="$QOSMO/configure"
    [ "$BUILD_DIR" = "$QOSMO/build" ] && conf=../configure
    mkdir -p "$BUILD_DIR" || { inst_fail "impossible de creer $BUILD_DIR"; return $INST_RC_FAIL; }
    ( cd "$BUILD_DIR" && avec_env "$conf" "${CONFIGURE_ARGS[@]}" ) \
        || { inst_hint "$BUILD_DIR/config.log et $BUILD_DIR/meson-logs/"; inst_fail "configure en echec"; return $INST_RC_FAIL; }
    inst_ok
}
inst_configure_verify() {
    [ -f "$BUILD_DIR/build.ninja" ] || { inst_fail "$BUILD_DIR/build.ninja absent apres configure"; return $INST_RC_FAIL; }
    grep -q '^TARGET_DIRS=arm-softmmu' "$BUILD_DIR/config-host.mak" 2>/dev/null \
        || { inst_fail "config-host.mak ne vise pas arm-softmmu"; return $INST_RC_FAIL; }
    inst_ok
}

# ── build ─────────────────────────────────────────────────────────────────────
# Pas de « deja fait » : make est incremental, il ne refait que ce qui a bouge.
INST_REGISTER build "Compilation (make)"
inst_build_check() {
    _MANQUE=()
    etape_retenue configure || exige "arbre configure ($BUILD_DIR/build.ninja)" "./install.sh --only configure" \
        test -f "$BUILD_DIR/build.ninja"
    exige "make" "apt-get install build-essential" have_cmd make
    exige "ninja (ninja-build)" "apt-get install ninja-build" have_cmd ninja
    [ -n "$QOSMO_CC" ] && exige "compilateur $QOSMO_CC (--cc)" "" have_cmd "$QOSMO_CC"
    exige "$BUILD_DIR inscriptible" "les droits sur $BUILD_DIR" ecrivable "$BUILD_DIR"
    bilan_exige
}
inst_build_run() {
    # Dockerfile:700 : make -j$(nproc) dans build/ (le Makefile de QEMU mene ninja).
    # Dockerfile.run:185 faisait `ninja` avec le PATH --cc : meme cible.
    avec_env make -C "$BUILD_DIR" -j"$JOBS" || { inst_fail "echec de compilation"; return $INST_RC_FAIL; }
    inst_ok
}
inst_build_verify() {
    [ -x "$BUILD_DIR/qemu-system-arm" ] && inst_ok || inst_fail "$BUILD_DIR/qemu-system-arm absent apres make"
}

# ── install : make install dans le prefixe ───────────────────────────────────
INST_REGISTER install "make install (prefixe, lanceur qosmo compris)"
inst_install_done() {
    cmp -s "$BUILD_DIR/qemu-system-arm" "$DESTDIR$PREFIX/bin/qemu-system-arm" \
        && [ -x "$DESTDIR$PREFIX/bin/qosmo" ]
}
inst_install_check() {
    _MANQUE=()
    etape_retenue configure || exige "arbre configure ($BUILD_DIR/build.ninja)" "./install.sh --only configure" \
        test -f "$BUILD_DIR/build.ninja"
    exige "$DESTDIR$PREFIX inscriptible" "--prefix DIR, ou sudo" ecrivable "$DESTDIR$PREFIX"
    bilan_exige
}
inst_install_run() {
    # Dockerfile:701 : make install (meson install ; DESTDIR respecte, cf. ISO)
    if [ -n "$DESTDIR" ]; then
        avec_env env DESTDIR="$DESTDIR" make -C "$BUILD_DIR" install
    else
        avec_env make -C "$BUILD_DIR" install
    fi || { inst_fail "make install en echec"; return $INST_RC_FAIL; }
    inst_ok
}
inst_install_verify() {
    [ -x "$DESTDIR$PREFIX/bin/qemu-system-arm" ] || { inst_fail "$DESTDIR$PREFIX/bin/qemu-system-arm absent"; return $INST_RC_FAIL; }
    [ -x "$DESTDIR$PREFIX/bin/qosmo" ] || { inst_fail "lanceur $DESTDIR$PREFIX/bin/qosmo absent (tools/qosmo/meson.build)"; return $INST_RC_FAIL; }
    inst_ok
}

# ── bin : copie dans le PATH ──────────────────────────────────────────────────
INST_REGISTER bin "qemu-system-arm et qosmo dans le PATH"
inst_bin_done() {
    [ "$BIN_DIR" != none ] || return 1
    local b; for b in qemu-system-arm qosmo; do
        cmp -s "$DESTDIR$PREFIX/bin/$b" "$DESTDIR$BIN_DIR/$b" || return 1
    done
}
inst_bin_check() {
    [ "$BIN_DIR" = none ] && { inst_na "--bin-dir none"; return $INST_RC_NA; }
    ecrivable "$DESTDIR$BIN_DIR" \
        || { inst_na "$DESTDIR$BIN_DIR non inscriptible : binaires laisses dans $PREFIX/bin (sudo, ou --bin-dir DIR)"; return $INST_RC_NA; }
    [ "$ACTION" = check ] && printf '      ok      %s inscriptible\n' "$DESTDIR$BIN_DIR"
    inst_ok
}
inst_bin_run() {
    # Dockerfile:702-703 : cp qemu-install/bin/{qemu-system-arm,qosmo} /usr/local/bin/
    # --remove-destination : un qemu-system-arm en cours d execution garde son
    # inode, la copie ne bute pas sur « Text file busy ».
    local b
    mkdir -p "$DESTDIR$BIN_DIR" || { inst_fail "impossible de creer $DESTDIR$BIN_DIR"; return $INST_RC_FAIL; }
    for b in qemu-system-arm qosmo; do
        [ -x "$DESTDIR$PREFIX/bin/$b" ] || { inst_fail "$DESTDIR$PREFIX/bin/$b absent (etape install)"; return $INST_RC_FAIL; }
        cp --remove-destination "$DESTDIR$PREFIX/bin/$b" "$DESTDIR$BIN_DIR/$b" \
            || { inst_fail "copie de $b vers $DESTDIR$BIN_DIR impossible"; return $INST_RC_FAIL; }
        inst_say "$DESTDIR$BIN_DIR/$b"
    done
    inst_ok
}
inst_bin_verify() { inst_bin_done && inst_ok || inst_fail "$DESTDIR$BIN_DIR differe de $DESTDIR$PREFIX/bin"; }

# ── verify : la machine calypso, avec la couche 1 gr-gsm ──────────────────────
# Le meme binaire sert aux deux montages : compile --enable-l1-grgsm, lance avec
# CALYPSO_DSP_EXTERN=1 pour le DSP (Dockerfile:679-682). Sans la couche 1
# gr-gsm, la description de la machine le dit (hw/arm/calypso/calypso_mb.c).
INST_REGISTER verify "Controle : -M help liste calypso (couche 1 : grgsm)"
inst_verify_run() {
    local q="$BUILD_DIR/qemu-system-arm" l
    [ -x "$q" ] || { inst_hint "./install.sh --only build"; inst_fail "$q absent"; return $INST_RC_FAIL; }
    l="$(timeout 30 "$q" -M help 2>/dev/null | grep -E '^calypso[[:space:]]')"
    inst_say "$q -M help : ${l:-<pas de machine calypso>}"
    [ -n "$l" ] || { inst_hint "le configure a-t-il garde --enable-l1-grgsm ? ($BUILD_DIR/config.log)"
                     inst_fail "machine calypso absente de -M help"; return $INST_RC_FAIL; }
    case "$l" in
        *"couche 1 : grgsm"*) ;;
        *) inst_fail "machine calypso sans la couche 1 gr-gsm : $l"; return $INST_RC_FAIL ;;
    esac
    if [ -z "$DESTDIR" ] && [ -x "$PREFIX/bin/qosmo" ]; then
        inst_say "lanceur : $("$PREFIX/bin/qosmo" -V 2>&1 | head -1)"
    fi
    inst_ok
}
# ═════════════════════════════════════════════════════════════════════════════
#  LE MOTEUR - copie IDENTIQUE dans les trois depots (voir LE CONTRAT)
# ═════════════════════════════════════════════════════════════════════════════
#  Meme deroule que osmo-operator/install.sh : pour chaque etape retenue,
#  « deja fait ? » (sauf --reinstall), prerequis, travail, controle. Le premier
#  echec arrete tout, dit pourquoi et montre la fin du journal de l etape.
#  Le composant fournit : COMPOSANT, PAQUETS, LOG_DIR, les etapes, et les
#  crochets options_composant / apres_options / resume_composant /
#  avant_etapes / apres_etapes.
# -----------------------------------------------------------------------------
ACTION=install ONLY="" SKIP="" WITH_DEPS=0
: "${REINSTALL:=0}" "${VERBOSE:=0}"
_ARGS_ORIG=("$@")
# _val OPTION [VALEUR] : une option qui attend une valeur doit l avoir.
_val() { [ $# -ge 2 ] && [ -n "$2" ] || { printf '%s : valeur attendue   (--help)\n' "$1" >&2; exit 2; }; }
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)    ACTION=help ;;
        --check)      ACTION=check ;;
        --list)       ACTION=list ;;
        --deps)       ACTION=deps ;;
        --print-deps) ACTION=print-deps ;;
        --with-deps)  WITH_DEPS=1 ;;
        --only)       _val "$@"; ONLY="$2"; shift ;;
        --skip)       _val "$@"; SKIP="$2"; shift ;;
        --reinstall)  REINSTALL=1 ;;
        -v|--verbose) VERBOSE=1 ;;
        *)  _n=0; options_composant "$@"
            [ "$_n" -gt 0 ] || { printf 'option inconnue : %s   (--help)\n' "$1" >&2; exit 2; }
            shift $((_n - 1)) ;;
    esac
    shift
done

if [ "$ACTION" = help ]; then
    awk 'NR > 2 && /^# -{10}/ { exit } NR > 2 && !/^# =+$/ { sub(/^# ?/, ""); print }' "$0"
    exit 0
fi
apres_options

if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
    TTY=1; C_OK=$'\033[32m'; C_KO=$'\033[31m'; C_SK=$'\033[33m'; C_DIM=$'\033[2m'; C_Z=$'\033[0m'
else
    TTY=0; C_OK=""; C_KO=""; C_SK=""; C_DIM=""; C_Z=""
fi
begin() { if [ $TTY -eq 1 ] && [ "$VERBOSE" != 1 ]; then printf '[ %s..%s ] %s' "$C_DIM" "$C_Z" "$1"; else printf '[ .. ] %s\n' "$1"; fi; }
end()   { [ $TTY -eq 1 ] && [ "$VERBOSE" != 1 ] && printf '\r\033[K'
          printf '[%s%s%s] %s' "$2" "$1" "$C_Z" "$3"
          [ -n "${4:-}" ] && printf ' %s(%s)%s' "$C_DIM" "$4" "$C_Z"; printf '\n'; }

# --- selection des etapes (--only / --skip, listes a virgules) ---------------
in_csv() { case ",$1," in *",$2,"*) return 0;; esac; return 1; }
for _s in ${ONLY//,/ } ${SKIP//,/ }; do
    [ -n "${INST_DESC[$_s]+x}" ] || { printf 'etape inconnue : %s   (etapes : %s)\n' "$_s" "${INST_ORDER[*]}" >&2; exit 2; }
done
SELECTED=()
for _s in "${INST_ORDER[@]}"; do
    [ -n "$ONLY" ] && ! in_csv "$ONLY" "$_s" && continue
    [ -n "$SKIP" ] && in_csv "$SKIP" "$_s" && continue
    SELECTED+=("$_s")
done
# Une etape qui produit ce qu une autre exige : en --check, on ne reproche pas
# a la seconde l absence de ce que la premiere va justement fabriquer.
etape_retenue() { in_csv "$(IFS=,; echo "${SELECTED[*]}")" "$1"; }

# --- paquets apt --------------------------------------------------------------
# rend la liste des absents ; code 2 sans dpkg (pas une Debian/Ubuntu)
paquets_absents() {
    have_cmd dpkg || return 2
    local p; for p in "${PAQUETS[@]}"; do have_pkg "$p" || printf '%s ' "$p"; done
    return 0
}
installer_paquets() {
    local manque rc
    manque="$(paquets_absents)"; rc=$?
    if [ $rc -eq 2 ]; then
        printf 'dpkg absent : installez l equivalent de : %s\n' "${PAQUETS[*]}" >&2; return 1
    fi
    if [ -z "${manque// }" ]; then
        printf 'paquets apt : les %d sont presents\n' "${#PAQUETS[@]}"; return 0
    fi
    if [ "$(id -u)" -ne 0 ]; then
        printf 'paquets apt manquants : %s\n  sudo apt-get install -y --no-install-recommends %s\n' "$manque" "$manque" >&2
        return 1
    fi
    printf 'apt-get install : %s\n' "$manque"
    apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $manque || return 1
    manque="$(paquets_absents)"
    [ -z "${manque// }" ] || { printf 'toujours absents apres apt-get : %s\n' "$manque" >&2; return 1; }
}

case "$ACTION" in
    print-deps) printf '%s\n' "${PAQUETS[*]}"; exit 0 ;;
    deps)       installer_paquets; exit $? ;;
    list)
        printf '%s - %d etape(s), dans cet ordre\n\n' "$COMPOSANT" "${#SELECTED[@]}"
        for _s in "${SELECTED[@]}"; do printf '  %-10s %s\n' "$_s" "${INST_DESC[$_s]}"; done
        printf '\n'; resume_composant | sed 's/^/  /'
        exit 0 ;;
    check)
        printf '%s : prerequis et etat (rien n est modifie)\n\n' "$COMPOSANT"
        resume_composant | sed 's/^/  /'
        _manque="$(paquets_absents)"; _rc=$?
        if [ $_rc -eq 2 ]; then printf '\n  paquets apt : dpkg absent, non verifies (%s)\n' "${PAQUETS[*]}"
        elif [ -n "${_manque// }" ]; then printf '\n  paquets apt absents (installes autrement ? sinon --deps) : %s\n' "$_manque"
        else printf '\n  paquets apt : les %d sont presents\n' "${#PAQUETS[@]}"; fi
        _rc=0
        for _s in "${SELECTED[@]}"; do
            _p="inst_${_s//-/_}"
            if ! declare -F "${_p}_done" >/dev/null; then _etat="pas de controle"
            elif "${_p}_done" >/dev/null 2>&1; then _etat="deja fait"
            else _etat="a faire"; fi
            printf '\n  %-10s %s  %s[%s]%s\n' "$_s" "${INST_DESC[$_s]}" "$C_DIM" "$_etat" "$C_Z"
            declare -F "${_p}_check" >/dev/null || continue
            _INST_REASON=""; _INST_HINT=""; _MANQUE=()
            "${_p}_check"; _r=$?
            case $_r in
                "$INST_RC_FAIL") _rc=1 ;;
                "$INST_RC_NA")   printf '      sans objet : %s\n' "$_INST_REASON" ;;
            esac
        done
        printf '\n'
        if [ $_rc -eq 0 ]; then printf '%sprerequis satisfaits%s\n' "$C_OK" "$C_Z"
        else printf '%sprerequis manquants%s (voir MANQUE ci-dessus)\n' "$C_KO" "$C_Z"; fi
        exit $_rc ;;
esac

# --- installation ---------------------------------------------------------------
if [ "$WITH_DEPS" = 1 ]; then
    installer_paquets || { printf 'paquets apt : echec - rien n a ete construit\n' >&2; exit 1; }
else
    _manque="$(paquets_absents 2>/dev/null)"
    [ -n "${_manque// }" ] && printf '%snote : paquets apt absents (installes autrement ? sinon --with-deps) : %s%s\n' "$C_DIM" "$_manque" "$C_Z"
fi
if [ "$VERBOSE" != 1 ]; then
    mkdir -p "$LOG_DIR" || { printf 'journal impossible : %s (LOG_DIR=...)\n' "$LOG_DIR" >&2; exit 1; }
fi
# _jouer FONCTION : la sortie va au journal de l etape, ou a l ecran en -v.
# Pas de tube : la fonction tourne dans CE shell (sa raison d echec survit).
_jouer() { if [ "$VERBOSE" = 1 ]; then "$1"; else "$1" >>"$_log" 2>&1; fi; }
_echec() {   # _echec ETAPE RAISON [journal] : dit tout ce qu on sait, et s arrete
    end FAIL "$C_KO" "${INST_DESC[$1]}" "$2"
    [ -n "$_INST_HINT" ] && printf '       → %s\n' "$_INST_HINT"
    # la fin du journal, quand c est le travail (run, verify) qui a echoue
    if [ -n "${3:-}" ] && [ "$VERBOSE" != 1 ] && [ -s "$_log" ]; then
        printf '       %sjournal : %s (fin ci-dessous)%s\n' "$C_DIM" "$_log" "$C_Z"
        tail -n 25 "$_log" | sed 's/^/       | /'
    fi
    printf '\n%s : installation interrompue a l etape %s\n' "$COMPOSANT" "$1"
    exit 1
}
avant_etapes
_nb_ok=0; _nb_skip=0
for _s in "${SELECTED[@]}"; do
    _p="inst_${_s//-/_}"; _log="$LOG_DIR/$_s.log"
    _INST_REASON=""; _INST_HINT=""; _MANQUE=()
    [ "$VERBOSE" = 1 ] || printf '\n===== %s  %s =====\n' "$(date '+%F %T')" "$_s" >>"$_log"
    begin "${INST_DESC[$_s]}"
    if [ "$REINSTALL" != 1 ] && declare -F "${_p}_done" >/dev/null && _jouer "${_p}_done"; then
        end SKIP "$C_SK" "${INST_DESC[$_s]}" "deja fait"; _nb_skip=$((_nb_skip + 1)); continue
    fi
    if declare -F "${_p}_check" >/dev/null; then
        _jouer "${_p}_check"; _r=$?
        case $_r in
            "$INST_RC_OK")   ;;
            "$INST_RC_NA")   end SKIP "$C_SK" "${INST_DESC[$_s]}" "${_INST_REASON:-sans objet}"
                             _nb_skip=$((_nb_skip + 1)); continue ;;
            *)               _echec "$_s" "${_INST_REASON:-prerequis non satisfait}" ;;
        esac
    fi
    _jouer "${_p}_run"; _r=$?
    case $_r in
        "$INST_RC_OK")   ;;
        "$INST_RC_DONE"|"$INST_RC_NA")
                         end SKIP "$C_SK" "${INST_DESC[$_s]}" "${_INST_REASON:-rien a faire}"
                         _nb_skip=$((_nb_skip + 1)); continue ;;
        *)               _echec "$_s" "${_INST_REASON:-echec}" journal ;;
    esac
    # installer n est pas avoir installe : on controle apres coup
    if declare -F "${_p}_verify" >/dev/null && ! _jouer "${_p}_verify"; then
        _echec "$_s" "faite, mais le controle echoue : ${_INST_REASON:-}" journal
    fi
    end " OK " "$C_OK" "${INST_DESC[$_s]}" "${_INST_REASON:-}"; _nb_ok=$((_nb_ok + 1))
done
printf '\n%s : %d ok · %d ignoree(s) · 0 echec\n' "$COMPOSANT" "$_nb_ok" "$_nb_skip"
apres_etapes
exit 0
