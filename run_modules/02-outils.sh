MOD_REGISTER outils "Outillage de l'hôte"
MOD_REQUIRED[outils]=1
MOD_PURE[outils]=1
MOD_TIMEOUT[outils]=10

: "${OUTILS_REQUIS:=bash sleep pgrep pkill kill stat grep sed awk ip mkfifo rm}"

if ! declare -p OUTILS_OPTIONNELS >/dev/null 2>&1; then
    OUTILS_OPTIONNELS=(
        "ss|sonde des ports d'écoute (repli : /dev/tcp, sans nom de tenant)"
        "tmux|fenêtres de la pile et épilogue « se connecter »"
        "setsid|détachement des processus lancés en arrière-plan"
        "flock|verrou d'unicité du démon audio"
        "socat|passerelles série et sockets de test"
        "nc|alimentation du HLR (repli telnet : fermeture différente, échec silencieux)"
        "python3|outils d'analyse, gr-gsm, transceiver de test"
    )
fi

mod_outils_check() {
    local c manquants=""
    for c in $OUTILS_REQUIS; do
        command -v "$c" >/dev/null 2>&1 || manquants="$manquants $c"
    done
    if [ -n "$manquants" ]; then
        mod_hint "installez-les :  apt-get install -y coreutils procps iproute2 grep sed gawk"
        mod_fail "commandes indispensables absentes du PATH :$manquants"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_outils_status() { return $MOD_RC_FAIL; }

mod_outils_start() {
    local c n d
    for c in $OUTILS_REQUIS; do
        mod_say "requis     $c -> $(command -v "$c" 2>/dev/null)"
    done
    for c in "${OUTILS_OPTIONNELS[@]}"; do
        n="${c%%|*}"; d="${c#*|}"
        if command -v "$n" >/dev/null 2>&1; then
            mod_say "optionnel  $n -> $(command -v "$n")"
        else
            mod_say "ABSENT     $n — dégrade : $d"
        fi
    done
    mod_ok
}

_outils_enfant_resout() {
    bash -c 'command -v ip >/dev/null 2>&1 && command -v pgrep >/dev/null 2>&1 && command -v stat >/dev/null 2>&1'
}

mod_outils_wait() {
    if ! wait_until "${MOD_TIMEOUT[outils]}" "outillage visible d'un enfant" _outils_enfant_resout; then
        mod_hint "PATH courant : $PATH — exportez-le, ou donnez les chemins absolus dans environnement/paths.env"
        mod_fail "un shell enfant ne résout pas l'outillage (PATH non hérité)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}
