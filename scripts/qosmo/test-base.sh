#!/bin/bash
# Le test de ce que qosmo EST : une plateforme qui se construit sans couche 1,
# et qui dit laquelle elle porte.
#
# [2026-09-16] Il remplace les 19 fichiers de test herites de qemu-calypso, qui
# visaient tous soit l'orchestration (run.sh, /tmp/calypso), soit le DSP (c54x,
# calypso_dsp.txt) - deux choses absentes de ce depot. Une suite dont le sujet
# n'est pas dans l'arbre ne teste rien, elle rassure.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
RACINE="$PWD"
B="${QOSMO_BUILD:-$RACINE/build}"
QEMU="$B/qemu-system-arm"
rouge=$'\033[31m'; vert=$'\033[32m'; nc=$'\033[0m'
ok=0; ko=0

verdict() { # <attendu> <obtenu> <intitule>
    if [ "$2" = "$1" ]; then printf '%s  ✓%s %s\n' "$vert" "$nc" "$3"; ok=$((ok+1))
    else printf '%s  ✗%s %s\n     attendu : %s\n     obtenu  : %s\n' "$rouge" "$nc" "$3" "$1" "$2"; ko=$((ko+1)); fi
}

[ -x "$QEMU" ] || { echo "qemu-system-arm introuvable dans $B - construire d'abord"; exit 2; }

# 1. la machine existe
verdict "oui" "$("$QEMU" -M help 2>/dev/null | grep -q '^calypso ' && echo oui || echo non)" \
        "la machine 'calypso' est enregistree"

# 2. elle nomme la couche 1 reellement liee. C'est le seul moyen court de
#    savoir ce qu'on a construit ; une L1 qui s'invite dans le build se voit ici.
l1="$("$QEMU" -M help 2>/dev/null | sed -n 's/.*couche 1 : \([a-z0-9]*\)).*/\1/p')"
attendu="${QOSMO_L1_ATTENDUE:-aucune}"
verdict "$attendu" "${l1:-<absent>}" "la machine annonce sa couche 1 (QOSMO_L1_ATTENDUE=$attendu)"

# 3. sans L1, aucun symbole de couche 1 ne doit avoir ete lie
if [ "$attendu" = "aucune" ]; then
    fuites="$(nm -C "$QEMU" 2>/dev/null | grep -cE ' [Tt] (calypso_l1_grgsm|calypso_c54x|c54x_)' || true)"
    verdict "0" "${fuites:-0}" "aucun symbole de couche 1 lie dans la base nue"
fi

# 4. le binaire demarre et tient la seconde (pas de segfault au realize)
timeout 3 "$QEMU" -M calypso -nographic -serial null -monitor none >/dev/null 2>&1
verdict "124" "$?" "la machine demarre et tourne (124 = toujours vivante au timeout)"

echo
if [ "$ko" -gt 0 ]; then printf '%s%d echec(s), %d succes%s\n' "$rouge" "$ko" "$ok" "$nc"; exit 1; fi
printf '%s%d succes%s\n' "$vert" "$ok" "$nc"
