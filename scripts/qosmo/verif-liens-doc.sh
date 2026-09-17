#!/bin/bash
# Verifie que tout chemin local cite dans un .md du depot existe reellement.
#
# [2026-09-16] Ce script existe a cause d'un cas precis. Apres le split de
# qemu-calypso, les README de qosmo-grgsm et qosmo-dsp sont restes IDENTIQUES
# (meme md5) et ont continue a renvoyer vers hw/arm/calypso/doc/ETAT_ACTUEL.md
# en le presentant comme « la source de verite, en cas de conflit c'est lui qui
# prime » - fichier qui n'existait plus dans l'un des deux. Meme histoire pour
# calypso_dsp.txt, parti avec le C54x, encore symlinke par le Dockerfile
# d'osmo-operator « pour les outils d'analyse » qui avaient migre avec lui.
#
# Aucune de ces deux ruptures n'a fait de bruit. Une doc qui pointe dans le vide
# ne plante pas : elle ment, et elle ment d'autant mieux qu'elle est detaillee.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

rouge=$'\033[31m'; vert=$'\033[32m'; nc=$'\033[0m'
casses=0; verifies=0

while IFS= read -r md; do
    # 1. les cibles de liens markdown [texte](chemin)
    # 2. les chemins entre backticks qui ressemblent a des fichiers du depot
    {
        grep -oE '\]\([^)#][^)]*\)' "$md" | sed -e 's/^](//' -e 's/)$//'
        # On retire d'abord les liens markdown complets : un nom entre backticks
        # qui sert de LIBELLE ([`x.c`](url)) est deja documente par sa cible,
        # traitee au-dessus. Sans ca, une reference inter-depot legitime est
        # signalee comme cassee.
        sed -E 's/\[[^]]*\]\([^)]*\)//g' "$md" \
          | grep -oE '`[a-zA-Z0-9_./-]+\.(c|h|sh|py|md|txt|cfg|conf|build|yml)`' | tr -d '`'
    } 2>/dev/null | sort -u | while IFS= read -r cible; do
        case "$cible" in
            http://*|https://*|mailto:*|'') continue ;;
        esac
        cible="${cible%%#*}"
        [ -z "$cible" ] && continue
        # relatif au .md, puis a la racine du depot
        base="$(dirname "$md")"
        # Un nom nu entre backticks (« calypso_trx.c ») est une MENTION, pas un
        # chemin : il suffit qu'un fichier de ce nom existe quelque part dans le
        # depot. Un chemin explicite (« hw/arm/.../x.c », ou la cible d'un lien
        # markdown) doit exister LA ou il est ecrit - c'est le cas qui a laisse
        # passer ETAT_ACTUEL.md.
        if [ -e "$base/$cible" ] || [ -e "$cible" ]; then
            echo "OK"
        elif [ "${cible%/*}" = "$cible" ] && \
             [ -n "$(find . -name "$cible" -not -path './build*' -print -quit 2>/dev/null)" ]; then
            echo "OK"
        else
            printf '%s  %s:%s cible introuvable : %s%s\n' \
                   "$rouge" "$md" "$(grep -n -- "$cible" "$md" | head -1 | cut -d: -f1)" "$cible" "$nc" >&2
            echo "KO"
        fi
    done
done < <(git ls-files '*.md' 2>/dev/null || find . -name '*.md' -not -path './build/*') > /tmp/.qosmo-liens.$$

# grep -c imprime « 0 » quand il ne trouve rien ET rend 1 : un « || echo 0 »
# ici produit « 0\n0 », que [ -gt ] refuse. On ignore le code, on garde la sortie.
verifies=$(grep -c . /tmp/.qosmo-liens.$$ 2>/dev/null); verifies=${verifies:-0}
casses=$(grep -c '^KO$' /tmp/.qosmo-liens.$$ 2>/dev/null); casses=${casses:-0}
rm -f /tmp/.qosmo-liens.$$

if [ "$casses" -gt 0 ]; then
    printf '%s%d lien(s) casse(s) sur %d verifie(s)%s\n' "$rouge" "$casses" "$verifies" "$nc" >&2
    exit 1
fi
printf '%s%d lien(s) de doc verifie(s), aucun casse%s\n' "$vert" "$verifies" "$nc"
