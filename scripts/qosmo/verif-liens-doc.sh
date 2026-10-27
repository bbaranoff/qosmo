#!/bin/bash
# Checks that every local path cited in a .md of the repository really exists.
# A doc that points into the void does not crash: it is just wrong.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

rouge=$'\033[31m'; vert=$'\033[32m'; nc=$'\033[0m'
casses=0; verifies=0

while IFS= read -r md; do
    # 1. the targets of markdown links [text](path)
    # 2. the backtick-quoted paths that look like repository files
    {
        grep -oE '\]\([^)#][^)]*\)' "$md" | sed -e 's/^](//' -e 's/)$//'
        # Complete markdown links are removed first: a backtick-quoted name used
        # as a LABEL ([`x.c`](url)) is already covered by its target, handled
        # above. Otherwise a legitimate cross-repository reference is reported
        # as broken.
        sed -E 's/\[[^]]*\]\([^)]*\)//g' "$md" \
          | grep -oE '`[a-zA-Z0-9_./-]+\.(c|h|sh|py|md|txt|cfg|conf|build|yml)`' | tr -d '`'
    } 2>/dev/null | sort -u | while IFS= read -r cible; do
        case "$cible" in
            http://*|https://*|mailto:*|'') continue ;;
        esac
        cible="${cible%%#*}"
        [ -z "$cible" ] && continue
        # relative to the .md, then to the repository root
        base="$(dirname "$md")"
        # A bare name between backticks ("calypso_trx.c") is a MENTION, not a
        # path: a file of that name only has to exist somewhere in the
        # repository. An explicit path ("hw/arm/.../x.c", or the target of a
        # markdown link) must exist WHERE it is written.
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

# grep -c prints "0" when it finds nothing AND returns 1: a "|| echo 0" here
# would produce "0\n0", which [ -gt ] rejects. The code is ignored, the output kept.
verifies=$(grep -c . /tmp/.qosmo-liens.$$ 2>/dev/null); verifies=${verifies:-0}
casses=$(grep -c '^KO$' /tmp/.qosmo-liens.$$ 2>/dev/null); casses=${casses:-0}
rm -f /tmp/.qosmo-liens.$$

if [ "$casses" -gt 0 ]; then
    printf '%s%d lien(s) casse(s) sur %d verifie(s)%s\n' "$rouge" "$casses" "$verifies" "$nc" >&2
    exit 1
fi
printf '%s%d lien(s) de doc verifie(s), aucun casse%s\n' "$vert" "$verifies" "$nc"
