#!/bin/bash
# make-overlay.sh — range dans un dossier les fichiers propres a qosmo.
#
#     QEMU vanilla (v$(cat VERSION))  +  overlay  =  qosmo
#
# « Propre a qosmo » = suivi par git ici ET (absent du QEMU vanilla OU different
# de lui). Tout le reste est du QEMU d'origine et reste dehors. Les liens
# symboliques ne comptent que si leur cible differe.
#
# Le dossier est reconstruit a chaque passage (pas de fichier perime) et porte :
#   MANIFEST.ajoutes   fichiers absents du vanilla
#   MANIFEST.modifies  fichiers vanilla modifies par qosmo
#   MANIFEST.retires   fichiers vanilla que qosmo ne suit pas (CI gitlab, roms/…),
#                      pour information : l'overlay ne les supprime pas
#
# Reconstituer l'arbre :  cp -a <vanilla> arbre && cp -a <overlay>/. arbre/
#                         (puis rm arbre/MANIFEST.*)
#
# Le vanilla est pris dans $QEMU_GENUINE (defaut ../qemu-<VERSION>) ; absent, il
# est clone (depth 1, tag v<VERSION>) depuis gitlab.com/qemu-project/qemu.
#
# Usage : ./make-overlay.sh [--dry-run] [DOSSIER]   (defaut ../qosmo-overlay)
set -euo pipefail
DRY=0; [ "${1:-}" = "--dry-run" ] && { DRY=1; shift; }
SRC="$(cd "$(dirname "$0")" && pwd)"
VER="$(cat "$SRC/VERSION")"
OUT="${1:-$SRC/../qosmo-overlay}"
GEN="${QEMU_GENUINE:-$SRC/../qemu-$VER}"

if [ ! -f "$GEN/VERSION" ]; then
    echo "vanilla absent : clone de v$VER dans $GEN"
    git clone -q --depth 1 --branch "v$VER" https://gitlab.com/qemu-project/qemu.git "$GEN"
fi
[ "$(cat "$GEN/VERSION")" = "$VER" ] || {
    echo "!! $GEN est en $(cat "$GEN/VERSION"), qosmo en $VER" >&2; exit 1; }

# Artefacts qui n'ont rien a faire dans un overlay, meme suivis par erreur.
exclu() {
    case "$1" in
        */__pycache__/*|*.pyc|*.o|*.d|*.orig|*.rej|*~|*.swp) return 0;;
    esac
    return 1
}

# Meme contenu ? Un lien se compare sur sa cible, pas sur ce qu'il pointe.
pareil() {
    local a="$SRC/$1" b="$GEN/$1"
    if [ -L "$a" ] || [ -L "$b" ]; then
        [ -L "$a" ] && [ -L "$b" ] && [ "$(readlink "$a")" = "$(readlink "$b")" ]
    else
        [ -f "$b" ] && cmp -s "$a" "$b"
    fi
}

tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
(cd "$GEN" && git ls-files) | sort > "$tmp/vanilla"
(cd "$SRC" && git ls-files) | sort > "$tmp/qosmo"
: > "$tmp/ajoutes"; : > "$tmp/modifies"; skip=0
while IFS= read -r f; do
    if exclu "$f"; then skip=$((skip+1)); continue; fi
    [ -e "$SRC/$f" ] || [ -L "$SRC/$f" ] || continue   # suivi mais supprime du disque
    echo "$f" >> "$tmp/ajoutes"
done < <(comm -13 "$tmp/vanilla" "$tmp/qosmo")
while IFS= read -r f; do
    pareil "$f" || echo "$f" >> "$tmp/modifies"
done < <(comm -12 "$tmp/vanilla" "$tmp/qosmo")
comm -23 "$tmp/vanilla" "$tmp/qosmo" > "$tmp/retires"

na=$(wc -l < "$tmp/ajoutes"); nm=$(wc -l < "$tmp/modifies"); nr=$(wc -l < "$tmp/retires")
echo "qosmo    : $SRC ($VER)"
echo "vanilla  : $GEN"
echo "overlay  : $OUT"
echo "----"
sed 's/^/  ajoute  /' "$tmp/ajoutes"
sed 's/^/  modifie /' "$tmp/modifies"
echo "----"
echo "$na ajoute(s), $nm modifie(s), $nr vanilla non suivi(s), $skip artefact(s) ecarte(s)"
[ "$DRY" = 1 ] && { echo "DRY-RUN : rien d'ecrit."; exit 0; }

# On ne vide que ce que ce script a lui-meme produit.
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT")" ] && [ ! -f "$OUT/MANIFEST.ajoutes" ]; then
    echo "!! $OUT existe, n'est pas vide et n'est pas un overlay qosmo : je n'y touche pas" >&2
    exit 1
fi
rm -rf "$OUT"; mkdir -p "$OUT"
(cd "$SRC" && cat "$tmp/ajoutes" "$tmp/modifies" | xargs -r -d "\n" cp -a --parents -t "$OUT")
cp "$tmp/ajoutes" "$OUT/MANIFEST.ajoutes"
cp "$tmp/modifies" "$OUT/MANIFEST.modifies"
cp "$tmp/retires" "$OUT/MANIFEST.retires"
echo "ecrit : $(find "$OUT" -type f -o -type l | wc -l) entree(s) dans $OUT"
