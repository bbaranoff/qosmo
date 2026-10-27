#!/bin/bash
# make-overlay.sh — collects the files specific to qosmo into a directory.
#
#     vanilla QEMU (v$(cat VERSION))  +  overlay  =  qosmo
#
# "Specific to qosmo" = tracked by git here AND (absent from vanilla QEMU OR
# different from it). Everything else is original QEMU and stays out. Symbolic
# links only count if their target differs.
#
# The directory is rebuilt on each run (no stale file) and carries:
#   MANIFEST.ajoutes   files absent from vanilla
#   MANIFEST.modifies  vanilla files modified by qosmo
#   MANIFEST.retires   vanilla files qosmo does not track (gitlab CI, roms/...),
#                      for information: the overlay does not delete them
#
# Rebuilding the tree:  cp -a <vanilla> tree && cp -a <overlay>/. tree/
#                       (then rm tree/MANIFEST.*)
#
# Vanilla is taken from $QEMU_GENUINE (default ../qemu-<VERSION>); if absent, it
# is cloned (depth 1, tag v<VERSION>) from gitlab.com/qemu-project/qemu.
#
# Usage: ./make-overlay.sh [--dry-run] [DIR]   (default ../qosmo-overlay)
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

# Artifacts that do not belong in an overlay, even if tracked by mistake.
exclu() {
    case "$1" in
        */__pycache__/*|*.pyc|*.o|*.d|*.orig|*.rej|*~|*.swp) return 0;;
    esac
    return 1
}

# Same content? A link is compared on its target string, not on what it points to.
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
    [ -e "$SRC/$f" ] || [ -L "$SRC/$f" ] || continue   # tracked but removed from disk
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

# Only wipe what this script produced itself.
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
