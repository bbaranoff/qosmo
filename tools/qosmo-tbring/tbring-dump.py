#!/usr/bin/env python3
"""tbring-dump : les N derniers blocs ARM executes, lus dans l'anneau du plugin tbring.

  tbring-dump [-n N] [-f /dev/shm/qemu-tbring] [-e layer1.highram.elf] [-a qemu-asm.log] [-r]

  -n N    nombre de blocs (defaut 200), les plus recents en dernier
  -e ELF  symboles (nm) : fonction+decalage a cote de chaque PC
  -a LOG  trace -d in_asm du meme run : les octets « OBJD-T » du bloc, desassembles
          avec arm-none-eabi-objdump si present (ce build de QEMU n'a pas capstone)
  -r      compte les blocs par PC sur toute la fenetre (histogramme) au lieu de la liste
  -c/-C   couleurs forcees / coupees (defaut : si la sortie est un terminal)

Lecture a chaud, sans arreter QEMU : l'anneau est de la memoire partagee."""
import argparse, struct, subprocess, shutil, collections, os, sys, tempfile

def lire(path, n):
    with open(path, 'rb') as f:
        hdr = f.read(64)
        magic, entries, count, shift = struct.unpack('<8sQQQ', hdr[:32])
        if magic != b'TBRING01':
            sys.exit(f"{path}: pas un anneau tbring")
        n = min(n, entries, count)
        out = []
        for i in range(count - n, count):
            f.seek(64 + (i & (entries - 1)) * 8)
            pc, tour = struct.unpack('<II', f.read(8))
            if tour != ((i >> shift) & 0xffffffff):
                out.append((None, i))      # ecrase entre-temps (QEMU a fait un tour)
            else:
                out.append((pc, i))
        return entries, count, out

def symboles(elf):
    syms = []
    try:
        txt = subprocess.run(['nm', '-n', '-S', elf], capture_output=True, text=True, check=True).stdout
    except Exception as e:
        print(f"nm {elf}: {e}", file=sys.stderr); return syms
    for l in txt.splitlines():
        p = l.split()
        if len(p) == 4 and p[2] in 'tTwW' and not p[3].startswith('$'):
            syms.append((int(p[0], 16), int(p[1], 16), p[3]))
    syms.sort()
    return syms

def resoudre(syms, pc):
    import bisect
    i = bisect.bisect_right([s[0] for s in syms], pc) - 1
    if i >= 0:
        a, sz, nom = syms[i]
        if pc < a + max(sz, 4):
            return f"{nom}+0x{pc - a:x}"
    return "?"

def in_asm(log):
    """pc -> (octets hex) pour chaque bloc traduit (premiere occurrence)."""
    table = {}
    pc = None
    with open(log, errors='replace') as f:
        for l in f:
            if l.startswith('0x') and l[10:11] == ':':
                try: pc = int(l[:10], 16)
                except ValueError: pc = None
            elif l.startswith('OBJD-T:') and pc is not None and pc not in table:
                table[pc] = l.split(':', 1)[1].strip()
                pc = None
    return table

def desasm(pc, hexs):
    od = shutil.which('arm-none-eabi-objdump')
    if not od:
        return [f"    .word {hexs}"]
    with tempfile.NamedTemporaryFile(suffix='.bin', delete=False) as t:
        t.write(bytes.fromhex(hexs)); tmp = t.name
    try:
        r = subprocess.run([od, '-D', '-b', 'binary', '-m', 'arm', '-M', 'force-thumb' if pc & 1 else 'no-force-thumb',
                            f'--adjust-vma=0x{pc & ~1:x}', tmp], capture_output=True, text=True)
        return ['    ' + l.strip() for l in r.stdout.splitlines() if l[:1] == ' ' and ':' in l]
    finally:
        os.unlink(tmp)

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('-n', type=int, default=200); ap.add_argument('-f', default='/dev/shm/qemu-tbring')
ap.add_argument('-e'); ap.add_argument('-a'); ap.add_argument('-r', action='store_true')
ap.add_argument('-c', action='store_true'); ap.add_argument('-C', action='store_true')
a = ap.parse_args()
COUL = (a.c or (sys.stdout.isatty() and os.environ.get('QOSMO_COULEURS', '1') != '0')) and not a.C
def K(n, s, g=''):
    return f"\033[{g}38;5;{n}m{s}\033[0m" if COUL else str(s)
entries, count, blocs = lire(a.f, (1 << 62) if a.r and a.n == 200 else a.n)   # -r sans -n : toute la fenetre
syms = symboles(a.e) if a.e else []
table = in_asm(a.a) if a.a else {}
print(K(242, f"# {a.f} : {count} blocs executes, anneau de {entries}, {len(blocs)} affiches"))
if a.r:
    h = collections.Counter(pc for pc, _ in blocs if pc is not None)
    for pc, k in h.most_common(60):
        print(f"{K(221, f'{k:10d}', '1;')}  {K(81, f'0x{pc:08x}', '1;')}  {K(114, resoudre(syms, pc) if syms else '')}")
else:
    for pc, i in blocs:
        if pc is None:
            print(K(203, f"{i:12d}  (ecrase)")); continue
        print(f"{K(242, f'{i:12d}')}  {K(81, f'0x{pc:08x}', '1;')}  {K(114, resoudre(syms, pc) if syms else '')}")
        if pc in table:
            for l in desasm(pc, table[pc]):
                p = l.split(None, 2)   # adresse:  octets  mnemonique operandes
                if len(p) == 3 and COUL:
                    m = p[2].split(None, 1)
                    print(f"    {K(242, p[0])} {K(242, p[1])}\t{K(221, m[0], '1;')} {m[1] if len(m) > 1 else ''}")
                else:
                    print(l)
