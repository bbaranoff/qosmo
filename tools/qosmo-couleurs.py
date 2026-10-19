#!/usr/bin/env python3
"""qosmo-couleurs : colore a la volee les journaux du banc (stdin -> stdout).

  tail -F qemu-asm.log | qosmo-couleurs            trace -d in_asm / exec de QEMU
  tail -F qemu.log     | qosmo-couleurs            [trx] [sim] [dcch] [c54x] ...
  tail -F mailbox.log  | qosmo-couleurs            ARM>WR ARM<RD DSP>WR DSP<RD
  watch -n 0.5 --color "tail -n 40 qemu-asm.log | qosmo-couleurs"

Les lignes deja colorees (mobile.log) passent telles quelles. QOSMO_COULEURS=0
ou une sortie qui n'est pas un terminal (et pas --force) : copie sans couleur.
Les panes tmux du banc (tmux_modules/_commun.sh) et ./run.sh --logs de c54x_exe
passent par ici. Rien n'est ecrit dans les fichiers : la couleur est a l'affichage."""
import os, re, sys, signal

def principal():
    force = '--force' in sys.argv[1:]
    actif = os.environ.get('QOSMO_COULEURS', '1') != '0' and (force or sys.stdout.isatty())
    out = sys.stdout
    if not actif:
        for l in sys.stdin:
            out.write(l); out.flush()
        return
    R = '\033[0m'
    def c(n, g=''):   return f'\033[{g}38;5;{n}m'
    DIM, GRAS = '\033[2m', '\033[1m'
    ADR, OCT, SYM, PC, HOTE = c(81, '1;'), c(242), c(114), c(221, '1;'), c(240)
    TAGS = {'trx': c(75), 'sim': c(176), 'dcch': c(80), 'c54x': c(179), 'mbx': c(114),
            'mailbox': c(114), 'sb': c(215), 'a_sch': c(215), 'sbwin': c(215), 'jalon': c(208, '1;'),
            'fb': c(215), 'pont': c(75), 'bsp': c(179), 'calypso-debug': c(242), 'run': c(250, '1;'),
            'gdb-telnet': c(250), 'api': c(114), 'a5': c(176), 'rhea': c(179)}
    SENS = {'ARM>WR': c(221, '1;'), 'ARM<RD': c(81), 'DSP>WR': c(114, '1;'), 'DSP<RD': c(176)}
    MAUVAIS = c(203, '1;'); BON = c(114, '1;'); ATTENTION = c(215)
    re_adr   = re.compile(r'^(0x[0-9a-f]{8}):(.*)$')
    re_objd  = re.compile(r'^(OBJD-T:)\s*([0-9a-f]+)\s*$')
    re_trace = re.compile(r'^(Trace \d+: )(0x[0-9a-f]+) \[([0-9a-f]+)/([0-9a-f]+)/([0-9a-f]+)/([0-9a-f]+)\]\s*(\S*)\s*$')
    re_stop  = re.compile(r'^(Stopped execution of TB chain before|Linking TBs|cpu_io_recompile)(.*)$')
    re_tag   = re.compile(r'^(\s*)\[([A-Za-z0-9_-]+)\]')
    re_mbx   = re.compile(r'^(\s*\d+\s+\d+\s+)(ARM>WR|ARM<RD|DSP>WR|DSP<RD)(\s+)(0x[0-9a-f]+|\S+)(\s+\S+\s+)(.*)$')
    re_fn    = re.compile(r'\b(fn=)(\d+)')
    re_hex   = re.compile(r'\b(0x[0-9A-Fa-f]{2,})\b')
    re_mal   = re.compile(r'(ERREUR|ECHEC|erreur|error|Error|ERROR|\bKO\b|perdu[es]?|PERDUE?|PAS (?:finie|recue)|crc_ko|Dropping|timeout|TIMEOUT|result=255|no cell|quand meme|INACTIF|introuvable|impossible|tick saute)')
    re_bon   = re.compile(r'(\bOK\b|CRC_OK|\bfinie\b|PLAUSIBLE|accept|ACTIF|tout tourne|Synchronize_TDMA|=> SB|camp)')
    re_att   = re.compile(r'(attente|attend|retard|en retard|toujours pas|LOST|saute)')
    re_asmins = re.compile(r'^(\s+[0-9a-f]+:\s+)([0-9a-f]{8}|[0-9a-f]{4} [0-9a-f]{4}|[0-9a-f]{4})(\s+)([a-z][a-z0-9.]*)(.*)$')
    def generique(s):
        s = re_mal.sub(MAUVAIS + r'\1' + R, s)
        s = re_bon.sub(BON + r'\1' + R, s)
        s = re_att.sub(ATTENTION + r'\1' + R, s)
        s = re_fn.sub(r'\1' + GRAS + r'\2' + R, s)
        s = re_hex.sub(c(81) + r'\1' + R, s)
        return s
    for l in sys.stdin:
        if '\033[' in l:                       # deja colore (mobile.log) : tel quel
            out.write(l); out.flush(); continue
        s = l.rstrip('\n'); t = s
        m = re_adr.match(s)
        if m:   t = f'{ADR}{m.group(1)}{R}:{m.group(2)}'
        elif (m := re_objd.match(s)):
            t = f'{DIM}{m.group(1)}{R} {OCT}{m.group(2)}{R}'
        elif (m := re_trace.match(s)):
            pc = m.group(4).lstrip('0') or '0'
            t = (f'{DIM}{m.group(1)}{R}{HOTE}{m.group(2)}{R} {DIM}[{m.group(3)}/{R}'
                 f'{PC}{"0x" + pc.rjust(8, "0")}{R}{DIM}/{m.group(5)}/{m.group(6)}]{R} {SYM}{m.group(7)}{R}')
        elif (m := re_stop.match(s)):
            t = f'{c(176)}{m.group(1)}{R}{DIM}{m.group(2)}{R}'
        elif s.startswith('IN:') or s.startswith('----'):
            t = f'{DIM}{s}{R}'
        elif (m := re_mbx.match(s)):
            t = (f'{DIM}{m.group(1)}{R}{SENS[m.group(2)]}{m.group(2)}{R}{m.group(3)}'
                 f'{ADR}{m.group(4)}{R}{SYM}{m.group(5)}{R}{generique(m.group(6))}')
        elif (m := re_asmins.match(s)):
            t = f'{DIM}{m.group(1)}{R}{OCT}{m.group(2)}{R}{m.group(3)}{PC}{m.group(4)}{R}{generique(m.group(5))}'
        elif s.startswith('#'):
            t = f'{DIM}{s}{R}'
        else:
            m = re_tag.match(s)
            if m:
                col = TAGS.get(m.group(2).lower(), c(250))
                t = f'{m.group(1)}{col}[{m.group(2)}]{R}' + generique(s[m.end():])
            else:
                t = generique(s)
        out.write(t + '\n'); out.flush()

if __name__ == '__main__':
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    try:
        principal()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
