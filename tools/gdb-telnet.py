#!/usr/bin/env python3
"""
gdb-telnet.py — console gdb (ARM du Calypso emule) servie en telnet, NON BLOQUANTE.

    telnet localhost 44444

Ce qu'on obtient : un gdb-multiarch attache au gdbstub de QEMU (-gdb tcp::1234),
la cible EN MARCHE (`continue &`), et le prompt (gdb) disponible tout de suite.
    Ctrl-C            arrete l'ARM (SIGINT -> gdb interrompt la cible), on inspecte
    continue &        repart, sans bloquer le prompt
    quit / Ctrl-]     la session se ferme, gdb est tue, QEMU reprend l'ARM

Pourquoi un serveur maison et pas `socat ... EXEC:gdb,pty` : telnet reste en
« ligne par ligne » face a un serveur muet, et Ctrl-C part alors en sequence
IAC IP que gdb ne comprend pas. Ici on negocie ECHO+SGA (mode caractere) : le
client envoie 0x03 brut, le pty (isig) le transforme en SIGINT pour gdb. Les
sequences IAC restantes sont filtrees, IAC IP/BRK deviennent un 0x03.

Une session a la fois : le gdbstub de QEMU n'accepte qu'un client. Tant que
personne n'est connecte, aucun gdb ne tourne et l'ARM n'est jamais touche.
"""
import argparse, os, pty, select, signal, socket, sys, time

IAC, DONT, DO, WONT, WILL, SB, SE = 255, 254, 253, 252, 251, 250, 240
IP, BRK = 244, 243
ECHO, SGA = 1, 3

# Ctrl-C : quand la cible tourne EN ARRIERE-PLAN (continue &), un SIGINT au
# prompt gdb ne l'arrete pas — readline repond juste « Quit ». Ce qui l'arrete,
# c'est la commande `interrupt`. On efface la ligne en cours (Ctrl-U) puis on
# l'envoie ; gdb affiche « Program received signal SIGINT » et la main revient.
CTRL_C = b'\x15interrupt\n'


def log(msg):
    sys.stderr.write(time.strftime('%H:%M:%S ') + msg + '\n'); sys.stderr.flush()


def filter_telnet(data, state):
    """Retire les commandes telnet, rend les octets destines a gdb."""
    out = bytearray()
    buf = state['pending'] + data
    i = 0
    while i < len(buf):
        b = buf[i]
        if b == IAC:
            if i + 1 >= len(buf):
                break                       # commande coupee : on attend la suite
            c = buf[i + 1]
            if c == IAC:
                out.append(IAC); i += 2; continue
            if c in (IP, BRK):
                out += CTRL_C; i += 2; continue           # Ctrl-C telnet -> interrupt
            if c in (DO, DONT, WILL, WONT):
                if i + 2 >= len(buf):
                    break
                i += 3; continue
            if c == SB:
                j = buf.find(bytes([IAC, SE]), i)
                if j < 0:
                    break
                i = j + 2; continue
            i += 2; continue
        if b == 13:                          # Entree telnet : CR LF ou CR NUL -> CR
            out.append(13)
            i += 2 if i + 1 < len(buf) and buf[i + 1] in (0, 10) else 1
            continue
        if b == 0:
            i += 1; continue
        if b == 3:                           # Ctrl-C brut (mode caractere)
            out += CTRL_C; i += 1; continue
        out.append(b); i += 1
    state['pending'] = bytes(buf[i:])
    return bytes(out)


def session(conn, args):
    conn.sendall(bytes([IAC, WILL, ECHO, IAC, WILL, SGA, IAC, DO, SGA]))
    banner = (f"\r\n[gdb-telnet] gdb -> QEMU gdbstub :{args.stub}  elf={args.elf}\r\n"
              "[gdb-telnet] cible EN MARCHE (continue &). Ctrl-C = stop, go = repart, "
              "quit = ferme (QEMU reprend)\r\n"
              "[gdb-telnet] panneau osmocom charge : tape  help_osmo\r\n\r\n")
    conn.sendall(banner.encode())
    pid, fd = pty.fork()
    if pid == 0:
        os.environ['TERM'] = 'vt100'
        # readline sans bracketed-paste : sinon chaque prompt est encadre de
        # sequences ESC[?2004h/l illisibles dans un telnet nu.
        rc = '/tmp/gdb-telnet.inputrc'
        try:
            with open(rc, 'w') as f:
                f.write('set enable-bracketed-paste off\nset bell-style none\n')
            os.environ['INPUTRC'] = rc
        except OSError:
            pass
        cmd = [args.gdb, '-q', '-nx',
               '-iex', 'set pagination off', '-iex', 'set confirm off',
               '-iex', 'set height 0', '-iex', 'set width 0', '-iex', 'set style enabled off',
               '-ex', 'set architecture armv5te',
               '-ex', f'target remote 127.0.0.1:{args.stub}',
               '-ex', 'continue &']
        # panneau osmocom (tools/cmd.gdb, genere par gdb_cmd.sh) : source auto
        panel = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cmd.gdb')
        if os.path.isfile(panel):
            cmd += ['-x', panel]
        if args.elf:
            cmd.append(args.elf)
        try:
            os.execvp(cmd[0], cmd)
        except OSError as e:
            os.write(2, f"exec {cmd[0]} : {e}\n".encode()); os._exit(127)
    state = {'pending': b''}
    try:
        while True:
            r, _, _ = select.select([conn, fd], [], [], 1.0)
            if conn in r:
                try:
                    data = conn.recv(4096)
                except OSError:
                    break
                if not data:
                    break
                if b'\x03' in data or bytes([IAC, IP]) in data or bytes([IAC, BRK]) in data:
                    # vrai SIGINT a gdb : coupe une boucle (trace_*, every) ou une
                    # execution en avant-plan ; le `interrupt` qui suit arrete
                    # la cible si elle tournait en arriere-plan (continue &).
                    try:
                        os.killpg(os.getpgid(pid), signal.SIGINT)
                    except OSError:
                        pass
                out = filter_telnet(data, state)
                if out:
                    os.write(fd, out)
            if fd in r:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                conn.sendall(data)
            try:
                if os.waitpid(pid, os.WNOHANG)[0] == pid:
                    pid = 0; break
            except ChildProcessError:
                pid = 0; break
    finally:
        if pid:
            # detach d'abord : la cible repart proprement meme si elle etait arretee
            try:
                os.write(fd, b'\x03')
                time.sleep(0.2)
                os.write(fd, b'detach\nquit\n')
                for _ in range(10):
                    if os.waitpid(pid, os.WNOHANG)[0] == pid:
                        pid = 0; break
                    time.sleep(0.1)
            except OSError:
                pass
            if pid:
                try:
                    os.kill(pid, signal.SIGKILL); os.waitpid(pid, 0)
                except OSError:
                    pass
        try:
            os.close(fd)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=44444, help='port telnet (defaut 44444)')
    ap.add_argument('--bind', default='127.0.0.1', help="adresse d'ecoute (defaut 127.0.0.1)")
    ap.add_argument('--stub', type=int, default=1234, help='port du gdbstub QEMU (defaut 1234)')
    ap.add_argument('--elf', default=os.environ.get('CALYPSO_GDB_ELF', ''), help='ELF du firmware (symboles)')
    ap.add_argument('--gdb', default='gdb-multiarch')
    args = ap.parse_args()
    signal.signal(signal.SIGCHLD, signal.SIG_DFL)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port)); srv.listen(2)
    log(f"ecoute telnet {args.bind}:{args.port} -> gdbstub :{args.stub} ({args.gdb}, elf={args.elf or '-'})")
    while True:
        conn, peer = srv.accept()
        log(f"session {peer[0]}:{peer[1]}")
        try:
            session(conn, args)
        except Exception as e:
            log(f"session : {e}")
        finally:
            try:
                conn.close()
            except OSError:
                pass
            log("session fermee, gdb termine, QEMU reprend")


if __name__ == '__main__':
    main()
