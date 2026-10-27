# demo-layer1.py — "demo": show that we control the layer1 firmware (osmocom-bb)
# running in QEMU, from the telnet console (tools/gdb-telnet.py, port 44444).
#
#   demo            everything: stop the ARM, state, one TDMA frame at a breakpoint, a
#                   printf in the firmware (visible in osmocon.log), then the ARM resumes
#   demo etat       state only (pc, stack, fn, cell, DSP mailbox); stays stopped
#   demo trame      runs until the next TDMA frame (tdma_sched_execute); stays stopped
#   demo hello      call printf(...) in the firmware -> sercomm console (osmocon.log); stays stopped
#   demo go         the ARM resumes (continue &)
#
# Between two steps: 4 s to read (set $demo_pause = N to change, 0 = no pause).
# If the ARM is running, "demo" stops it first (interrupt) and goes on at the next
# stop event (gdb.events.stop): nothing is read until the target is stopped.
# Reads go through the cmd.gdb panel commands (fn, l1, tasks), loaded beforehand.
import gdb, time


def ex(c):
    try:
        return gdb.execute(c, to_string=True)
    except gdb.error as e:
        return "  (%s : %s)\n" % (c, e)


def out(t):
    gdb.write(t if t.endswith("\n") else t + "\n")
    gdb.flush()


def en_marche():
    try:
        return any(t.is_running() for t in gdb.selected_inferior().threads())
    except gdb.error:
        return False


def pause_etape():
    # between two steps, time to read: 4 s by default, "set $demo_pause = 2" to change
    p = gdb.convenience_variable("demo_pause")
    t = float(p) if p is not None else 4.0
    if t > 0:
        out("[demo]        ... (suite dans %.0f s)" % t)
        time.sleep(t)


def val(expr):
    try:
        return int(gdb.parse_and_eval(expr))
    except gdb.error:
        return None


def etat():
    fr = gdb.newest_frame()
    sal = fr.find_sal()
    ou = "%s:%d" % (sal.symtab.filename, sal.line) if sal.symtab else "?"
    out("[demo] ===== 1. L'ARM EST ARRETE : pc=0x%08x dans %s (%s)" % (fr.pc(), fr.name() or "?", ou))
    out(ex("bt 6"))
    out(ex("x/3i $pc"))
    pause_etape()
    out("[demo] ===== 2. CE QUE LA COUCHE 1 SAIT (symboles l1s du firmware) :")
    out(ex("fn"))
    out(ex("l1"))
    pause_etape()
    out("[demo] ===== 3. LA BOITE AUX LETTRES ARM<->DSP, lue par l'ARM a 0xffd00000")
    out("[demo]        (= « d@ 0800 8 » dans harvard.sh : meme memoire, /dev/shm/calypso_api_ram)")
    out(ex("x/8hx 0xffd00000"))
    out(ex("tasks"))
    pause_etape()


def trame():
    avant = val("l1s.current_time.fn")
    out("[demo] ===== 4. ON RELANCE JUSQU'A LA PROCHAINE TRAME TDMA (tbreak tdma_sched_execute), fn=%s" % avant)
    t0 = time.time()
    ex("tbreak tdma_sched_execute")
    gdb.execute("continue")
    apres = val("l1s.current_time.fn")
    d = (apres - avant) if (apres is not None and avant is not None) else "?"
    out("[demo]        arrete dans tdma_sched_execute : fn=%s (+%s) en %.0f ms  (une trame = 4,615 ms)"
        % (apres, d, 1000 * (time.time() - t0)))
    out(ex("bt 4"))
    pause_etape()


def hello():
    # No malloc in the firmware: gdb cannot place a string literal there. It is written
    # by hand just above the bss (_end + 0x100): free area (no heap, the stacks are at
    # the very top of RAM, TOP_OF_RAM 0x83fff0 in board/compal/macros.S), then puts() on it.
    fn = val("l1s.current_time.fn")
    out("[demo] ===== 5. ON FAIT PARLER LE FIRMWARE : la chaine ecrite en RAM libre, puis call puts() -> console sercomm (osmocon.log)")
    adr = val("(unsigned long)&_end")
    if adr is None:
        out("[demo]        (_end introuvable : ELF sans symboles ?)"); return
    adr = (adr + 0x100) & ~0xf
    # ANSI colours in the string itself: osmocon copies the sercomm console byte for byte
    # (hdlc_console_cb -> write(1, ...)), so "tail -f osmocon.log" in a terminal shows it in colour.
    # hello (green) a (white) 40c3 (purple): "hello a 40c3", written to RAM then output by puts().
    texte = ("\033[1;32mhello\033[0m \033[1;37ma\033[0m \033[1;35m40c3\033[0m "
             "\033[36m(0x%x, fn=%s)\033[0m\n" % (adr, fn))
    msg = texte.encode() + b"\0"
    gdb.selected_inferior().write_memory(adr, msg)
    out("[demo]        %d octets ecrits a 0x%08x : %s" % (len(msg), adr, ex("x/s 0x%x" % adr).strip()))
    out("[demo]        soit, rendu : " + texte.rstrip("\n"))
    out(ex("call (int)puts((const char *)0x%x)" % adr))
    out("[demo]        (regarder : tail -f /tmp/c54x-pont/osmocon.log)")
    pause_etape()


def fin():
    out(ex("continue &"))
    out("[demo] ===== FIN : l'ARM est RELANCE et tourne a nouveau, rien de casse (Ctrl-C ou stop pour l'arreter, help_osmo pour le reste).")
    try:
        time.sleep(1.0)
        out("[demo]        1 s plus tard, l'ARM a bien repris : %s" % ("il tourne (continue &)" if en_marche() else "il est ARRETE ?!"))
    except Exception:
        pass


def corps(arg):
    try:
        if arg in ("etat", "tout"):
            etat()
        if arg in ("trame", "tout"):
            trame()
        if arg in ("hello", "tout"):
            hello()
        if arg in ("go", "tout"):
            fin()
        if arg not in ("etat", "trame", "hello", "go", "tout"):
            out("usage : demo [tout|etat|trame|hello|go]")
    except Exception as e:
        out("[demo] erreur : %s" % e)


class Demo(gdb.Command):
    """demo [tout|etat|trame|hello|go] — control of the layer1 firmware: stop, state, one frame, printf, resume."""

    def __init__(self):
        super().__init__("demo", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        arg = arg.strip() or "tout"
        if en_marche():
            out("[demo] l'ARM tourne : interrupt ; la suite des que la cible est arretee...")

            def on_stop(ev):
                gdb.events.stop.disconnect(on_stop)
                gdb.post_event(lambda: corps(arg))
            gdb.events.stop.connect(on_stop)
            gdb.execute("interrupt")
        else:
            corps(arg)


Demo()
gdb.write("[demo-layer1] tape  demo  (ou demo etat | trame | hello | go)\n")
