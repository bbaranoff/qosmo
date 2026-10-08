# demo-layer1.py — « demo » : montrer qu'on a la main sur le firmware layer1 (osmocom-bb)
# qui tourne dans QEMU, depuis la console telnet (tools/gdb-telnet.py, port 44444).
#
#   demo            tout : arrete l'ARM, etat, une trame TDMA au point d'arret, un printf
#                   dans le firmware (visible dans osmocon.log), puis l'ARM repart
#   demo etat       seulement l'etat (pc, pile, fn, cellule, boite aux lettres DSP) ; reste arrete
#   demo trame      tourne jusqu'a la prochaine trame TDMA (tdma_sched_execute) ; reste arrete
#   demo hello      call printf(...) dans le firmware -> console sercomm (osmocon.log) ; reste arrete
#   demo go         l'ARM repart (continue &)
#
# Entre deux etapes : 4 s pour lire (set $demo_pause = N pour changer, 0 = enchainer).
# Si l'ARM tourne, « demo » l'arrete d'abord (interrupt) et enchaine au prochain evenement
# d'arret (gdb.events.stop) : rien n'est lu tant que la cible n'est pas arretee.
# Les lectures passent par les commandes du panneau cmd.gdb (fn, l1, tasks), charge avant.
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
    # entre deux etapes, le temps de lire : 4 s par defaut, « set $demo_pause = 2 » pour changer
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
    # Pas de malloc dans le firmware : gdb ne peut pas y poser une chaine litterale. On l'ecrit
    # nous-memes juste au-dessus du bss (_end + 0x100) : zone libre (pas de tas, les piles sont
    # tout en haut de la RAM, TOP_OF_RAM 0x83fff0 dans board/compal/macros.S), puis puts() dessus.
    fn = val("l1s.current_time.fn")
    out("[demo] ===== 5. ON FAIT PARLER LE FIRMWARE : la chaine ecrite en RAM libre, puis call puts() -> console sercomm (osmocon.log)")
    adr = val("(unsigned long)&_end")
    if adr is None:
        out("[demo]        (_end introuvable : ELF sans symboles ?)"); return
    adr = (adr + 0x100) & ~0xf
    # Couleurs ANSI dans la chaine elle-meme : osmocon recopie la console sercomm octet pour octet
    # (hdlc_console_cb -> write(1, ...)), donc « tail -f osmocon.log » sous terminal l'affiche en couleur.
    # hello (vert) a (blanc) 40c3 (violet) : « hello a 40c3 », ecrit en RAM puis sorti par puts().
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
    """demo [tout|etat|trame|hello|go] — la main sur le firmware layer1 : arret, etat, une trame, printf, reprise."""

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
