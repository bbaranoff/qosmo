/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_mem.c — Memoire donnee et programme : data_read/write, prog_fetch, overlay
 *
 * Extrait de calypso_c54x.c le 2026-09-18 (decoupage par role).
 * Carte des fichiers dans c54x_internal.h.
 */
#include "c54x_internal.h"

static uint16_t data_read_locked(C54xState *s, uint16_t addr);
/* [2026-07-28] DEMODIO : prototype — le helper est defini plus bas mais
 * data_read, qui l appelle, vient avant. */
static void dio_note(C54xState *s, const char *rw, uint16_t addr, uint16_t val);

/* FBWATCH (2026-05-30 soir) : sonde FB-dispatch gatée par une env DÉDIÉE
 * (CALYPSO_FBWATCH=1), résolue UNE fois en static int → check int cheap, PAS
 * via calypso_debug_enabled (master-gate reste 0, 127 gates court-circuités →
 * QEMU temps-réel → mobile vivant). Déclaré ici (avant data_read_locked qui
 * l'utilise). Filme : page-read / dispatch 0x833b / 0x9ac0 / d_fb_det / canary. */
int      g_fbwatch_on = -1;

uint16_t data_read(C54xState *s, uint16_t addr)
{
    /* [2026-07-29] Moniteur mailbox. On trace la valeur telle qu'elle est en
     * mémoire à l'entrée : les quelques cellules synthétisées plus bas (FB-STREAM)
     * sont de toute façon visibles côté écriture. */
    calypso_mbx(MBX_DSP_RD, addr, s->data[addr], 0, s->pc, 0, s->insn_count);

    /* ─────────────────────────────────────────────────────────────────────────
     * [2026-08-04] FEED-DST — QUEL TAMPON LE DEMOD LIT-IL VRAIMENT ?
     * Gate CALYPSO_FEED_DST (defaut 0). LECTURE SEULE, plafonnee.
     *
     * CE QU'ON TRANCHE. Mesure du 04/08 : le DMA livre du VRAI IQ
     * (`contenu : 295/296 mots non nuls` en regime, empreintes variees) et le DSP
     * publie quand meme 89 % de zeros dans `a_cd`. Avant d'accuser le decodeur
     * d'opcodes, il faut savoir si le demod lit seulement la ou le DMA ecrit.
     *
     *   zone A = 0x0cce..0x0df5 : DESTINATION du DMA (AAD=0x99c -> api_ram[0x4ce],
     *            mot DSP 0x0800+0x4ce). C'est la que le vrai IQ arrive.
     *   zone B = 0x4c00..0x4d27 : tampon corrélateur (CALYPSO_BSP_DARAM_ADDR),
     *            celui que corr_iq.py analyse.
     *
     * LECTURE DU RESULTAT : A>0 et B=0 -> le demod lit bien le DMA, le probleme
     * est en aval (decodeur/ALU). A=0 et B>0 -> il lit un tampon que le DMA
     * n'alimente pas : ca suffit a expliquer les zeros SANS bug d'ALU. Les deux
     * a 0 -> ni l'un ni l'autre, et c'est la question qui change.
     *
     * ⚠️ Compte des LECTURES, pas des trames : un ratio A/B ne dit pas « le demod
     * prefere A », il dit combien de mots ont ete lus ou. Et les PC listes sont
     * les 8 PREMIERS distincts rencontres, pas les plus frequents. */
    {
        static int fd_on = -1;
        if (fd_on < 0) {
            fd_on = calypso_gate("CALYPSO_FEED_DST", 0);
            /* Trace d'armement : sans elle, une sonde muette serait
             * indiscernable d'une sonde eteinte. Si FEED-DST s'annonce ARMEE et
             * qu'aucune ligne A/B ne suit, la reponse est « le demod ne lit NI
             * 0x0cce NI 0x4c00 » — c'est un resultat, pas une panne. */
            fprintf(stderr, "[c54x] FEED-DST %s (CALYPSO_FEED_DST=%d) — "
                    "A=0x0cce..0x0df5 (dest DMA), B=0x4c00..0x4d27 (correlateur)\n",
                    fd_on ? "ARMEE" : "eteinte", fd_on);
        }
        if (fd_on) {
            int zone = (addr >= 0x0cce && addr <= 0x0df5) ? 0
                     : (addr >= 0x4c00 && addr <= 0x4d27) ? 1 : -1;
            if (zone >= 0) {
                static unsigned long long n_rd[2];
                static uint16_t pcs[2][8];
                static unsigned  pcn[2][8];
                static int       npc[2];
                n_rd[zone]++;
                int k;
                for (k = 0; k < npc[zone]; k++)
                    if (pcs[zone][k] == s->pc) { pcn[zone][k]++; break; }
                if (k == npc[zone] && npc[zone] < 8) {
                    pcs[zone][k] = s->pc; pcn[zone][k] = 1; npc[zone]++;
                }
                unsigned long long tot = n_rd[0] + n_rd[1];
                static unsigned n_print;
                if ((tot == 1 || (tot % 20000) == 0) && n_print < 20) {
                    n_print++;
                    fprintf(stderr, "[c54x] FEED-DST A(DMA 0x0cce)=%llu "
                            "B(corr 0x4c00)=%llu\n", n_rd[0], n_rd[1]);
                    for (int z = 0; z < 2; z++) {
                        fprintf(stderr, "[c54x] FEED-DST   %s PC:",
                                z == 0 ? "A" : "B");
                        for (int j = 0; j < npc[z]; j++)
                            fprintf(stderr, " 0x%04x(%u)", pcs[z][j], pcn[z][j]);
                        fprintf(stderr, "%s\n", npc[z] ? "" : " (aucune lecture)");
                    }
                    fflush(stderr);
                }
            }
        }
    }

    /* [2026-07-26 golive-mac] WATCH-9F00-RD : l'etage demod deroule 0x9f00
     * ecrit son resultat en 0x2a00 (workzone). Son ENTREE = ce qu'il LIT hors
     * du workzone. On trace les lectures quand PC in [0x9f00..0x9fb8] et addr
     * HORS 0x2a00..0x2b27 pour localiser le vrai buffer IQ source (a nourrir a
     * la place de 0x2a00). Gate CALYPSO_WATCH_9F00_RD. */
    /* [2026-07-27] FB-STREAM : injecte un echantillon FCCH FRAIS a chaque lecture
     * de la cellule sample du demod (0x9213 I / 0x9215 Q) -> vraie fenetre dans
     * 0x2a00, sans dependre de la cadence. Modelise le DMA on-chip. Gate CALYPSO_FB_STREAM. */
    /* [2026-07-27] cellules I/Q du demod configurables : WATCH-9F00-RD a montre
     * que le demod lit 0x9260/0x9261 (pas 0x9213/0x9215). CALYPSO_FB_STREAM_CELL(Q). */
    static uint16_t _fscI = 0, _fscQ = 0;
    if (_fscI == 0) {
        const char *c = getenv("CALYPSO_FB_STREAM_CELL");  _fscI = c ? (uint16_t)strtol(c, NULL, 0) : 0x9213;
        const char *q = getenv("CALYPSO_FB_STREAM_CELLQ"); _fscQ = q ? (uint16_t)strtol(q, NULL, 0) : 0x9215;
    }
    if (s->pc >= 0x9f00 && s->pc <= 0x9fb8 && (addr == _fscI || addr == _fscQ)) {
        static int _fs = -1;
        /* @BEQUILLE — FB_STREAM (lecture)  (CALYPSO_FB_STREAM, defaut OFF)
         *   masque  : l absence de DMA on-chip. On INJECTE un echantillon frais a
         *             chaque lecture de la cellule au lieu qu un peripherique
         *             remplisse le tampon.
         *   retirer : quand le tampon est alimente par le vrai chemin (BSP -> DARAM).
         *   Note : inerte avec CORR_ENTRY=0x94f5 — les cellules 0x9260/61 n y sont
         *   jamais lues (mesure 2026-07-28, WATCH-9F00-RD). */
        if (_fs < 0) _fs = calypso_gate("CALYPSO_FB_STREAM", 0);
        if (_fs) {
            static uint16_t _si, _sq; static int _hv = 0; uint16_t _rv;
            if (addr == _fscI) { _hv = calypso_bsp_fb_stream_next(&_si, &_sq) ? 1 : 0; _rv = _hv ? _si : s->data[addr]; }
            else { _rv = _hv ? _sq : s->data[addr]; }
            static unsigned _sl = 0;
            if (_sl++ < 24)
                fprintf(stderr, "[c54x] FB-STREAM addr=0x%04x -> 0x%04x (cell 0x%04x) hv=%d PC=0x%04x\n",
                        addr, _rv, s->data[addr], _hv, s->pc);
            return _rv;
        }
    }
    if (s->pc >= 0x9f00 && s->pc <= 0x9fb8) {
        static int _r9 = -1;
        if (_r9 < 0) _r9 = calypso_gate("CALYPSO_WATCH_9F00_RD", 0);
        if (_r9) {
            /* Lectures du chemin actif (0x9f00..0x9fb8) SANS exclusion : localise les
             * cellules SOURCE lues avant le fill workzone 0x2a00 (0x9213/0x9215 IQ ?). */
            static unsigned _n9 = 0;
            if (_n9++ < 200)
                fprintf(stderr, "[c54x] WATCH-9F00-RD PC=0x%04x reads addr=0x%04x val=0x%04x insn=%u\n",
                        s->pc, addr, s->data[addr], s->insn_count);
        }
    }
    /* Correlator read tracer (env-gated CALYPSO_CORRELATOR_TRACE=1).
     * Record addr seulement quand PC ∈ [CORR_PC_LO..CORR_PC_HI) (FB-det range).
     * Range étendu 2026-05-25 night à 0x8d00..0x9000 (cf comment block au L639).
     * Lazy-init ici plutôt qu'au top-of-loop pour rester centralisé.
     * Coût quand OFF : 1 compare + 1 branch (g_corr_trace_enabled). */
    if (g_corr_trace_enabled > 0 && s->pc >= CORR_PC_LO && s->pc < CORR_PC_HI) {
        corr_read_record(addr);
    }
    /* IQ-READ tracer (2026-05-30) : qui lit le buffer DMA BSP [0x2a00..0x2b27] ?
     * Confirme que le corrélateur FB consomme bien la vraie I/Q écrite par le
     * BSP, et à quel PC (= le vrai site corrélateur). Cap 60, ~zéro coût hors zone. */
    if (addr >= 0x2a00 && addr < 0x2b28 && s->data[addr] != 0) {
        static unsigned iqr = 0, iqseen = 0;
        iqseen++;
        /* boot (first 60) + DÉTECTION : tire aussi 1/8000 après insn>50M pour
         * voir ce que le corrélateur lit VRAIMENT à l'instant FB-det (insn~71M),
         * pas seulement le buffer stale du boot (2026-06-02). */
        if (iqr < 60 || (s->insn_count > 50000000u && (iqseen % 8000) == 0)) {
            uint16_t val = s->data[addr];
            /* A/B (accumulateurs corr complexe, sign-ext 40b) + valeurs aux
             * AUTRES pointeurs (candidats réf cos/sin) PENDANT la lecture I/Q. */
            int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
            fprintf(stderr, "[c54x] IQ-READ #%u addr=0x%04x val=0x%04x PC=0x%04x A=%lld B=%lld "
                    "T=%04x s=%p | AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                    iqr, addr, val, s->pc, (long long)a, (long long)b, s->t, (void*)s,
                    s->ar[3], s->data[s->ar[3]], s->ar[4], s->data[s->ar[4]],
                    s->ar[5], s->data[s->ar[5]], s->insn_count);
            iqr++;
        }
        /* SPAN write-vs-read (2026-06-02, spec CC-web) : à l'instant détection,
         * dumpe UNE fois le span contigu data[0x2a00..0x2a1f] (ce que le
         * corrélateur PEUT lire) ET bsp_buf[0..31] (ce que le BSP a écrit).
         *   data span = waveform & bsp_buf = waveform  -> buffer OK -> bug = AR3/corrélateur (lit que [0]).
         *   data span = [0] puis DC, bsp_buf = waveform -> livraison PORTR ne copie que [0] (stride/len).
         *   bsp_buf = DC                                -> conversion cs16 BSP tronque. */
        static int span_done = 0;
        if (!span_done && s->insn_count > 60000000u) {
            span_done = 1;
            fprintf(stderr, "[c54x] SPAN-READ  data[0x2a00..0x2a1f] insn=%u:", s->insn_count);
            for (int _i = 0; _i < 32; _i++) fprintf(stderr, " %04x", s->data[0x2a00 + _i]);
            fprintf(stderr, "\n[c54x] SPAN-WRITE bsp_buf[0..31] (bsp_len=%d):", s->bsp_len);
            for (int _i = 0; _i < 32 && _i < s->bsp_len; _i++) fprintf(stderr, " %04x", s->bsp_buf[_i]);
            fprintf(stderr, "\n");
        }
    }
    /* MTTCG : protège DARAM access (DSP + ARM-OVLY peuvent racer).
     * Sans MTTCG : mutex non contesté (overhead minimal). */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    uint16_t v = data_read_locked(s, addr);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    dio_note(s, "R", addr, v);
    return v;
}

static void flow_log(const char *rw, uint16_t addr, uint16_t val, uint16_t pc, unsigned insn);

/* [2026-07-28] RMAP : carte agregee des adresses LUES par les PC d une plage.
 * Symetrique de WMAP. Voir en-tete du patch rmap.py. */
struct c54x_g_rmap_s g_rmap[RMAP_PCS];
int      g_rmap_n;
uint32_t g_rmap_tot;
int      g_rmap_on = -1;
uint16_t g_rmap_pclo, g_rmap_pchi;

static void rmap_dump(void)
{
    fprintf(stderr, "[c54x] RMAP PC 0x%04x..0x%04x  lectures=%u  PCs=%d%s\n",
            g_rmap_pclo, g_rmap_pchi, g_rmap_tot, g_rmap_n,
            g_rmap_n >= RMAP_PCS ? "  *** SATUREE ***" : "");
    for (int i = 0; i < g_rmap_n; i++) {
        fprintf(stderr, "[c54x] RMAP   PC=0x%04x n=%-7u lit 0x%04x..0x%04x  ex:",
                g_rmap[i].pc, g_rmap[i].n, g_rmap[i].amn, g_rmap[i].amx);
        for (int k = 0; k < g_rmap[i].na && k < 8; k++)
            fprintf(stderr, " %04x", g_rmap[i].a[k]);
        fprintf(stderr, "\n");
    }
}

static void rmap_note(uint16_t addr, uint16_t pc)
{
    if (g_rmap_on < 0) {
        const char *e = getenv("CALYPSO_RMAP");
        g_rmap_on = (e && atoi(e) > 0) ? 1 : 0;
        const char *lo = getenv("CALYPSO_RMAP_PCLO"), *hi = getenv("CALYPSO_RMAP_PCHI");
        g_rmap_pclo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x9f00;
        g_rmap_pchi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x9fff;
        if (g_rmap_on)
            fprintf(stderr, "[c54x] RMAP armed PC 0x%04x..0x%04x\n", g_rmap_pclo, g_rmap_pchi);
    }
    if (!g_rmap_on || pc < g_rmap_pclo || pc > g_rmap_pchi) return;

    int i;
    for (i = 0; i < g_rmap_n; i++) if (g_rmap[i].pc == pc) break;
    if (i == g_rmap_n) {
        if (g_rmap_n >= RMAP_PCS) return;
        g_rmap_n++;
        g_rmap[i].pc = pc; g_rmap[i].n = 0; g_rmap[i].na = 0;
        g_rmap[i].amn = 0xffff; g_rmap[i].amx = 0;
    }
    g_rmap[i].n++;
    if (addr < g_rmap[i].amn) g_rmap[i].amn = addr;
    if (addr > g_rmap[i].amx) g_rmap[i].amx = addr;
    if (g_rmap[i].na < 8) {
        int seen = 0;
        for (int k = 0; k < g_rmap[i].na; k++) if (g_rmap[i].a[k] == addr) { seen = 1; break; }
        if (!seen) g_rmap[i].a[g_rmap[i].na++] = addr;
    }
    if (++g_rmap_tot % 5000 == 0) rmap_dump();
}

static uint16_t data_read_locked(C54xState *s, uint16_t addr)
{
    {   /* ─────────────────────────────────────────────────────────────────────
         * [2026-08-03] DTASKD-WATCH, patte 4/4 — CALYPSO_DTASKD_WATCH=1, defaut 0.
         * LECTURE SEULE, plafonnee. Pattes 1-2 dans calypso_trx.c, patte 3 dans
         * data_write_locked.
         *
         * CE QU'ON CHERCHE. Mesure du run de 19:22 (`CALYPSO_DISPATCH_PROBE=1`) :
         * la file de taches du DSP ne recoit JAMAIS qu'un seul handler, `0xb5a1`,
         * 32 559 fois ; le helper index->handler `0xa9ea` n'est appele qu'UNE fois
         * sur tout le run, et avec l'index 42 (DESARMEMENT). L'index 41 (armement
         * RX, `0xa5cd`) n'est jamais demande, et ni ses quatre sites d'appel, ni
         * les entrees de bloc, ni les chargeurs ne sont atteints.
         *
         * Autrement dit le chemin NB/CCCH n'est jamais mis dans la file. La
         * question descend donc d'un cran : **le DSP LIT-IL seulement la cellule
         * ou l'ARM depose la tache ?** Si le ROM ne lit jamais `d_task_d`, il
         * n'apprend jamais qu'une tache NB existe, et tout le reste en decoule
         * mecaniquement — inutile de chercher plus bas.
         *
         * CELLULES (pages W, cote MCU->DSP ; cf. dsp_api.h:22-23) :
         *     W p0 = data[0x0800]     W p1 = data[0x0814]
         * On surveille AUSSI `d_task_md` (data[0x0804]/[0x0818]), la tache que le
         * DSP consomme REELLEMENT aujourd'hui : c'est le temoin de comparaison.
         * Sans lui, une patte 4 muette serait ambigue entre « le DSP ne lit pas
         * d_task_d » et « le DSP ne lit rien du tout dans la page W ».
         *
         * LECTURE DU RESULTAT :
         *   md lu, d lu       -> le DSP voit la tache NB : le verrou est apres.
         *   md lu, d PAS lu   -> RACINE. Le ROM ignore d_task_d dans ce chemin.
         *   ni l'un ni l'autre-> la page W n'est pas lue du tout ; remonter au
         *                        selecteur 0xaad5 et a ce qui alimente 0xaac3. */
        static int _dw = -1;
        if (_dw < 0) {
            _dw = calypso_gate("CALYPSO_DTASKD_WATCH", 0);
            if (_dw) {
                fprintf(stderr, "[dtaskd] patte 4/4 armee (lectures DSP page W) : "
                        "d_task_d=data[0x0800]/[0x0814]  d_task_md=data[0x0804]/[0x0818]\n");
                fflush(stderr);
            }
        }
        if (_dw && (addr == 0x0800 || addr == 0x0814 ||
                    addr == 0x0804 || addr == 0x0818)) {
            static unsigned long long _nd = 0, _nmd = 0;
            int is_d = (addr == 0x0800 || addr == 0x0814);
            unsigned long long _n = is_d ? ++_nd : ++_nmd;
            if (_n <= 30 || (_n % 5000) == 0) {
                fprintf(stderr,
                        "[dtaskd] DSP<RD  %-9s data[0x%04x] = 0x%04x  "
                        "(d_task_d lus=%llu  d_task_md lus=%llu)  PC=0x%04x insn=%u\n",
                        is_d ? "d_task_d" : "d_task_md", addr, s->data[addr],
                        _nd, _nmd, s->last_exec_pc, s->insn_count);
                fflush(stderr);
            }
        }
    }
    rmap_note(addr, s->pc);
    {   /* [2026-07-28] WZREAD : voir en-tete du patch (gate CALYPSO_WZWRITE). */
        static int _wr = -1; static unsigned _wrn = 0;
        if (_wr < 0) _wr = calypso_gate("CALYPSO_WZWRITE", 0);
        if (_wr && addr == 0x2c00 && s->pc == 0xa07c && _wrn < 40) {
            /* v3 : seule la lecture du noyau MAC nous interesse (0x9aba = boucle
             * de normalisation, bruyante et deja caracterisee en B4B). */
            _wrn++;
            fprintf(stderr, "[c54x] WZREAD  data[0x%04x] = 0x%04x PC=0x%04x op=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    addr, s->data[addr], s->pc, prog_fetch(s, s->pc),
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        }
    }
    {   /* [2026-07-28] DMAWATCH (lecture) : voir en-tete du patch. */
        static int _dw2 = -1; static unsigned _dwr = 0;
        if (_dw2 < 0) _dw2 = calypso_gate("CALYPSO_DMAWATCH", 0);
        if (_dw2 && addr >= 0x0054 && addr <= 0x0057 && _dwr < 40) {
            _dwr++;
            const char *_nm = (addr==0x0054) ? "DMPREC?(modele:DMSA)" :
                              (addr==0x0055) ? "DMSA?(modele:DMSDI)" :
                              (addr==0x0056) ? "DMSDI?" : "DMSDN";
            fprintf(stderr, "[c54x] DMAWATCH RD 0x%04x %-20s = 0x%04x PC=0x%04x insn=%u\n",
                    addr, _nm, s->data[addr], s->pc, s->insn_count);
        }
    }
    {   /* [2026-07-28] DEMODRD : voir en-tete du patch. */
        static int _dr = -1; static unsigned _drn = 0;
        if (_dr < 0) _dr = calypso_gate("CALYPSO_DEMODRD", 0);
        if (_dr && _drn < 60 && s->pc == 0x9fb5) {   /* seule la lecture des echantillons */
            _drn++;
            fprintf(stderr, "[c54x] DEMODRD PC=0x%04x XPC=%u op=0x%04x lit data[0x%04x]=0x%04x "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "BK=%04x | 5 mots: %04x %04x %04x %04x %04x | insn=%u\n",
                    s->pc, (unsigned)s->xpc, prog_fetch(s, s->pc), addr, s->data[addr],
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    s->ar[6], s->ar[7], s->bk,
                    s->data[(uint16_t)(addr+0)], s->data[(uint16_t)(addr+1)],
                    s->data[(uint16_t)(addr+2)], s->data[(uint16_t)(addr+3)],
                    s->data[(uint16_t)(addr+4)], s->insn_count);
        }
    }
    {   /* [2026-07-27] SLOTSRC-RD : quelle adresse contient le stub 0xab38 ? */
        static int _sr = -1; static unsigned _srn = 0;
        if (_sr < 0) _sr = calypso_gate("CALYPSO_SLOTSRC", 0);
        if (_sr && _srn < 40 && s->data[addr] == 0xab38) {
            _srn++;
            fprintf(stderr, "[c54x] SLOTSRC-RD data[0x%04x] = 0xab38 (STUB) lu PC=0x%04x insn=%u\n",
                    addr, s->pc, s->insn_count);
        }
    }
    flow_log("R", addr, s->data[addr], s->pc, s->insn_count);
    read_stats_record(addr);
    /* MEM-WATCH-2B80 (2026-07-02, gated CALYPSO_MEM_WATCH_2B80) : le correlateur
     * FB (PC=0xee38) lit data[0x2b97] via AR3 (STM hardcode ROM), region
     * [0x2b80,0x2c00) distincte du buffer DMA BSP [0x2a00,0x2b28). Hypothese :
     * cette region n'est jamais peuplee (reste a 0) -> le MAC accumule zero en
     * boucle. Log tout READ dans cette fenetre (valeur, PC), cap 200. */
    if (addr >= 0x2b80 && addr < 0x2c00) {
        static int mw2b80_en = -1;
        if (mw2b80_en < 0) mw2b80_en = calypso_gate("CALYPSO_MEM_WATCH_2B80", 0);
        if (mw2b80_en) {
            static unsigned mw2b80_n = 0;
            if (mw2b80_n < 200) {
                mw2b80_n++;
                fprintf(stderr, "[c54x] MEM-WATCH-2B80-RD data[0x%04x]=0x%04x "
                        "PC=0x%04x insn=%u\n", addr, s->data[addr], s->pc,
                        s->insn_count);
            }
        }
    }
    /* PROBE 2026-05-31 frame-IT : valeur FIGÉE des flags polled + qui polle.
     * La valeur lue (jamais changée) = ce que le BSP doit produire/toggler. */
    if (addr == 0x006e || addr == 0x585f || addr == 0x8a44) {
        static uint32_t fr_n = 0;
        if (fr_n < 24) {
            fprintf(stderr, "[c54x] FLAGRD data[0x%04x]=0x%04x PC=0x%04x A=0x%04x "
                    "TC=%d insn=%u\n", addr, s->data[addr], s->pc,
                    (uint16_t)(s->a & 0xFFFF), !!(s->st0 & ST0_TC), s->insn_count);
            fr_n++;
        }
        /* one-shot : dump overlay de la loop 0x010b (hors dump ROM) + état IT
         * (IFR/IMR/INTM) pendant le spin → tranche source-dead vs IMR-masqué
         * vs INTM-collé (review c web). */
        static int dumped_010b = 0;
        if (addr == 0x006e && s->pc == 0x010b && !dumped_010b) {
            dumped_010b = 1;
            fprintf(stderr, "[c54x] SPIN-IT @0x010b IFR=0x%04x IMR=0x%04x INTM=%d "
                    "(INT3 bit3 IFR=%d IMR=%d ; BRINT0 bit5 IFR=%d IMR=%d) insn=%u\n",
                    s->ifr, s->imr, !!(s->st1 & ST1_INTM),
                    !!(s->ifr&(1<<3)), !!(s->imr&(1<<3)),
                    !!(s->ifr&(1<<5)), !!(s->imr&(1<<5)), s->insn_count);
            fprintf(stderr, "[c54x] OVERLAY-DUMP prog[0x0100..0x0118] (loop poll 0x006e):\n");
            for (uint16_t a = 0x0100; a <= 0x0118; a++)
                fprintf(stderr, "[c54x]   prog[0x%04x]=0x%04x\n", a, prog_fetch(s, a));
        }
    }
    /* D_TASK_MD-RD probe : trace DSP reads of d_task_md (write page 0
     * @ data[0x0804], write page 1 @ data[0x0818]). The DSP dispatcher
     * reads task_md then branches to FB / SB / ALLC / etc. routines.
     * If only one PC reads it, that's the single dispatcher. Capped 30. */
    if ((addr == 0x0804 || addr == 0x0818) && calypso_debug_enabled("D_TASK_MD-RD")) {
        /* UNCAP (gated par token) + EA/page/val + Δ depuis le write ARM=5.
         * Si après le write (Δ>0) la valeur lue ≠ 5 → le 5 n'atteint pas cette
         * EA : compare ARM5_EA (EA écrite par l'ARM) vs EA (EA lue par le DSP).
         *  - mêmes EA, val≠5 → timing/ordre (read avant write dans la frame)
         *  - EA ≠ ARM5_EA (stride page) → parité de flip w_page/r_page. */
        fprintf(stderr,
                "[c54x] D_TASK_MD-RD EA=0x%04x page=%d val=0x%04x "
                "ARM5_EA=0x%04x dArm5=%lld PC=0x%04x insn=%u\n",
                addr, (addr == 0x0804) ? 0 : 1, s->data[addr],
                g_arm_taskmd5_ea,
                (long long)((int64_t)s->insn_count - (int64_t)g_arm_taskmd5_insn),
                s->pc, s->insn_count);
    }
    /* WATCH-RD-ADDR (générique, gated CALYPSO_DEBUG=WATCH-RD + env
     * CALYPSO_WATCH_RD_ADDR=0xNNNN) : log toute LECTURE d'une adresse data
     * arbitraire (PC + valeur lue + insn) — voir QUI lit une cellule
     * pointeur-dispatcher (ex. 0x3af7) et AVEC QUELLE valeur (0 avant
     * peuplement vs valeur valide après). Symétrique du WATCH-WR. */
    {
        static int watch_rd_addr = -1;
        if (watch_rd_addr < 0) {
            const char *e = getenv("CALYPSO_WATCH_RD_ADDR");
            watch_rd_addr = (e && *e) ? (int)strtol(e, NULL, 0) : 0;
        }
        if (watch_rd_addr && addr == (uint16_t)watch_rd_addr) {
            C54_DBG("WATCH-RD",
                "WATCH-RD data[0x%04x] = 0x%04x PC=0x%04x DP=0x%03x insn=%u",
                addr, s->data[addr], s->pc, (s->st0 & 0x1FF),
                (unsigned)s->insn_count);
        }
    }
    /* DISP-POLL (CALYPSO_DEBUG=DISP-POLL) : le busy-loop dispatcher (d1xx↔da0d)
     * polle la zone flag DARAM[0x60-0x70]. On veut voir EN STEADY-STATE (post
     * +1.9s) CE qu'il lit (quel slot) et si ce flag devient jamais non-zéro
     * (= qqn le pose). Throttlé : 1ère/40000 par slot pour éviter l'explosion
     * du busy-loop, + TOUTE valeur non-zéro (l'évènement qui compte). */
    if (addr >= 0x0060 && addr <= 0x0070 && calypso_debug_enabled("DISP-POLL")) {
        static uint64_t poll_n = 0;
        uint16_t v = s->data[addr];
        if (v != 0 || (poll_n++ % 40000) == 0)
            fprintf(stderr,
                "[c54x] DISP-POLL-RD data[0x%04x]=0x%04x PC=0x%04x INTM=%d insn=%u%s\n",
                addr, v, s->pc, !!(s->st1 & ST1_INTM), s->insn_count,
                v ? "  <-- NON-ZERO (flag posé !)" : "");
    }
    /* FBDB-PROBE read 0x3DC0 (= SARAM flag polled by fc63 BITF).
     * Env CALYPSO_FBDB_PROBE=1. Logs first 30 reads + each 10000th. */
    if (addr == 0x3DC0 && g_fbdb_probe_enabled > 0) {
        fbdb_probe_read_3dc0(addr, s->data[addr], s->pc, s->insn_count);
    }
    /* D_BURST_D_W probe : DSP lit db_w->d_burst_d ?
     * 0x0801 (W_PAGE_0 + offset 1), 0x0815 (W_PAGE_1 + offset 1).
     * Si DSP read voit 0,1,2,3 séquentiel → ARM écrit correctement db_w.
     * Si DSP read voit 0 toujours → ARM ne configure pas burst_id.
     * Si DSP ne lit jamais → DSP ne consulte pas db_w pour le burst sequence. */
    if (addr == 0x0801 || addr == 0x0815) {
        static uint64_t dbw_total[2];
        static uint64_t dbw_per_val[2][16];
        static uint64_t dbw_last_log[2];
        static uint16_t dbw_last_val[2];
        int page = (addr == 0x0815) ? 1 : 0;
        uint16_t cur_val = s->data[addr] & 0xF;
        dbw_total[page]++;
        if (cur_val < 16) dbw_per_val[page][cur_val]++;
        bool changed = (cur_val != dbw_last_val[page]);
        dbw_last_val[page] = cur_val;
        if (dbw_total[page] <= 100 || changed
            || (s->insn_count - dbw_last_log[page]) > 1000000) {
            fprintf(stderr,
                    "[c54x] D_BURST_D_W-RD page=%d #%llu addr=0x%04x "
                    "val=0x%04x exec_pc=0x%04x insn=%u\n",
                    page, (unsigned long long)dbw_total[page], addr,
                    s->data[addr], s->last_exec_pc, s->insn_count);
            dbw_last_log[page] = s->insn_count;
        }
        /* Summary toutes les 50000 reads : histogramme valeurs lues */
        if ((dbw_total[page] % 50000) == 0) {
            if (calypso_debug_enabled("D_BURST_D_W-SUMMARY")) fprintf(stderr,
                    "[c54x] D_BURST_D_W-SUMMARY page=%d total=%llu "
                    "val[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                    page, (unsigned long long)dbw_total[page],
                    (unsigned long long)dbw_per_val[page][0],
                    (unsigned long long)dbw_per_val[page][1],
                    (unsigned long long)dbw_per_val[page][2],
                    (unsigned long long)dbw_per_val[page][3],
                    (unsigned long long)(dbw_total[page]
                        - dbw_per_val[page][0] - dbw_per_val[page][1]
                        - dbw_per_val[page][2] - dbw_per_val[page][3]));
        }
    }
    /* PC-histogram pour identifier la routine PM. Deux ranges :
     *   [0x3fb0..0x3fbf] = buffer BSP (samples I/Q)
     *   [0x3dcf..0x3dd5] = buffer scratch dominant (78k+52k reads observés)
     * Compte par PC, dump top-10 toutes les 50k reads dans chaque range.
     * Plus compteur d'entrée par PC dominant pour distinguer
     * "PM cassée" vs "PM jamais appelée" (vu IRQ rate 1.5 Hz). */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static uint32_t pc_hist_3fb[65536];
        static uint32_t total_3fb;
        pc_hist_3fb[s->pc]++;
        total_3fb++;
        if ((total_3fb % 50000) == 0) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3fb[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            if (calypso_debug_enabled("PC-HIST-3FB")) fprintf(stderr, "[c54x] PC-HIST-3FB total=%u :", total_3fb);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    if (addr >= 0x3dcf && addr <= 0x3dd5) {
        static uint32_t pc_hist_3dd[65536];
        static uint32_t total_3dd;
        pc_hist_3dd[s->pc]++;
        total_3dd++;
        if ((total_3dd % 50000) == 0) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3dd[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            if (calypso_debug_enabled("PC-HIST-3DD")) fprintf(stderr, "[c54x] PC-HIST-3DD total=%u :", total_3dd);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    /* === CANARY-READ probe (2026-05-28 method 3) ===
     * Quand CALYPSO_BSP_INJECT_CANARY=1, le BSP overwrite tous les samples
     * avec 0xCAFE. Si le DSP lit 0xCAFE depuis une addr, c'est que cette
     * addr est dans son chemin de lecture des samples = la vraie addr cible
     * pour CALYPSO_BSP_DARAM_ADDR. Trace cap 100 pour eviter le flood. */
    if (addr < 0x4000) {
        uint16_t v = s->data[addr];
        if (v == 0xCAFE) {
            static unsigned canary_log;
            const unsigned LIMIT = 100;
            if (canary_log < LIMIT) {
                fprintf(stderr,
                        "[c54x] CANARY-READ #%u addr=0x%04x PC=0x%04x "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        canary_log, addr, s->pc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                        s->insn_count);
                canary_log++;
                if (canary_log == LIMIT)
                    fprintf(stderr, "[c54x] CANARY-READ capped at %u\n", LIMIT);
            }
        }
    }

    /* Watch the mailbox slots that the firmware polls at PROM0 0xb41a
     * (LDU *(0x0ffe), A then BACC A) and 0xb41c (CMPM *(0x0fff), 4).
     * If these stay zero / 0x10 forever, ARM never wrote them. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x0ffc || addr == 0x0ffd) {
        static unsigned watch_count;
        static uint16_t last_vd[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
        int widx = (addr == 0x0fff) ? 0 : (addr == 0x0ffe) ? 1 : (addr == 0x0ffd) ? 2 : 3;
        uint16_t vd = s->data[addr];
        watch_count++;
        /* LOG-ON-CHANGE (anti-spam) : premiers 60 + tout CHANGEMENT de valeur. Le spin
         * bootloader 0xb424 lit data[0x0fff]=0x0001 1.36 Md de fois → l'ancien
         * `%10000` crachait 135k lignes (20 Mo, QEMU ramé, osmocon LOST). On garde le
         * signal exact (transition IDLE 0x0001 → commande 0x0002/0x0004) sans le bruit. */
        if (watch_count <= 60 || vd != last_vd[widx]) {
            uint16_t va = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0xDEAD;
            fprintf(stderr,
                    "[c54x] WATCH-READ #%u data[0x%04x] data=0x%04x api_ram=0x%04x api_set=%d PC=0x%04x insn=%u\n",
                    watch_count, addr, vd, va, s->api_ram ? 1 : 0, s->pc, s->insn_count);
        }
        last_vd[widx] = vd;
    }
    /* Wait-loop diagnostic: 0x3dd0 was found to absorb ~99.5 % of DARAM
     * reads after the first ~500k reads — the DSP is stuck polling it.
     * Log the first PCs and then sample once per million reads so we can
     * trace the loop without flooding the log. */
    if (addr == 0x3dd0) {
        static unsigned wait_log;
        static unsigned wait_seen;
        wait_seen++;
        if (wait_log < 20 || (wait_seen % 1000000) == 0) {
            wait_log++;
            fprintf(stderr,
                    "[c54x] WAIT-3DD0 #%u data[0x3dd0]=0x%04x PC=0x%04x AR2=%04x AR3=%04x insn=%u\n",
                    wait_seen, s->data[0x3dd0], s->pc,
                    s->ar[2], s->ar[3], s->insn_count);
        }
    }
    /* d_fb_det watch — REAL DSP word address is 0x08F8.
     * Mapping: ARM 0xFFD001F0 (BASE_API_NDB 0xFFD001A8 + 36 words × 2)
     *        = DSP word 0x0800 + 0x1F0/2 = 0x08F8.
     * Earlier 0x01F0 was the ARM byte-offset, NOT a DSP word address —
     * watching it logged unrelated DARAM 0x01F0 (junk). Now we trace
     * the real slot the firmware polls. */
    if (addr == 0x08F8) {
        static unsigned fb_read;
        if (fb_read++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-READ d_fb_det[0x08F8]=0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x08F8], s->pc, s->insn_count);
        }
    }
    /* === DIAG-FORCE-DARAM62 ===
     * Pinned diag (env-gated, default OFF) : when set, override the read
     * of daram[0x62] inside the dispatcher loop (PC ∈ 0xCC62..0xCC6F) to
     * return 1 instead of the actual stored value. Goal: force the
     * dispatcher's "branch if flag != 0" to fire and observe whether the
     * DSP escapes the loop and jumps to api[0x1f0c]=0x770c (the dispatch
     * target). Three outcomes (binary diagnostic) :
     *   - PC leaves cc62..cc6f → 0x770c and new code paths run :
     *     loop hypothesis correct, flag is the gate, INT3 ISR is the
     *     missing writer (next step: trace writes to confirm).
     *   - PC reaches 0x770c then returns to cc62 immediately :
     *     flag is set but handler bails because something else missing
     *     (a_cd[] init, NDB cell, ...).
     *   - No change : branch / compare is more subtle than read.
     * This is a force-test, not a fix — remove or env-leave-off after. */
    /* === DSP idle dispatcher trace (PC ∈ 0xCC62..0xCC6F) ===
     * The DSP gets stuck in this PROM0 loop polling task slots. Dump the
     * exact (PC, addr, value, AR2..AR5) for the first N reads so we can
     * see WHICH memory location the dispatcher inspects to decide whether
     * to branch out (task_md ? db_r ? api_ram ? other ?). Capped to keep
     * log size manageable.
     *
     * Captures all reads (DARAM + API RAM + MMR) so we don't miss the
     * critical poll address. */
    /* === BOOT-POLL-RD probe (2026-05-28) ===
     * Trace data reads in the boot polling loop (DARAM 0x00ed..0x00ff =
     * OVLY mirror of PROM0[0x70ed..0x70ff]). Per CLAUDE.md DSP_ROM_MAP :
     * "0x7026-0x71FF = Boot polling loop (writes API RAM tables)".
     * Goal : identifier QUELLE adresse le firmware polle pour décider
     * de sortir → quel signal devrait être émis par notre émulateur
     * peripherals pour débloquer légitimement (au lieu du drift AR1 bug).
     * Cap 300. */
    if (s->pc >= 0x00ed && s->pc <= 0x010f) {
        static unsigned bpr_log;
        const unsigned LIMIT = 300;
        if (bpr_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/oth";
            }
            fprintf(stderr,
                    "[c54x] BOOT-POLL-RD #%u PC=0x%04x op=0x%04x [%s 0x%04x]=0x%04x "
                    "AR1=%04x AR2=%04x AR3=%04x SP=%04x insn=%u\n",
                    bpr_log, s->pc, s->prog[s->pc], region, addr, v,
                    s->ar[1], s->ar[2], s->ar[3], s->sp, s->insn_count);
            bpr_log++;
            if (bpr_log == LIMIT) {
                fprintf(stderr, "[c54x] BOOT-POLL-RD log capped at %u\n", LIMIT);
            }
        }
    }

    /* === CORR-RD probe (2026-05-28) ===
     * Trace data reads in the FB-det correlator inner body
     * (PROM0[0x9aba..0x9abf] = RPTBD body of publish routine at 0x9aaf+).
     * Goal : identifier QUELLE zone DARAM le correlateur lit. Si addr ∈
     * [0x3fb0..0x3fbf] → bonne zone (BSP RX), valeur = sample. Si addr
     * ailleurs (low DARAM, MMR, ...) → bug d'addressing : correlateur
     * regarde un buffer vide → A reste à 0 → publish 0 → no lock.
     * Cap 200. */
    if (s->pc >= 0x9aba && s->pc <= 0x9abf) {
        static unsigned cr_log;
        const unsigned LIMIT = 200;
        if (cr_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/oth";
            }
            fprintf(stderr,
                    "[c54x] CORR-RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x A_lo=%04x B_lo=%04x insn=%u\n",
                    cr_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    (uint16_t)(s->a & 0xFFFF),
                    (uint16_t)(s->b & 0xFFFF),
                    s->insn_count);
            cr_log++;
            if (cr_log == LIMIT) {
                fprintf(stderr, "[c54x] CORR-RD log capped at %u\n", LIMIT);
            }
        }
    }
    if (s->pc >= 0xCC62 && s->pc <= 0xCC6F) {
        static unsigned idle_rd_log;
        const unsigned LIMIT = 200;
        if (idle_rd_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/other";
            }
            fprintf(stderr,
                    "[c54x] IDLE-DISP RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    idle_rd_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            idle_rd_log++;
            if (idle_rd_log == LIMIT) {
                fprintf(stderr,
                        "[c54x] IDLE-DISP RD log capped at %u — pattern should be visible above\n",
                        LIMIT);
            }
        }
    }

    /* === UPPER-DARAM RD HIST (2026-05-28) ===
     * Histogramme des reads addr ∈ [0x4000..0xFFFF] (zone haute DARAM).
     * Complementaire au DARAM RD HIST existant (low DARAM). Goal :
     * identifier sur quelles addresses upper le DSP polle ses samples.
     * Permet de trouver la vraie zone DARAM cible pour
     * CALYPSO_BSP_DARAM_ADDR sans brute-force. Dump top-16 tous les
     * 100k reads. Skip les 1M premieres insn pour eviter le bruit boot. */
    if (addr >= 0x4000 && s->insn_count > 1000000) {
        static unsigned uhist[0xC000]; /* 0x4000..0xFFFF = 48K words */
        static unsigned ureads;
        uhist[addr - 0x4000]++;
        ureads++;
        if ((ureads % 100000) == 0) {
            unsigned best[16] = {0}; uint16_t baddr[16] = {0};
            for (unsigned a = 0; a < 0xC000; a++) {
                unsigned c = uhist[a];
                if (c <= best[15]) continue;
                int p = 15;
                while (p > 0 && best[p-1] < c) {
                    best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                }
                best[p] = c; baddr[p] = (uint16_t)(0x4000 + a);
            }
            if (calypso_debug_enabled("UPPER-DARAM")) fprintf(stderr,
                    "[c54x] UPPER-DARAM RD HIST (reads=%u): ", ureads);
            for (int i = 0; i < 16 && best[i]; i++)
                fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
            fprintf(stderr, "\n");
        }
    }
    /* === DARAM discovery histogram ===
     * Track ALL data reads from DARAM (addr < 0x4000) regardless of PC.
     * The FB handler runs from both PROM0 (0xBD47) and DARAM overlay,
     * so filtering by PC misses critical reads. */
    if (addr < 0x4000 && addr >= 0x20) {  /* skip MMRs 0x00-0x1F */
        static unsigned hist[0x4000]; /* 16 KW DARAM */
        static unsigned reads;
        if (addr < 0x4000) {
            hist[addr]++;
            reads++;
            if ((reads % 50000) == 0) {
                /* find top-16 */
                unsigned best[16] = {0}; uint16_t baddr[16] = {0};
                for (uint16_t a = 0; a < 0x4000; a++) {
                    unsigned c = hist[a];
                    if (c <= best[15]) continue;
                    int p = 15;
                    while (p > 0 && best[p-1] < c) {
                        best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                    }
                    best[p] = c; baddr[p] = a;
                }
                if (calypso_debug_enabled("DARAM")) fprintf(stderr,
                        "[c54x] DARAM RD HIST (FB-det, reads=%u): ",
                        reads);
                for (int i = 0; i < 16 && best[i]; i++)
                    fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
                fprintf(stderr, "\n");
            }
        }
    }
    /* === BSP discovery: trace data reads in FB-det handler ===
     * Wide range over the PROM0 user-code area: handler PCs observed in
     * timeout traces cluster around 0x7e92..0x7eb8 (the FB-det inner
     * loop), so we widen the catch zone to 0x7000..0x7fff. */
    /* FB-det / dispatcher subroutine trace.
     * The 0x7e80..0x7eb8 wrapper CALLS into 0x81a5/0x81c8 with AR5=0x0e4c
     * (the FB sample buffer). Cover both ranges to catch both wrapper
     * polls and inner correlator reads. Skip the boot init phase. */
    if ((s->pc >= 0x7e80 && s->pc <= 0x7ec0) ||
        (s->pc >= 0x81a0 && s->pc <= 0x82ff)) {
        static int fbdet_rd_log = 0;
        if (s->insn_count > 50000000 && fbdet_rd_log < 2000) {
            uint16_t v;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
            else
                v = s->data[addr];
            C54_LOG("FBDET RD [0x%04x]=0x%04x PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                    addr, v, s->pc, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            fbdet_rd_log++;
        }
    }
    /* Log AR0..AR7 when entering FB-det subroutines to understand
     * what each AR points at (sample buffer? coeffs? status?). */
    if ((s->pc == 0x81a5 || s->pc == 0x81c8) && s->insn_count > 50000000) {
        static int ar_log = 0;
        if (ar_log < 10) {
            C54_LOG("FB-CALL PC=0x%04x AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x BK=%04x",
                    s->pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp, s->bk);
            ar_log++;
        }
    }
    /* d_spcx_rif (NDB word 2 = api 0xD6 = DSP data 0x08D6) */
    if (addr == 0x08D6) {
        static int spcx_rd = 0;
        if (spcx_rd < 32) {
            C54_LOG("d_spcx_rif RD = 0x%04x PC=0x%04x insn=%u",
                    s->api_ram ? s->api_ram[0xD6] : s->data[addr],
                    s->pc, s->insn_count);
            spcx_rd++;
        }
    }
    /* Log reads from API RAM at 0x08D4 (d_dsp_page).
     * [2026-07-29] Sondait 0x08E2 = d_dsp_state : elle regardait une cellule que
     * la ROM ne lit jamais, donc elle ne pouvait rien voir. */
    if (addr == 0x08D4) {
        static int dsp_page_log = 0;
        if (dsp_page_log < 50) {
            C54_LOG("d_dsp_page RD = 0x%04x PC=0x%04x insn=%u SP=0x%04x",
                    s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                    s->pc, s->insn_count, s->sp);
            dsp_page_log++;
        }
        /* FBWATCH PRODUCTEUR : le DSP re-lit-il d_dsp_page PAR-FRAME ? (env one-shot) */
        if (g_fbwatch_on < 0) g_fbwatch_on = calypso_gate("CALYPSO_FBWATCH", 0);
        if (g_fbwatch_on) {
            static unsigned wpg = 0;
            if (wpg++ < 60)
                fprintf(stderr, "[c54x] FBWATCH-PAGE-RD #%u val=0x%04x PC=0x%04x insn=%u\n",
                        wpg, s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                        s->pc, s->insn_count);
        }
    }
    /* Timer registers (0x0024-0x0026) — read returns current value */
    if (addr == TIM_ADDR) return s->data[TIM_ADDR];
    if (addr == PRD_ADDR) return s->data[PRD_ADDR];
    if (addr == TCR_ADDR) {
        /* TCR: PSC is read from bits 9:6, rest from stored value */
        uint16_t tcr = s->data[TCR_ADDR] & ~TCR_PSC_MASK;
        tcr |= (s->timer_psc & 0xF) << TCR_PSC_SHIFT;
        return tcr;
    }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:  return s->imr;
        case MMR_IFR:
        {
            static int ifr_log = 0;
            if ((s->ifr & 0x0020) && ifr_log < 10) {
                /* bit 5 = BRINT0 per C54X header (vec 21). */
                C54_LOG("IFR READ=0x%04x (BRINT0 pending) PC=0x%04x", s->ifr, s->pc);
                ifr_log++;
            }
            return s->ifr;
        }
        case MMR_ST0:  return s->st0;
        case MMR_ST1:  return s->st1;
        case MMR_AL:   return (uint16_t)(s->a & 0xFFFF);
        case MMR_AH:   return (uint16_t)((s->a >> 16) & 0xFFFF);
        case MMR_AG:   return (uint16_t)((s->a >> 32) & 0xFF);
        case MMR_BL:   return (uint16_t)(s->b & 0xFFFF);
        case MMR_BH:   return (uint16_t)((s->b >> 16) & 0xFFFF);
        case MMR_BG:   return (uint16_t)((s->b >> 32) & 0xFF);
        case MMR_T:    return s->t;
        case MMR_TRN:  return s->trn;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            return s->ar[addr - MMR_AR0];
        case MMR_SP:   return s->sp;
        case MMR_BK:   return s->bk;
        case MMR_BRC:  return s->brc;
        case MMR_RSA:  return s->rsa;
        case MMR_REA:  return s->rea;
        case MMR_PMST: return s->pmst;
        case MMR_XPC:  return s->xpc;
        default: return 0;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram) {
            uint16_t val = s->api_ram[addr - C54X_API_BASE];
            /* Log ALL API reads during interrupt handler (first 100) */
            static int api_rd_log = 0;
            if (api_rd_log < 100 && s->insn_count > 66000) {
                C54_LOG("API RD [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                        addr, val, s->pc, s->insn_count);
                api_rd_log++;
            }
            return val;
        }
    }

    /* Log data reads during SINT17 handler (PC in 0xFFC0-0xFFFF) */
    if (s->pc >= 0xFFC0 && s->insn_count > 66090) {
        static int handler_rd_log = 0;
        if (handler_rd_log < 30) {
            C54_LOG("H_RD [0x%04x]=0x%04x PC=0x%04x", addr, s->data[addr], s->pc);
            handler_rd_log++;
        }
    }

    /* [2026-07-26 WF golive-handshake] PROBE-3FAD-GATE : capture EXACTE de la
     * valeur lue par le BITF *(0x3fad),#0x8000 @0x8753 (le SEUL verrou du
     * dispatcher FB : CC 0xa0a0 -> kernel 0xa076). f930=CC (call cond, TC=1),
     * donc les CALL 0x90b0/b8/c8/ed/914d reviennent : rien ne bloque le sweep,
     * seul bit15 de 0x3fad decide. Si val&0x8000 ici => TC=1 => kernel PRIS.
     * Si val&0x8000==0 alors qu'RX-FBFLAGS l'a pose => un clearer per-frame
     * (XPC=0 0xace8/0xad04/0xad24 ou overlay XPC=2 0x28040/0x282d0 : 76f8 3fad
     * 0000) l'a efface entre l'ecriture BSP et le sweep DSP. Cap 200, gate
     * CALYPSO_PROBE_3FAD_GATE. Zero cout/effet quand OFF. */
    if (addr == 0x3fad && s->pc == 0x8753 && calypso_rxfb_fired) {  /* [fix v3] gate = RX-FBFLAGS a REELLEMENT pose 3fad bit15 (definitif) */
        static int p3_en = -1; static unsigned p3_n = 0;
        if (p3_en < 0) p3_en = calypso_gate("CALYPSO_PROBE_3FAD_GATE", 0);
        if (p3_en && p3_n < 200) {
            p3_n++;
            fprintf(stderr, "[c54x] PROBE-3FAD-GATE @0x8753 val=0x%04x bit15=%d "
                    "(=>TC/kernel) task_md0=0x%04x xpc=%d insn=%u\n",
                    s->data[0x3fad], !!(s->data[0x3fad] & 0x8000),
                    s->data[0x0804], s->xpc, s->insn_count);
        }
        /* [2026-07-26 FIX v3] le clearer per-frame (76f8 3fad 0000) efface bit15
         * entre l ecriture BSP et ce sweep -> on le RE-POSE ici, sur le chemin de
         * LECTURE DSP, quand RX-FBFLAGS l a arme -> le BITF @0x8753 voit TC=1 ->
         * CC 0xa0a0 -> kernel 0xa076. Gate CALYPSO_FORCE_3FAD_KERNEL. */
        /* @BEQUILLE — FORCE_3FAD_KERNEL  (CALYPSO_FORCE_3FAD_KERNEL, EXISTS, defaut OFF)
         *   masque  : le producteur du flag "burst pret" data[0x3fad] bit15. Un clearer
         *             per-frame (76f8 3fad 0000) l'efface entre l'ecriture BSP et le sweep
         *             DSP ; on le RE-POSE sur le chemin de lecture @0x8753.
         *   retirer : quand le poseur natif (chaine RX/BRINT0) tient bit15 jusqu'au BITF
         *             @0x8753 — c.-a-d. quand PROBE_3FAD_GATE voit bit15=1 sans ce gate.
         *   NB      : conditionne par calypso_rxfb_fired, pose par CALYPSO_RX_FBFLAGS.
         */
        { static int _fk = -1; if (_fk < 0) _fk = calypso_gate("CALYPSO_FORCE_3FAD_KERNEL", 0);
          if (_fk && calypso_rxfb_fired) s->data[0x3fad] |= 0x8000; }
    }

    return s->data[addr];
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val);

/* === Stack-write ring (ORPHAN trace 2026-05-30) : capture les data_write vers
 * la zone pile, pour nommer AU POPM ST0 @0xf48b qui a écrit le slot lu (clobber
 * sous SP, famille circulaire/dual-operand). No-fire au dump = personne n'a
 * écrit le slot dans la frame → slot STALE → SP désaligné (POP sans PUSH). */
StkwEv   g_stkw_ring[STKW_RING_N];
unsigned g_stkw_idx  = 0;
int      g_orphan_on = -1;
static void stkw_rec(C54xState *s, uint16_t addr, uint16_t val)
{
    if (g_orphan_on < 0) g_orphan_on = calypso_gate("CALYPSO_ORPHAN", 0);  /* env dédiée (hors CALYPSO_DEBUG → master reste 0, anti-Heisenbug) */
    if (!g_orphan_on) return;
    if (addr < 0x1000 || addr > 0x6000) return;   /* zone pile (SP dérive 0x1100→0x56xx) */
    StkwEv *e = &g_stkw_ring[g_stkw_idx % STKW_RING_N];
    e->addr = addr; e->val = val; e->pc = s->pc; e->op = prog_fetch(s, s->pc);
    g_stkw_idx++;
    /* Track-value : nomme le CALL/push qui pose la valeur orpheline (défaut
     * 0x3125, override CALYPSO_TRACK_STKVAL=0xNNNN) → corréler avec RCD@0x765c
     * et POPM@0xf48b en ordre insn. */
    {
        static int tval = -2;
        if (tval == -2) {
            const char *te = getenv("CALYPSO_TRACK_STKVAL");
            tval = (te && *te) ? (int)strtol(te, NULL, 0) : 0x3125;
        }
        if (tval >= 0 && val == (uint16_t)tval)
            fprintf(stderr, "[c54x] STK-PUSH val=0x%04x addr=0x%04x PC=0x%04x op=0x%04x SP=0x%04x insn=%u\n",
                    val, addr, s->pc, prog_fetch(s, s->pc), s->sp, s->insn_count);
    }
}

/* [2026-08-22] SCRATCH-WR (CALYPSO_SCRATCH_WR, defaut OFF) — la table de
 * coefficients du FIRS est-elle JAMAIS ecrite ?
 * FIRS-COEF a montre prog[0x61..0x66]=0xF4E4 (remplissage) ET data[0x61..0x66]=0
 * au moment de l execution. Le PROGRAMME 0x0000-0x6FFF n est couvert par aucun
 * fichier ROM (PROM0 debute a 0x7000) — c est exactement la que tombe pmad=0x61.
 *   aucune ecriture   -> segment ROM manquant ;
 *   ecriture tardive  -> ordonnancement.
 * On date (insn) et on situe (PC) chaque ecriture des DEUX espaces, avec bilan
 * periodique pour distinguer « rien ecrit » de « sonde muette ». LECTURE SEULE. */
static int scratchwr_on(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_SCRATCH_WR", 0);
        fprintf(stderr, "[c54x] SCRATCH-WR %s : ecritures dans 0x0060-0x007F "
                "(scratch-pad DARAM + espace PROGRAMME via MVDP) — zone des "
                "coefficients du FIRS (pmad=0x0061)\n",
                g ? "ACTIVE" : "INACTIVE (defaut)");
    }
    return g;
}

void scratchwr_note(C54xState *s, uint16_t a, uint16_t v, const char *espace)
{
    if (!scratchwr_on()) return;
    /* [2026-08-22] v2 : plafond PAR ADRESSE, jamais global. La v1 avait un
     * plafond global de 40 lignes, consomme par data[0x74] a insn ~5000-10000 ;
     * les ecritures des coefficients (MVDD a insn ~1376674) n ont donc jamais
     * ete affichees et j ai lu ce plafond comme une absence. */
    int zone = -1;
    if (a >= 0x0060 && a <= 0x007F)      zone = 0;   /* destination */
    else if (a >= 0x2CB0 && a <= 0x2CBF) zone = 1;   /* SOURCE de la table */
    if (zone < 0) return;

    static unsigned long long cum[2] = {0, 0};
    static unsigned long long cum_coef = 0;   /* 0x0061..0x0066 seulement */
    static unsigned long long cum_src  = 0;   /* 0x2CB9..0x2CBF seulement */
    static unsigned char shown[2][0x20];

    cum[zone]++;
    if (a >= 0x0061 && a <= 0x0066) cum_coef++;
    if (a >= 0x2CB9 && a <= 0x2CBF) cum_src++;

    unsigned idx = (zone == 0) ? (unsigned)(a - 0x0060) : (unsigned)(a - 0x2CB0);
    if (idx < 0x20 && shown[zone][idx] < 3) {
        shown[zone][idx]++;
        fprintf(stderr, "[c54x] SCRATCH-WR %s[0x%04x] <- 0x%04x PC=0x%04x insn=%u%s\n",
                espace, a, v, s->pc, s->insn_count,
                (a >= 0x0061 && a <= 0x0066) ? "  <<<< COEFFICIENTS (destination)" :
                (a >= 0x2CB9 && a <= 0x2CBF) ? "  <<<< SOURCE de la table" : "");
    }
    if (((cum[0] + cum[1]) % 500) == 0)
        fprintf(stderr, "[c54x] SCRATCH-WR bilan : scratch(0x60-0x7F)=%llu "
                "source(0x2CB0-0x2CBF)=%llu | dont coefficients(0x61-0x66)=%llu "
                "source utile(0x2CB9-0x2CBF)=%llu\n",
                (unsigned long long)cum[0], (unsigned long long)cum[1],
                (unsigned long long)cum_coef, (unsigned long long)cum_src);
}

void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    scratchwr_note(s, addr, val, "data");
    {   /* [2026-08-03] HANDLER-WATCH — CALYPSO_DISPATCH_PROBE=1, lecture seule.
         *
         * data[0x43d8] est le slot du HANDLER COURANT : le dispatcher fait
         *     0xb01c  ld   *(0x43d8), A
         *     0xb01e  cala A
         * et la trace du 03/08 montre qu'il y appelle systematiquement 0xab38,
         * le RET partage des slots vides. Toute la chaine RX se reduit donc a
         * « qui ecrit cette cellule, et pourquoi jamais 0xa5cd ».
         *
         * On surveille l'ECRITURE plutot qu'un PC : deux fois de suite j'ai vise
         * l'operande au lieu de l'instruction (0xb0f1/0xb0ff au lieu des `add`,
         * puis 0xbb01 au lieu de 0xbb00). Surveiller la cellule est insensible a
         * cette erreur — et attrape aussi tout ecrivain qu'on n'a pas identifie. */
        static int _hw = -1;
        if (_hw < 0) _hw = calypso_gate("CALYPSO_DISPATCH_PROBE", 0);
        if (_hw && addr == 0x43d8) {
            static unsigned long long _n = 0;
            uint16_t old = s->data[addr];
            if (old != val || _n < 8) {
                _n++;
                fprintf(stderr, "[dispatch] *** data[0x43d8] : 0x%04x -> 0x%04x "
                        "par PC=0x%04x%s  (data[0x43b0]=0x%04x) insn=%u\n",
                        old, val, s->pc,
                        (val == 0xa5cd) ? "  <<< ARMEMENT RX INSTALLE !" :
                        (val == 0xab38) ? "  (RET partage = slot vide)" : "",
                        s->data[0x43b0], s->insn_count);
            }
        }
    }
    /* [2026-07-29] Moniteur mailbox — voir calypso_mailbox.h. Placé EN TÊTE :
     * s->data[addr] contient encore l'ancienne valeur. Coût nul quand éteint
     * (test d'un int en ligne), et data_write est un chemin chaud. */
    calypso_mbx(MBX_DSP_WR, addr, val, s->data[addr], s->pc, 0, s->insn_count);

    /* [2026-07-27] WATCH-ACD : le DSP (opcode) ecrit-il a_cd et clobbe-t-il
     * l ecriture DIRECTE du shunt (SI1-4) apres un reset ? Gate CALYPSO_WATCH_ACD.
     *
     * [2026-07-30 - DEMENTI LE 04/08, CE PARAGRAPHE EST FAUX] BORNES CORRIGEES :
     * la fenetre etait 0x09D2..0x09E0, DECALEE DE
     * +2 sur la vraie zone. a_cd[15] occupe les mots DSP 0x09D0..0x09DE — cf. le
     * commentaire de A_CD-WR plus bas dans ce meme data_write, et l'antiseche
     * osmocom-bb include/calypso/dsp_api.h : « API a_cd[15]; // Header +
     * CCCH/SACCH downlink information ». L'ancienne fenetre ratait les DEUX mots
     * de HEADER et debordait de deux mots hors zone : son zero ne prouvait rien.
     *
     * NB : la sonde A_CD-WR (watch_write_zone_check, 0x09d0..0x09de) couvre deja
     * correctement la zone, n'a AUCUN gate et logue les 500 premieres ecritures.
     * C'est ELLE le juge de reference ; WATCH-ACD n'est qu'un doublon cible. */
    if (addr >= 0x09D0 && addr <= 0x09DE) {
        static int _wac = -1;
        if (_wac < 0) _wac = calypso_gate("CALYPSO_WATCH_ACD", 0);
        if (_wac) { static unsigned _nac = 0;
            if (_nac++ < 60)
                fprintf(stderr, "[c54x] WATCH-ACD DSP-opcode-write data[0x%04x]=0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count); }
    }
    /* [2026-07-26 golive-mac] WATCH-2A00 : trace toute ecriture OPCODE vers le
     * buffer IQ 0x2a00..0x2a07 (capture si le DSP lui-meme repose 0x12ed apres
     * feed_iq). feed_iq ecrit s->data[] EN DIRECT (hors data_write) -> invisible
     * ici : si 0x12ed apparait ici c'est un writer opcode DSP. s=%p pour comparer
     * avec le pointeur feed_iq. Gate CALYPSO_WATCH_2A00. */
    if (addr >= 0x2a00 && addr <= 0x2a07) {
        static int _w2a = -1;
        if (_w2a < 0) _w2a = calypso_gate("CALYPSO_WATCH_2A00", 0);
        if (_w2a) {
            static unsigned _n2a = 0;
            if (_n2a++ < 80)
                fprintf(stderr, "[c54x] WATCH-2A00 opcode-write data[0x%04x]=0x%04x "
                        "(was 0x%04x) PC=0x%04x s=%p insn=%u\n",
                        addr, val, s->data[addr], s->pc, (void*)s, s->insn_count);
        }
    }
    /* [2026-07-27 golive-mac] WATCH-9200 : les cellules 0x9210-0x9218 / 0x9260-0x9261
     * sont lues par le demod (0x9fab-0x9fb5) comme source IQ mais restent CONSTANTES
     * (0xff06/0x04a3) -> workzone 0x2a00 plat. Trace TOUTE ecriture opcode vers cette
     * region pour voir si qqun l'alimente per-frame (et d'ou). Gate CALYPSO_WATCH_9200. */
    if ((addr >= 0x9210 && addr <= 0x9220) || (addr >= 0x9260 && addr <= 0x9262)) {
        static int _w92 = -1;
        if (_w92 < 0) _w92 = calypso_gate("CALYPSO_WATCH_9200", 0);
        if (_w92) {
            static unsigned _n92 = 0;
            if (_n92++ < 80)
                fprintf(stderr, "[c54x] WATCH-9200 opcode-write data[0x%04x]=0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* [2026-07-26 golive-mac] WATCH-RESULT : trace les ecritures OPCODE vers les
     * cellules resultat FB natif (= adresses shunt_legit, confirmees osmocom NDB) :
     * d_fb_det 0x08F8, a_sync_demod TOA/PM/ANGLE/SNR 0x08FA..0x08FD. Montre le
     * SHADOW que le correlateur natif produit, meme quand d_fb_det reste 0.
     * Gate CALYPSO_WATCH_RESULT. */
    if (addr >= 0x08F8 && addr <= 0x08FD) {
        static int _wr = -1;
        if (_wr < 0) _wr = calypso_gate("CALYPSO_WATCH_RESULT", 0);
        if (_wr) {
            static unsigned _nr = 0;
            const char *_nm = (addr==0x08F8)?"d_fb_det":(addr==0x08F9)?"d_fb_mode":
                              (addr==0x08FA)?"TOA":(addr==0x08FB)?"PM":
                              (addr==0x08FC)?"ANGLE":"SNR";
            if (_nr++ < 120)
                fprintf(stderr, "[c54x] WATCH-RESULT data[0x%04x]=%-8s 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, _nm, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* [2026-07-25] WATCH-0810 (gated CALYPSO_WATCH_0810) : trace toute ecriture
     * DSP-side a data[0x0810] (d_ctrl_system / B_TASK_ABORT bit15) avec le PC
     * auteur. Complete ARM-WRITE-0810 (cote trx). Ensemble : QUI repose bit15
     * apres le clear ARM ? (self-clear DSP @0xa549 attendu ; le wire CTRLSYS
     * ecrit s->data[] directement, hors data_write -> invisible ici = c'est LUI
     * le re-setter s'il n'apparait pas). Cap 200. */
    if (addr == 0x0810) {
        static int w810 = -1;
        if (w810 < 0) w810 = calypso_gate("CALYPSO_WATCH_0810", 0);
        if (w810) {
            static unsigned n810 = 0;
            if (n810++ < 200)
                fprintf(stderr, "[c54x] WATCH-0810 DSP-write data[0x0810]=0x%04x "
                        "(was 0x%04x) PC=0x%04x insn=%u\n",
                        val, s->data[0x0810], s->pc, s->insn_count);
        }
    }
    /* [2026-07-25] RANK1b FIX (workflow overlay, context-safe) : le prologue ISR
     * overlay 0x013b fait POPD *(0x3fcd) = depile l'adresse de retour HW 0x72d5
     * dans data[0x3fcd], puis 23 PSHM (sauvegarde contexte), puis PSHD *(0x3fcd)
     * + RET qui DEPILE data[0x3fcd] = retourne a 0x72d5 -> reboucle sur le
     * scheduler 0x7234 (self-loop). Le chemin NATIF doit retourner a 0xa4e4
     * (init lineaire ORM/RSBX/IMR -> dispatcher 0x8341 -> LUT FB -> correlateur
     * AR3=0x2a00). On substitue la valeur ecrite par 0x013b UNIQUEMENT (pas
     * l'epilogue 0x011e qui ecrit aussi 0x3fcd) : les 23 PSHM s'executent, SP
     * reste equilibre, le RET depile 0xa4e4 -> chemin natif SANS storm (vs le
     * PC-redirect qui sautait le contexte). Gate CALYPSO_FIX_3FCD (def off). */
    if (addr == 0x3fcd && s->pc == 0x013b) {
        /* @BEQUILLE — FIX_3FCD  (CALYPSO_FIX_3FCD, EXISTS, defaut OFF ; calypso_wire.env:=1)
         *   masque  : le prologue ISR overlay 0x013b depile une adresse de retour HW
         *             (0x72d5) au lieu de 0xa4e4 ; la branche reelle = un vectoring
         *             d'interruption qui empile la bonne adresse de retour.
         *   retirer : des que le frame-IT vectorise vers 0xa4e4 sans substitution
         *             (data[0x3fcd] vaut 0xa4e4 sans le gate).
         */
        static int f3f = -1;
        if (f3f < 0) f3f = calypso_gate("CALYPSO_FIX_3FCD", 0);
        if (f3f && val != 0xa4e4) {
            static unsigned f3n = 0;
            if (f3n++ < 8)
                fprintf(stderr, "[c54x] FIX-3FCD : data[0x3fcd] 0x%04x -> 0xa4e4 "
                        "(retour natif go-live, pas 0x72d5 self-loop) PC=0x013b insn=%u\n",
                        val, s->insn_count);
            val = 0xa4e4;
        }
    }

    /* WRITE-WATCH (2026-06-24, RO) : qui ECRIT 0x434f (FIFO write ptr),
     * 0x434e (FIFO read ptr), 0x3f6d (soft-vector go-live) — avec quel PC.
     * Tranche empiriquement ECRITURE-ACTIVE (par un chemin) vs residu/defaut :
     * Garde1 = FIFO init-vive vs jamais-setup ; Garde2 = soft-vector pose
     * activement vers 0xa4df vs valeur de reset. NE JAMAIS deduire ces valeurs
     * de la narration quand ce watch peut les LIRE. Cap 80. */
    if (addr == 0x434f || addr == 0x434e || addr == 0x3f6d) {
        static unsigned ww = 0;
        if (ww++ < 80)
            fprintf(stderr, "[c54x] WRITE-WATCH data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
    }
    /* F70-SETBIT1 (2026-06-24, RO) : data[0x3f70] bit1 = le flag qui fait sortir la
     * wait-loop (test 0xa4d4 -> RET si TC). Log UNIQUEMENT quand bit1 (0x0002) est
     * ECRIT -> si ca ne fire JAMAIS, le DSP ne pose jamais "frame prete" = racine
     * confirmee. Writers de 0x0002 : 0x710c/0xa5bd/0xb3ef/0xde9c. Cap 40. */
    if (addr == 0x3f70 && (val & 0x0002)) {
        static unsigned f70 = 0;
        if (f70++ < 40)
            fprintf(stderr, "[c54x] F70-SETBIT1 data[0x3f70] 0x%04x->0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x3f70], val, s->pc, s->insn_count);
    }
    /* VEC-INSTALL (2026-06-24, RO) : la table de vecteurs runtime vit en DARAM
     * a 0x0080 (IPTR=1, OVLY). Mesure live : vec19(FRAME)@0xcc et vec21(BRINT0)
     * @0xd4 sont des stubs RETE(0xf4eb)/NOP a froid, alors que vec17/20/22/24..
     * portent de vrais handlers (FB 0xf8xx). Question falsifiable (H-A vs H-B) :
     * le firmware installe-t-il JAMAIS un vrai branch au mot-0 de FRAME/BRINT0,
     * et a quel PC/insn ? Watch les 8 mots des 2 slots + tout mot-0 de la table
     * [0x80..0xFC] ou un FB-branch (0xf8xx) est ecrit. Cap 200. */
    if ((addr >= 0x00cc && addr <= 0x00cf) || (addr >= 0x00d4 && addr <= 0x00d7) ||
        ((addr >= 0x0080 && addr <= 0x00ff) && ((addr & 3) == 0) &&
         (val & 0xFF80) == 0xF880)) {
        static unsigned vi = 0;
        if (vi++ < 200) {
            int vec = (addr - 0x0080) / 4;
            int word = (int)((addr - 0x0080) & 3);
            fprintf(stderr, "[c54x] VEC-INSTALL vec%d@0x%04x w%d <- 0x%04x %s PC=0x%04x insn=%u\n",
                    vec, addr, word, val,
                    (vec==19?"<FRAME":(vec==21?"<BRINT0":"")),
                    s->pc, s->insn_count);
        }
    }
    /* TASKTAB-WR (revival dsp 2026-06-22, RO) : qui sème data[0x4c5b/0x4c5c]
     * (table de tâches lue par LD *(0x4c5c),A puis CALA ; devrait venir du
     * handler FRAME 0xA04C). 0 write = jamais semée → CALA A=0 → boot stub. */
    if (addr >= 0x4c5b && addr <= 0x4c5d) {
        static unsigned ttab_n = 0;
        if (ttab_n++ < 80)
            fprintf(stderr, "[c54x] TASKTAB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
    }
    /* === SENTINELLE cohérence ARM<->DSP (CALYPSO_FBDET_SENTINEL=1) ===
     * Force toute écriture DSP de d_fb_det (0x08f8) à 0xDEAD. L'ARM lit ce mot
     * via arm=0x01f0 -> s->dsp->data[0x08f8] (calypso_trx.c). La sonde "ARM RD
     * d_fb_det" (token TRX) montre alors :
     *   ARM lit 0xDEAD  -> mémoire COHÉRENTE (même cellule) -> H1 mort, c'est H3 (PM/seuil).
     *   ARM lit 0x0000  -> DÉSYNC ARM<->DSP -> H1 confirmé.
     * NE PAS combiner avec CALYPSO_FORCE_TOA (qui override la lecture 0x01f0). */
    {
        static int sent = -1;
        if (sent < 0) { const char *e = getenv("CALYPSO_FBDET_SENTINEL"); sent = e ? atoi(e) : 0;
            if (sent==1) fprintf(stderr, "[c54x] FBDET-SENTINEL=1 FORCE : data[0x08f8] forcé à 0xDEAD\n");
            else if (sent==2) fprintf(stderr, "[c54x] FBDET-SENTINEL=2 MONITOR : logge la vraie valeur écrite à 0x08f8 (pas de force)\n"); }
        /* [2026-09-03] FORCE d_fb_det SUPPRIMEE. Sous SHUNT_LEGIT, toute ecriture
         * DSP de data[0x08f8] etait reecrite a 1 des que gr-gsm avait decode la
         * SCH — le correlateur natif ecrivait 0, on transportait la detection de
         * l'hote par-dessus. Son annotation disait « retirer quand le correlateur
         * natif pose d_fb_det lui-meme » : c'est fait depuis le 2026-08-24
         * (d_fb_det 0->1 mesure 437 fois, ecrit par la mask-ROM a PC=0x79e4).
         * data[0x08f8] contient desormais ce que le DSP y met, point. */
        {
            /* [2026-07-26 RANK5] force a_pm (rxlev) sur le VRAI array lu par
             * l'ARM : calypso_trx.c lit s->dsp->data[off/2+0x800], PAS api_ram.
             * a_pm read page 0 = data[0x830..0x832], page 1 = data[0x844..0x846]
             * (ARM off 0x60/0x88 -> /2+0x800). Le DSP les ecrit a 0 (pas de vraie
             * mesure) -> on force la valeur calibree trf6151 -> rxlev stable. */
            {
                /* MODELE INTEGRATEUR RSSI HW — le vrai pm_meas natif.
                 * Sur vrai Calypso la tache PM (md=1) ne calcule PAS a_pm depuis les
                 * samples : le DSP zero-remplit la page resultat (PROM0 0xb446,
                 * `stl *AR1+,A` en rptb 80), puis un integrateur lit un REGISTRE HW de
                 * puissance cote ABB/RF et pose a_pm. Ce registre n'est pas dans l'ADC
                 * modelise ; on le MODELISE depuis la magnitude reelle du DL mesuree
                 * par le BSP. L'ancrage MAV_REF -> RF_REF est la calibration du
                 * frontend (comme le gain trf6151), pas une valeur decretee : deux
                 * signaux differents donnent deux a_pm differents.
                 *
                 * [2026-09-03] La branche concurrente `apm_for_rf(CALYPSO_TRF_TARGET_RF)`
                 * est SUPPRIMEE : elle posait une cible RF figee (-60 dBm), une
                 * constante decretee, et n'etait active que sous le parapluie
                 * SHUNT_LEGIT — donc jamais en natif. CALYPSO_TRF_RXLEV et
                 * CALYPSO_TRF_TARGET_RF disparaissent avec elle. Le modele de gain
                 * trf6151 (suivi par les writes TSP), lui, reste : c'est lui qui
                 * calibre la conversion ci-dessous. */
                static int rssi_on = -1;
                if (rssi_on < 0) {
                    rssi_on = calypso_gate("CALYPSO_PM_RSSI", 1);
                }
                /* DSP 33-36 : db_r = {..d_task_ra(7), a_serv_demod[4](8..11),
                 * a_pm[3](12..14), a_sch[5](15..19)}. a_pm read page 0 = word 12
                 * = data[base0x28+12+0x800]=data[0x834..0x836] ; page 1 =
                 * data[0x3C+12+0x800]=data[0x848..0x84A]. (0x830/0x844 = a_serv_demod!) */
                if (rssi_on && ((addr >= 0x0834 && addr <= 0x0836) ||
                            (addr >= 0x0848 && addr <= 0x084A))) {
                    val = calypso_bsp_rssi_apm();
                }
            }
        }
        if (sent && addr == 0x08f8) {
            uint16_t orig = val;
            if (sent == 1) val = 0xDEAD;   /* mode FORCE (test cohérence) */
            static unsigned sn = 0;
            if (sn++ < 40 || (sn % 2000) == 0)
                fprintf(stderr, "[c54x] FBDET-SENTINEL #%u DSP write d_fb_det[0x08f8] orig=0x%04x%s PC=0x%04x insn=%u\n",
                        sn, orig, (sent==1) ? " ->0xDEAD" : " (monitor)", s->pc, s->insn_count);
        }
    }
    stkw_rec(s, addr, val);   /* ORPHAN : ring écritures pile */
    /* ORPHAN : tracker stores directs zone [0x1100..0x1140] (vecteur légit vs
     * vierge). Slots au-dessus de SP_base → jamais touchés par un push. */
    if (addr >= STKSLOT_LO && addr <= STKSLOT_HI) {
        int _si = addr - STKSLOT_LO;
        g_stkslot_wpc[_si] = s->pc;
        g_stkslot_wop[_si] = prog_fetch(s, s->pc);
        g_stkslot_written[_si] = 1;
    }
    /* MVPD overlay occupancy : count writes to [0x0080..0x27FF] during
     * boot phase. Env-gated CALYPSO_MVPD_TRACE=1. */
    if (g_mvpd_trace_enabled > 0) {
        mvpd_trace_record(addr);
    }
    /* ANGLE-WR tracer (2026-05-30) : qui écrit a_sync_demod[ANGLE]=0x08fc
     * (et TOA=0x08fa, SNR=0x08fd) = la SORTIE du vrai détecteur de fréquence
     * FCCH. Capture A/B (corr complexe) + réf (AR3/4/5) au store → le vrai
     * site de corrélation fréquentielle. Cap 40. */
    /* FBMODE-WR : qui écrit d_fb_mode (0x08f9) et avec quelle valeur (large
     * vs étroit) ? Si bascule étroit après le rejet boot → plus de cold-acq. */
    if (addr == 0x08f9) {
        static unsigned fm = 0;
        if (fm < 40) {
            fprintf(stderr, "[c54x] FBMODE-WR #%u d_fb_mode <- 0x%04x PC=0x%04x insn=%u\n",
                    fm, val, s->pc, s->insn_count);
            fm++;
        }
    }
    /* === FBWATCH (env CALYPSO_FBWATCH, one-shot, hors master-gate) === */
    if (g_fbwatch_on < 0) g_fbwatch_on = calypso_gate("CALYPSO_FBWATCH", 0);
    if (g_fbwatch_on) {
        /* (1) qui ÉCRIT le flag dispatch FB (slot data[0x60-0x70] / 0x3dc0-2) ? */
        if ((addr >= 0x0060 && addr <= 0x0070) || (addr >= 0x3dc0 && addr <= 0x3dc2)) {
            static unsigned wf = 0;
            if (wf++ < 80)
                fprintf(stderr, "[c54x] FBWATCH-FLAG data[0x%04x] <- 0x%04x PC=0x%04x insn=%u%s\n",
                        addr, val, s->pc, s->insn_count, val ? "  *** NON-ZERO ***" : "");
        }
        /* (3) le DSP écrit-il une détection FB (d_fb_det 0x08f8 non-zéro) ? */
        if (addr == 0x08f8 && val) {
            static unsigned wd = 0;
            if (wd++ < 40)
                fprintf(stderr, "[c54x] FBWATCH-DET d_fb_det <- 0x%04x PC=0x%04x insn=%u\n",
                        val, s->pc, s->insn_count);
        }
        /* (5) LE FLAG PRODUCTEUR : qui écrit data[0x585f] (mot d'état foreground/ISR) ?
         * bit7 (0x0080) pollé par le foreground @0xf7af, bit8 (0x0100) par les ISR.
         * Si jamais écrit / bit7 jamais posé = le producteur du flag manque. */
        if (addr == 0x585f) {
            static unsigned w5 = 0;
            if (w5++ < 80)
                fprintf(stderr, "[c54x] FBWATCH-585F data[0x585f] <- 0x%04x PC=0x%04x insn=%u%s\n",
                        val, s->pc, s->insn_count, (val & 0x0080) ? "  *** BIT7 SET ***" : "");
        }
        /* la table de dispatch est-elle peuplée ? data[0x4c5b] = cible BACC A @0x7127 */
        if (addr == 0x4c5b) {
            static unsigned wt = 0;
            if (wt++ < 20)
                fprintf(stderr, "[c54x] FBWATCH-INITTAB-WR data[0x4c5b] <- 0x%04x PC=0x%04x insn=%u\n",
                        val, s->pc, s->insn_count);
        }
    }
    if (addr >= 0x08fa && addr <= 0x08fd) {
        static unsigned aw = 0;
        if (aw < 40) {
            int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
            const char *nm = addr==0x08fa?"TOA":addr==0x08fb?"PM":addr==0x08fc?"ANGLE":"SNR";
            fprintf(stderr, "[c54x] ANGLE-WR #%u %s[0x%04x]<-0x%04x PC=0x%04x A=%lld B=%lld T=%04x | "
                    "AR2=%04x[%04x] AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                    aw, nm, addr, val, s->pc, (long long)a, (long long)b, s->t,
                    s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                    s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]], s->insn_count);
            aw++;
        }
    }
    /* MTTCG lock : voir data_read ci-dessus. */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    data_write_locked(s, addr, val);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}

/* [2026-07-27] FLOWTRACE : voir en-tete du patch. */
FILE *g_flow_f = NULL;
long  g_flow_budget = -2;
int   g_flow_armed = 0;
static void flow_log(const char *rw, uint16_t addr, uint16_t val, uint16_t pc, unsigned insn)
{
    if (g_flow_budget == -2) {
        const char *e = getenv("CALYPSO_FLOWTRACE");
        g_flow_budget = (e && *e) ? atol(e) : -1;
        if (g_flow_budget > 0) {
            g_flow_f = fopen("/tmp/calypso_flow.txt", "w");
            fprintf(stderr, "[c54x] FLOWTRACE armed budget=%ld -> /tmp/calypso_flow.txt\n", g_flow_budget);
        }
    }
    if (g_flow_budget <= 0 || !g_flow_f || !g_flow_armed) return;
    if (addr < 0x2800 || addr >= 0x3000) return;
    fprintf(g_flow_f, "%s %04x %04x pc=%04x insn=%u\n", rw, addr, val, pc, insn);
    if (--g_flow_budget == 0) { fflush(g_flow_f); fclose(g_flow_f); g_flow_f = NULL;
        fprintf(stderr, "[c54x] FLOWTRACE done -> /tmp/calypso_flow.txt\n"); }
}


/* [2026-07-28] WMAP : carte agregee des ecrivains d une plage data[].
 * Voir en-tete du patch wmap.py — un agregat, pas un flux : aucun plafond de
 * lignes, donc aucune fenetre a rater (les 3 sondes precedentes ont toutes ete
 * tronquees). Sortie periodique : PC, n, valeurs distinctes, min/max. */
struct c54x_g_wmap_s g_wmap[WMAP_PCS];
int      g_wmap_n;
uint32_t g_wmap_tot;
int      g_wmap_on = -1;
uint16_t g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2;

static void wmap_dump(void)
{
    fprintf(stderr, "[c54x] WMAP plages 0x%04x..0x%04x + 0x%04x..0x%04x  ecritures=%u  ecrivains=%d%s\n",
            g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2, g_wmap_tot, g_wmap_n,
            g_wmap_n >= WMAP_PCS ? "  *** TABLE SATUREE, ecrivains manquants ***" : "");
    for (int i = 0; i < g_wmap_n; i++) {
        fprintf(stderr, "[c54x] WMAP   PC=0x%04x n=%-7u @0x%04x(n=%u) distinct=%s%d  min=0x%04x max=0x%04x  ex:",
                g_wmap[i].pc, g_wmap[i].n, g_wmap[i].addr0, g_wmap[i].n0,
                g_wmap[i].nv >= 8 ? ">=" : "", g_wmap[i].nv,
                g_wmap[i].mn, g_wmap[i].mx);
        for (int k = 0; k < g_wmap[i].nv && k < 8; k++)
            fprintf(stderr, " %04x", g_wmap[i].v[k]);
        fprintf(stderr, "%s\n", g_wmap[i].n0 < 2 ? "   <= (trop peu d echantillons)"
                : g_wmap[i].nv == 1 ? "   <= CONSTANTE dans le temps"
                                    : "   <= VARIE dans le temps = PORTE DE LA DONNEE");
    }
}

/* v3 : battement — prouve que la sonde est VIVANTE meme a zero ecriture.
 * Appele sur chaque write hors plage ; n imprime qu une fois par tranche de
 * 5e6 writes globaux, en rappelant le total DANS la plage (0 = vrai zero). */
static void wmap_heartbeat(void)
{
    static uint64_t k;
    if (!g_wmap_on) return;
    if (++k % 5000000ULL) return;
    fprintf(stderr, "[c54x] WMAP heartbeat: writes DSP=%llu, dans plages=%u"
            " (0 = la plage n est jamais ecrite par une instruction DSP)\n",
            (unsigned long long)k, g_wmap_tot);
}

static void wmap_note(uint16_t addr, uint16_t val, uint16_t pc)
{
    if (g_wmap_on < 0) {
        const char *e = getenv("CALYPSO_WMAP");
        g_wmap_on = (e && atoi(e) > 0) ? 1 : 0;
        const char *lo = getenv("CALYPSO_WMAP_LO"), *hi = getenv("CALYPSO_WMAP_HI");
        g_wmap_lo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x2c00;
        g_wmap_hi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x2c1f;
        const char *lo2 = getenv("CALYPSO_WMAP_LO2"), *hi2 = getenv("CALYPSO_WMAP_HI2");
        g_wmap_lo2 = lo2 ? (uint16_t)strtoul(lo2, NULL, 0) : 0xffff;
        g_wmap_hi2 = hi2 ? (uint16_t)strtoul(hi2, NULL, 0) : 0x0000;
        if (g_wmap_on)
            fprintf(stderr, "[c54x] WMAP armed 0x%04x..0x%04x\n", g_wmap_lo, g_wmap_hi);
    }
    if (!g_wmap_on) return;
    if (!((addr >= g_wmap_lo  && addr <= g_wmap_hi) ||
          (addr >= g_wmap_lo2 && addr <= g_wmap_hi2))) { wmap_heartbeat(); return; }

    int i;
    for (i = 0; i < g_wmap_n; i++) if (g_wmap[i].pc == pc) break;
    if (i == g_wmap_n) {
        if (g_wmap_n >= WMAP_PCS) return;
        g_wmap_n++;
        g_wmap[i].pc = pc; g_wmap[i].n = 0; g_wmap[i].nv = 0;
        g_wmap[i].mn = 0xffff; g_wmap[i].mx = 0;
        g_wmap[i].addr0 = addr; g_wmap[i].n0 = 0;   /* v2 : cellule temoin figee */
    }
    g_wmap[i].n++;
    if (val < g_wmap[i].mn) g_wmap[i].mn = val;
    if (val > g_wmap[i].mx) g_wmap[i].mx = val;
    /* v2 : le critere de SIGNAL est la variation dans le TEMPS a adresse FIXE. */
    if (addr != g_wmap[i].addr0) return;
    g_wmap[i].n0++;
    if (g_wmap[i].nv < 8) {
        int seen = 0;
        for (int k = 0; k < g_wmap[i].nv; k++) if (g_wmap[i].v[k] == val) { seen = 1; break; }
        if (!seen) g_wmap[i].v[g_wmap[i].nv++] = val;
    }
    /* v3 : seuil bas + tick temporel, pour que l ABSENCE soit mesurable. */
    if (++g_wmap_tot % WMAP_TICK == 0) wmap_dump();
}


/* [2026-07-28] DEMODIO : voir en-tete du patch demodio.py. */
int      g_dio_on = -1;
uint64_t g_dio_after;
unsigned g_dio_n;
uint16_t g_dio_pclo, g_dio_pchi;

static void dio_init(void)
{
    const char *e = getenv("CALYPSO_DEMODIO");
    g_dio_on = (e && atoi(e) > 0) ? 1 : 0;
    const char *a = getenv("CALYPSO_DEMODIO_AFTER");
    g_dio_after = a && *a ? strtoull(a, NULL, 0) : 40000000ULL;
    const char *lo = getenv("CALYPSO_DEMODIO_PCLO"), *hi = getenv("CALYPSO_DEMODIO_PCHI");
    g_dio_pclo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x9f95;
    g_dio_pchi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x9fe2;
    if (g_dio_on)
        fprintf(stderr, "[c54x] DEMODIO armed PC 0x%04x..0x%04x apres insn=%llu\n",
                g_dio_pclo, g_dio_pchi, (unsigned long long)g_dio_after);
}

static void dio_note(C54xState *s, const char *rw, uint16_t addr, uint16_t val)
{
    if (g_dio_on < 0) dio_init();
    if (!g_dio_on) return;
    if (s->pc < g_dio_pclo || s->pc > g_dio_pchi) return;
    if (s->insn_count < g_dio_after) return;
    if (g_dio_n >= 160) return;
    int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
    int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
    /* [2026-07-30] ASM ajoute a la ligne. Motif : la boucle 0x9fab-0x9fb8 fabrique
     * la reference du correlateur dans 0x2a00+ via
     *     0x9fb5  ld  *AR6-0%, TS, A     (A = table(+-1) << T,  T dans [16,31])
     *     0x9fb6  sfta A, +8
     *     0x9fb7  sfta A, -8             (les deux s'annulent)
     *     0x9fb8  sth *AR4+, A, ASM      (0x8694 = la variante ASM, cf. l.10528)
     * et elle n'ecrit que 0x0000 / 0xffff, soit {0, -1} au lieu de {+K, -K} —
     * une reference a |DC|/rms ~ 0,7, ce qui recoupe le 0,60 mesure sur
     * daram_2a00.cfile et explique qu'aucune correlation ne pique.
     * Or « mot haut de (A >> |ASM|) » vaut 0 pour toute magnitude < 2^16 mais
     * 0xffff pour toute valeur negative (extension de signe) : l'asymetrie vient
     * de la, et elle depend entierement de ASM et de T. T etait deja imprime,
     * ASM non — et le seul ASM que j'avais vu (-12) provenait d'une AUTRE sonde a
     * un AUTRE PC (SHADOW-DADST @0xa077), donc d'une autre fenetre : impossible de
     * conclure sans le mesurer ICI. On imprime donc ASM et ST1 bruts. */
    int _asm = s->st1 & 0x1F; if (_asm & 0x10) _asm |= ~0x1F;
    fprintf(stderr, "[c54x] DEMODIO %s PC=0x%04x op=0x%04x addr=0x%04x val=0x%04x "
            "A=%lld B=%lld T=0x%04x ASM=%d ST1=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
            rw, s->pc, prog_fetch(s, s->pc), addr, val, (long long)a, (long long)b,
            s->t, _asm, s->st1, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
    g_dio_n++;
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val)
{
    { static long _dwl_n = 0; if (getenv("CALYPSO_DWL_PROVE") && (_dwl_n++ % 100000) == 0)
        fprintf(stderr, "[c54x] DWL-PROVE appel #%ld addr=0x%04x pc=0x%04x\n", _dwl_n, addr, s->pc); }
    {   /* ─────────────────────────────────────────────────────────────────────
         * [2026-08-03] FBCNT-WATCH — CALYPSO_FBROUTE=1 (meme gate que FBROUTE,
         * dont c'est la suite directe). LECTURE SEULE, plafonnee.
         *
         * CE QU'ON CHERCHE. Depuis que le DMA livre les echantillons, la routine
         * FB s'execute enfin (FBROUTE : ENTER x2, high-water 0x794e — la zone
         * n'etait JAMAIS entree avant). Et la garde de `0x79e3` devient mesurable :
         *     jalon 0x7720 : DP=0x083 -> dma(0x7e) = data[0x41fe] = 1, 2, 3, 3, 3, 3
         *     la garde exige == 4.
         * Le compteur plafonne a 3. Il manque exactement une unite.
         *
         * PISTE A VERIFIER, PAS A SUPPOSER : le DMA livre le burst en 4 PAGES
         * (ALGTH=192 -> 96 mots, burst=296) et le compteur en compte 3. La
         * coincidence est trop belle pour etre citee sans preuve — ce watch dit
         * QUI ecrit la cellule et QUAND, ce qui la confirmera ou la tuera.
         *
         * ⚠️ LIMITE ASSUMEE : `0x41fe` est l'adresse resolue en `0x7720` (DP=0x083).
         * La garde est en `0x79e3`, qui n'est JAMAIS atteint — on ne peut donc pas
         * verifier qu'elle lit la meme cellule. L'inference est raisonnable (meme
         * adressage direct @0x7e, meme routine, DP constant sur 6 releves) mais
         * NON PROUVEE. Si le watch montre un ecrivain incoherent, c'est cette
         * inference qu'il faut suspecter en premier, pas le compteur. */
        static int _fc = -1;
        if (_fc < 0) _fc = calypso_gate("CALYPSO_FBROUTE", 0);
        if (_fc && addr == 0x41fe) {
            static unsigned _n = 0;
            if (_n < 60) {
                _n++;
                fprintf(stderr,
                        "[c54x] FBCNT-WR #%u data[0x41fe] 0x%04x -> 0x%04x "
                        "PC=0x%04x A=0x%06llx insn=%u\n",
                        _n, s->data[0x41fe], val, s->last_exec_pc,
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
                fflush(stderr);
            }
        }
    }
    {   /* [2026-08-22] WATCH-3FB4 — index d'état/slot de la recherche FB (0x3fb4).
         * Le coarse TOA (mode 0, branche A 0x791c) = (data[0x3fb4]-3)*T. On veut savoir
         * s'il PROGRESSE (0->1->2..) ou reste bloqué à 0. Writers DSP = 0x7744/0x7776.
         * LECTURE SEULE, plafonnée. Gate CALYPSO_WATCH_3FB4. */
        static int _w4 = -1;
        if (_w4 < 0) _w4 = calypso_gate("CALYPSO_WATCH_3FB4", 0);
        if (_w4 && addr == 0x3fb4) {
            static unsigned _n4 = 0;
            if (_n4 < 80) {
                _n4++;
                fprintf(stderr, "[c54x] WATCH-3FB4 data[0x3fb4] 0x%04x -> 0x%04x PC=0x%04x "
                        "T=0x%04x A=0x%06llx insn=%u\n", s->data[0x3fb4], val, s->pc,
                        s->t, (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
                fflush(stderr);
            }
        }
    }
    {   /* ─────────────────────────────────────────────────────────────────────
         * [2026-08-03] DTASKD-WATCH, patte 3/3 — CALYPSO_DTASKD_WATCH=1, defaut 0.
         * LECTURE SEULE, plafonnee. Voir les pattes 1 et 2 dans calypso_trx.c.
         *
         * CE QU'ON CHERCHE. La console du firmware (osmocon.log) montre, a chaque
         * cycle : `l1s_nb_cmd()` commande une reception, puis `l1s_nb_resp()`
         * imprime `EMPTY` (prim_rx_nb.c:74), dont le test est
         *     if (dsp_api.db_r->d_task_d == 0) { puts("EMPTY\n"); return 0; }
         *
         * PIEGE A NE PAS REPRODUIRE. `db_w->d_task_d` et `db_r->d_task_d` ne sont
         * PAS la meme cellule — ce sont deux structures distinctes du firmware :
         *     db_w = T_DB_MCU_TO_DSP @ 0xFFD00000 (p0) / 0xFFD00028 (p1)
         *     db_r = T_DB_DSP_TO_MCU @ 0xFFD00050 (p0) / 0xFFD00078 (p1)
         * (osmocom-bb, include/calypso/dsp_api.h:20-23 ; d_task_d est a l'offset 0
         * des DEUX structures.) Donc `EMPTY` ne veut PAS dire « l'ecriture de
         * l'ARM s'est perdue » : il veut dire « le DSP n'a jamais recopie la tache
         * dans sa page REPONSE ». C'est cette patte-ci qui tranche.
         *
         * Mots DSP correspondants (api = data[0x0800 + offset_ARM/2]) :
         *     W p0 = data[0x0800]   W p1 = data[0x0814]   <- ecrit par l'ARM
         *     R p0 = data[0x0828]   R p1 = data[0x083C]   <- doit etre ecrit ICI
         *
         * INDICE DEJA AU JOURNAL : la sonde DISPATCH-PROBE montre AR2 alternant
         * entre 0x0828 et 0x083c dans les lignes EMPILEMENT — soit exactement les
         * deux bases de page R. Le DSP a donc le bon pointeur en main ; reste a
         * voir s'il ECRIT la cellule.
         *
         * L'ABSENCE DOIT ETRE LISIBLE (lecon §13.6) : le resume periodique imprime
         * les compteurs meme a zero, pour qu'aucune ligne ne soit ambigue entre
         * « le DSP n'ecrit pas » et « la sonde n'est pas armee ». */
        static int _dw = -1;
        if (_dw < 0) {
            _dw = calypso_gate("CALYPSO_DTASKD_WATCH", 0);
            if (_dw) {
                fprintf(stderr, "[dtaskd] patte 3/3 armee (ecritures DSP) : "
                        "R p0=data[0x0828] R p1=data[0x083C]\n");
                fflush(stderr);
            }
        }
        if (_dw && (addr == 0x0828 || addr == 0x083C)) {
            static unsigned long long _n0 = 0, _n1 = 0, _nz = 0;
            unsigned long long _n = (addr == 0x0828) ? ++_n0 : ++_n1;
            if (val) _nz++;
            if (_n <= 40 || (_n % 5000) == 0) {
                fprintf(stderr,
                        "[dtaskd] DSP>WR  R p%d  data[0x%04x] <- 0x%04x  "
                        "(non_nuls=%llu  p0=%llu p1=%llu)  PC=0x%04x insn=%u\n",
                        (addr == 0x0828) ? 0 : 1, addr, val,
                        _nz, _n0, _n1, s->last_exec_pc, s->insn_count);
                fflush(stderr);
            }
        }
    }
    dio_note(s, "W", addr, val);
    wmap_note(addr, val, s->pc);
    flow_log("W", addr, val, s->pc, s->insn_count);
    {   /* [2026-07-28] WZWRITE : qui remplit l entree du noyau ? (voir en-tete) */
        static int _wz = -1; static unsigned _wzn = 0;
        if (_wz < 0) _wz = calypso_gate("CALYPSO_WZWRITE", 0);
        if (_wz && addr == 0x2c00 && (s->pc == 0x9fd5 || s->pc == 0x9ab1) && _wzn < 40) {
            /* v3 : filtre par PC PRODUCTEUR — 0xa03d/a042/a079 (init MAC) exclus,
             * ils saturaient le plafond et masquaient les bursts suivants. */
            _wzn++;
            fprintf(stderr, "[c54x] WZWRITE data[0x%04x] <- 0x%04x PC=0x%04x op=0x%04x "
                    "A=0x%010llx AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    addr, val, s->pc, prog_fetch(s, s->pc),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        }
    }
    {   /* [2026-07-29] DMAQ-RESET : le pointeur d'ecriture de la file DMA
         * revient-il a sa base sans avoir ete consomme ? C'est l'hypothese
         * restante : le producteur empile, un reset efface, le consommateur
         * trouve vide. Gate CALYPSO_DMAQ, plafond 30. */
        static int _dqr = -1; static unsigned _dqrn = 0;
        if (_dqr < 0) _dqr = calypso_gate("CALYPSO_DMAQ", 0);
        if (_dqr && _dqrn < 30 && (addr == 0x433f || addr == 0x433e)
            && val == 0x4330 && s->data[addr] != 0x4330) {
            _dqrn++;
            fprintf(stderr, "[c54x] DMAQ-RESET data[0x%04x] 0x%04x -> 0x4330 "
                    "PC=0x%04x (la file contenait %u element(s)) insn=%u\n",
                    addr, s->data[addr], s->pc,
                    (unsigned)((s->data[0x433f] - s->data[0x433e]) & 0xFFFF),
                    s->insn_count);
        }
    }
    {   /* [2026-07-28] ERRWATCH : qui pose d_error_status ? (voir en-tete) */
        static int _ew = -1; static unsigned _ewn = 0;
        if (_ew < 0) _ew = calypso_gate("CALYPSO_ERRWATCH", 0);
        /* v3 : sur d_error_status (0x08D5) on ignore les ecritures de 0 —
         * elles sont le nettoyage de boot et consommaient tout le plafond. */
        if (_ew && addr == 0x08D5 && val != 0 && _ewn < 60) {
            _ewn++;
            const char *_b = (val & 0x0800) ? "STACK_OV" : (val & 0x0400) ? "DMA_UL_PEND" :
                             (val & 0x0200) ? "DMA_UL_PROG" : (val & 0x0100) ? "DMA_UL_TASK" :
                             (val & 0x0080) ? "VM" : (val & 0x0020) ? "DMA_PEND" :
                             (val & 0x0010) ? "DMA_TASK" : (val & 0x0008) ? "DMA_PROG" :
                             (val & 0x0004) ? "IQ_SAMPLES" : (val & 0x0001) ? "RHEA" : "(clear)";
            fprintf(stderr, "[c54x] ERRWATCH data[0x%04x] 0x%04x -> 0x%04x [%s] "
                    "PC=0x%04x op=0x%04x A=0x%06llx AR1=%04x AR2=%04x AR6=%04x insn=%u\n",
                    addr, s->data[addr], val, _b, s->pc, prog_fetch(s, s->pc),
                    (unsigned long long)(s->a & 0xFFFFFFULL),
                    s->ar[1], s->ar[2], s->ar[6], s->insn_count);
        }
    }
    {   /* [2026-07-28] DMAWATCH (ecriture). */
        static int _dw3 = -1; static unsigned _dww = 0;
        if (_dw3 < 0) _dw3 = calypso_gate("CALYPSO_DMAWATCH", 0);
        if (_dw3 && addr >= 0x0054 && addr <= 0x0057 && _dww < 40) {
            _dww++;
            fprintf(stderr, "[c54x] DMAWATCH WR 0x%04x <- 0x%04x (etait 0x%04x) PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    {   /* [2026-07-28] BOOTCMD cote DSP : qui ecrase la commande ? */
        static int _bc2 = -1; static unsigned _bc2n = 0;
        if (_bc2 < 0) _bc2 = calypso_gate("CALYPSO_BOOTCMD", 0);
        if (_bc2 && addr >= 0x0FFC && addr <= 0x0FFF && _bc2n < 40) {
            _bc2n++;
            fprintf(stderr, "[c54x] BOOTCMD DSP data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x insn=%u%s\n",
                    addr, s->data[addr], val, s->pc, s->insn_count,
                    (addr == 0x0FFF) ? "   <<<< CELLULE DE COMMANDE" : "");
        }
    }
    {   /* [2026-07-27] DISPTAB-WR : qui remplit la table de dispatch ? */
        static int _dt = -1; static unsigned _dtn = 0;
        if (_dt < 0) _dt = calypso_gate("CALYPSO_DISPTAB", 0);
        if (_dt && addr >= 0x4380 && addr <= 0x43cf && _dtn < 60) {
            _dtn++;
            fprintf(stderr, "[c54x] DISPTAB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
        }
    }
    /* [2026-07-27] DEMOD-NOCLOBBER (gated CALYPSO_DEMOD_NOCLOBBER) : l etage
     * demod emule (PC 0x9fb8=I / 0x9fe2=Q) remplit le buffer d entree du
     * correlateur avec des paires CONSTANTES (0000,52ed) — prouve par la trace
     * de flux — ecrasant la vraie FCCH deposee par feed_iq (FB_IQ_DARAM=1).
     * On ignore ces ecritures : feed_iq devient autoritaire sur 0x2a00. */
    {
        /* @BEQUILLE — DEMOD_NOCLOBBER  (CALYPSO_DEMOD_NOCLOBBER, atoi>0, defaut OFF)
         *   masque  : l'etage demod emule ecrit des paires constantes dans le buffer
         *             d'entree correlateur [0x2a00,0x2b28) et ecrase la FCCH reelle
         *             deposee par feed_iq ; la vraie branche = un demod qui consomme
         *             l'I/Q RX au lieu de produire des constantes. Ici on SUPPRIME
         *             l'ecriture (return) au lieu de corriger le producteur.
         *   retirer : des que l'etage demod 0x9f95-0x9fe2 lit une source I/Q reelle, ou
         *             des que FB_IQ_OWNS=1 rend feed_iq seul proprietaire de 0x2a00.
         */
        static int _nc = -1;
        if (_nc < 0) { const char *e = getenv("CALYPSO_DEMOD_NOCLOBBER"); _nc = (e && atoi(e) > 0) ? 1 : 0; }
        if (_nc && addr >= 0x2a00 && addr < 0x2b28 &&
            (s->pc == 0x9fb8 || s->pc == 0x9fe2)) {
            static unsigned _ncn = 0;
            if (_ncn++ < 8)
                fprintf(stderr, "[c54x] DEMOD-NOCLOBBER skip PC=0x%04x data[0x%04x] <- 0x%04x "
                        "(ecriture demod ignoree ; l alimentation vient de rx_burst sauf si FB_IQ_OWNS=1)\n", s->pc, addr, val);
            return;
        }
    }
    /* MEM-WATCH-2B80 (2026-07-02, gated CALYPSO_MEM_WATCH_2B80) : voir le
     * commentaire jumeau dans data_read_locked. Log tout WRITE dans
     * [0x2b80,0x2c00), cap 200 -- confirme/infirme si un boot-copy peuple
     * jamais cette region avant que le correlateur la lise. */
    /* [2026-07-27] B4 (gated CALYPSO_B4) : watchpoint d_fb_det (0x08f8) -> distingue
     * "le DSP ecrit 0" (correlateur conclut negatif) de "jamais ecrit" (chemin non
     * atteint). Deux bugs differents. */
    if (addr == 0x08f8) {
        static int _b4 = -1; static unsigned _b4n = 0;
        if (_b4 < 0) _b4 = calypso_gate("CALYPSO_B4", 0);
        if (_b4 && _b4n < 64) {
            _b4n++;
            fprintf(stderr, "[c54x] B4-DFBDET-WR data[0x08f8] 0x%04x -> 0x%04x PC=0x%04x xpc=%u insn=%u\n",
                    s->data[0x08f8], val, s->pc, s->xpc, s->insn_count);
        }
    }
    /* [2026-07-27] B1 boot-copy watch (gated CALYPSO_B1) : ecritures dans la
     * table de reference [0x2c00,0x2c10) avec PC source -> confirme (ou non) la
     * boot-copy 0x76f8->0x2c00 et QUI l ecrit. */
    if (addr >= 0x2c00 && addr < 0x2c10) {
        static int _b1w = -1; static unsigned _b1wn = 0;
        if (_b1w < 0) _b1w = calypso_gate("CALYPSO_B1", 0);
        if (_b1w && _b1wn < 64 && val != 0) {
            _b1wn++;
            fprintf(stderr, "[c54x] B1-BOOTCOPY-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x xpc=%u insn=%u\n",
                    addr, s->data[addr], val, s->pc, s->xpc, s->insn_count);
        }
    }
    if (addr >= 0x2b80 && addr < 0x2c00) {
        static int mw2b80_wen = -1;
        if (mw2b80_wen < 0) mw2b80_wen = calypso_gate("CALYPSO_MEM_WATCH_2B80", 0);
        if (mw2b80_wen) {
            static unsigned mw2b80_wn = 0;
            if (mw2b80_wn < 200) {
                mw2b80_wn++;
                fprintf(stderr, "[c54x] MEM-WATCH-2B80-WR data[0x%04x] 0x%04x -> 0x%04x "
                        "PC=0x%04x insn=%u\n", addr, s->data[addr], val, s->pc,
                        s->insn_count);
            }
        }
    }
    /* SONDE GAP-1 : transitions du pointeur de handler data[0x3f5e] (machine à
     * états L1) + cellules de contrôle API 0x0908/0x0909 au même instant. Montre
     * si le scheduler avance l'état ou reste figé sur 0x7013 (FB), et d'où (PC). */
    if (addr == 0x3f5e) {
        static unsigned stwr = 0;
        if (stwr++ < 80)
            fprintf(stderr, "[c54x] STATE-WR data[0x3f5e] 0x%04x -> 0x%04x PC=0x%04x "
                    "api[0x908]=0x%04x api[0x909]=0x%04x api[0x945]=0x%04x insn=%u\n",
                    s->data[0x3f5e], val, s->pc, s->data[0x0908], s->data[0x0909],
                    s->data[0x0945], s->insn_count);
    }

    /* SONDE GAP-1 : qui ECRIT data[0x0c36] (le pointeur de TACHE CALA'd a 0xb3a5,
     * nul -> derail) ? Catch des writes DSP (y compris indirect via AR). */
    if (addr == 0x0c36) {
        static unsigned tw = 0;
        if (tw++ < 60)
            fprintf(stderr, "[c54x] TASKPTR-WR data[0x0c36] 0x%04x -> 0x%04x PC=0x%04x "
                    "XPC=%u AR[0..7]=%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x insn=%u\n",
                    s->data[0x0c36], val, s->pc, s->xpc,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->insn_count);
    }

    /* [2026-07-23] STATE435B-WR : qui ecrit data[0x435b] (mot d etat de la SM
     * go-live 0xa4e4 : bits 0x10/0x40/0x100 gatent la progression + l enable INTM).
     * =0 -> SM bloquee. Trouver qui doit le peupler (ARM ? task done ?). Cap 40. */
    if (addr == 0x435b) {
        static unsigned sw=0;
        if (sw++ < 40)
            fprintf(stderr, "[c54x] STATE435B-WR data[0x435b] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx insn=%u\n",
                    s->data[0x435b], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* [2026-07-23] WATCH-09BC-WR : data[0x09bc] deja annote "flag ARM" (comment
     * SM-TRACE ligne ~12352). Full-ROM scan (2026-07-23) montre que le gate go-live
     * 0xa544 (BITF *(0x09bc),1 ; BC 0xa549 si NTC) SAUTE le seul BACC natif
     * (0xa546-0xa548, via d[0x3fe0]=0x70ce) vers le bootstrap operationnel 0x7102
     * CALL 0xd247 SAUF SI bit0 de data[0x09bc] est deja pose. AUCUN site trouve
     * dans PROM0-3 ne fait un ORM/SET clair de ce bit (seulement 2 clears ANDM a
     * 0x70fc/0x712d + un site ambigu 0xcf53 au decode incertain). Ce watch confirme
     * en RUNTIME : (a) qui ecrit 0x09bc et avec quelle valeur, (b) si bit0 est
     * JAMAIS mis a 1 nativement. Cap 40, READ-ONLY. */
    if (addr == 0x09bc) {
        static unsigned w9 = 0;
        if (w9++ < 40)
            fprintf(stderr, "[c54x] WATCH-09BC-WR data[0x09bc] 0x%04x -> 0x%04x (bit0 %d->%d) "
                    "PC=0x%04x A=0x%06llx insn=%u\n",
                    s->data[0x09bc], val, s->data[0x09bc] & 1, val & 1, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* [2026-07-23] WATCH-000B-WR : data[0x000b] teste par BITF au moins 2x dans
     * le dispatcher background (0xdeb6: BITF *(0x000b),0x4000 ; 0xdec2: BITF
     * *(0x000b),0x2000 -- bits 14 et 13). User hypothese : "l'histoire des 11"
     * (11x tpu_enq_at(0) dans l1s_rx_win_ctrl pour FB, jamais modelise en timing
     * -- cf calypso_tpu.c) -- est-ce que 0x000b (=11 decimal, coincidence
     * d'ADRESSE pas de compteur a priori) est le mot que le sequenceur TPU est
     * cense faire progresser via ces 11 delais, et qui reste bloque a 0 faute de
     * timing modelise ? Ce watch confirme en RUNTIME si data[0x000b] est ECRIT
     * par QUOI QUE CE SOIT nativement (natif = pas de hack). Si jamais ecrit ->
     * BITF y lit toujours 0 -> TC toujours faux -> boucle jamais debloquee par
     * cette voie -> confirme/infirme l'hypothese. Cap 40, READ-ONLY. */
    if (addr == 0x000b) {
        static unsigned wb = 0;
        if (wb++ < 40)
            fprintf(stderr, "[c54x] WATCH-000B-WR data[0x000b] 0x%04x -> 0x%04x "
                    "PC=0x%04x A=0x%06llx insn=%u\n",
                    s->data[0x000b], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* [2026-07-23] WATCH-0810-WR : data[0x0810] = db_w->d_ctrl_system (write-page
     * MCU->DSP, offset 16), champ nomme confirme via osmocom-bb (dsp_api.h:75
     * "Control Register for RESET/RESUME"). Bit15 = B_TASK_ABORT
     * (l1_environment.h:365), ARM le pose dans l1s_reset() ("abort RF tasks, dsp
     * will reset current+pending tasks"). Le DSP teste CE bit a 0xa53c/0xa53f
     * (BITF *(AR1+0x10),0x8000, AR1=0x0800) : bit15 SET -> tombe vers
     * a541-a544-a546-0x09bc-0xd247 (chemin operationnel/bootstrap) ; bit15 CLEAR
     * -> saute a 0xa575 (court-circuit, jamais bootstrap). Confirme si/quand
     * l'ARM ecrit ce bit, et sa valeur au moment ou le DSP le lit. Cap 40. */
    if (addr == 0x0810) {
        static unsigned w810 = 0;
        if (w810++ < 40)
            fprintf(stderr, "[c54x] WATCH-0810-WR data[0x0810] 0x%04x -> 0x%04x "
                    "(bit15/B_TASK_ABORT %d->%d) PC=0x%04x insn=%u\n",
                    s->data[0x0810], val, !!(s->data[0x0810] & 0x8000), !!(val & 0x8000),
                    s->pc, s->insn_count);
    }
    /* [2026-07-23] DISPATCH-CELL-RESEED (READ-ONLY) : d[0x43d8]/d[0x3fd4]/d[0x4368]
     * confirmes CONSTANTS (0xab38/0xc1fa/0xaff9) sur tout runtime observe cette
     * session -- watch WRITE pour prouver/refuter qu'ils sont jamais reseedes vers
     * autre chose (ferme definitivement le gap "static constant" du workflow xref-scan). */
    if (addr == 0x43d8 || addr == 0x3fd4 || addr == 0x4368) {
        static unsigned _ndcr = 0;
        if (_ndcr++ < 30)
            fprintf(stderr, "[c54x] DISPATCH-CELL-RESEED data[0x%04x] 0x%04x -> 0x%04x "
                    "PC=0x%04x insn=%u\n", addr, s->data[addr], val, s->pc, s->insn_count);
    }
    /* [2026-07-23] FBDET-WR (READ-ONLY, INCONDITIONNEL) : data[0x08F8] = d_fb_det, le
     * champ NDB que le firmware ARM lit pour savoir si le corrélateur a trouve une FCCH.
     * JAMAIS vu ecrit nativement cette session. Watch total (pas de cap) -- champ critique. */
    if (addr == 0x08F8) {
        fprintf(stderr, "[c54x] FBDET-WR data[0x08F8] 0x%04x -> 0x%04x PC=0x%04x insn=%u\n",
                s->data[0x08F8], val, s->pc, s->insn_count);
    }
    /* [2026-07-23] READY-WR : qui peuple la readiness du go-live (0xaad5 lit
     * *data[0x434e]=*0x4340 et *data[0x434f]=*0x7f75 ; A=0 -> 0xa4cd skip enable).
     * Watch les pointeurs (0x434e/0x434f) ET les cibles (0x4340/0x7f75). Cap 40. */
    if (addr == 0x434e || addr == 0x434f || addr == 0x4340 || addr == 0x7f75) {
        static unsigned rw=0;
        if (rw++ < 40)
            fprintf(stderr, "[c54x] READY-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx insn=%u\n",
                    addr, s->data[addr], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* [2026-07-23] 3FCD-WR : qui ecrit data[0x3fcd] (la CIBLE du RET@0x0157 :
     * le handler frame 0x013b fait PSHD *(0x3fcd) puis RET -> saute a data[0x3fcd].
     * =0 en QEMU -> derail. Trouver qui/quoi doit le peupler. Cap 40. */
    if (addr == 0x0155 || addr == 0x013b || addr == 0x0154) {
        static unsigned ov=0;
        if (ov++ < 30)
            fprintf(stderr, "[c54x] OVLD-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx insn=%u\n",
                    addr, s->data[addr], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    if (addr == 0x3fcd || addr == 0x3fce || addr == 0x3fcf) {
        static unsigned f3=0;
        if (f3++ < 40)
            fprintf(stderr, "[c54x] 3FCD-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx XPC=%u insn=%u\n",
                    addr, s->data[addr], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->xpc, s->insn_count);
    }
    /* SONDE GAP-1 : qui ECRIT data[0x43c0] (le pointeur de dispatch lu par le
     * tremplin 0xb40e/0xb40f bacc), qui vaut 0xf074 (adresse de table = derail) ? */
    if (addr == 0x43c0) {
        static unsigned dw = 0;
        if (dw++ < 60)
            fprintf(stderr, "[c54x] DISPPTR-WR data[0x43c0] 0x%04x -> 0x%04x PC=0x%04x "
                    "A=0x%010llx XPC=%u insn=%u\n",
                    s->data[0x43c0], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->xpc, s->insn_count);
    }

    /* SONDE Phase B DISPVAL-WR : qui ECRIT la valeur 0xf074 (= base LUT, la
     * cible de bacc fautive) dans la table de dispatch 0x4300-0x43ff ? Nomme
     * le planteur du pointeur de handler corrompu. Cap 60. */
    if (val == 0xf074 && addr >= 0x4300 && addr < 0x4400) {
        static unsigned dvw = 0;
        if (dvw++ < 60)
            fprintf(stderr, "[c54x] DISPVAL-WR data[0x%04x] <- 0xf074 PC=0x%04x "
                    "A=0x%010llx XPC=%u insn=%u\n",
                    addr, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->xpc, s->insn_count);
    }

    /* SLOT4387-WR (2026-06-24) : qqn reecrit-il le slot dispatch terminal
     * data[0x4387] avec un vrai handler (!= 0xab38 idle) ? (test H1). Cap 40. */
    if (addr == 0x4387) {
        static unsigned s4=0;
        if (s4++<12) {
            /* [2026-07-22] contexte COMPLET : d ou vient la valeur 0x%04x ? (index
             * de dispatch). Dump AR/A/B/DP/ST0 + les 6 mots prog autour du store
             * pour reconstruire le calcul d index de la table 0xab10. */
            fprintf(stderr, "[c54x] SLOT4387-WR data[0x4387] <- 0x%04x PC=0x%04x insn=%u\n"
                    "         A=0x%06llx B=0x%06llx T=0x%04x DP=0x%03x ST0=0x%04x\n"
                    "         AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x\n"
                    "         prog[pc-3..pc+2]=%04x %04x %04x %04x %04x %04x\n",
                    val, s->pc, s->insn_count,
                    (unsigned long long)(s->a & 0xFFFFFFULL), (unsigned long long)(s->b & 0xFFFFFFULL),
                    s->t, (s->st0 & ST0_DP_MASK), s->st0,
                    s->ar[0],s->ar[1],s->ar[2],s->ar[3],s->ar[4],s->ar[5],s->ar[6],s->ar[7],
                    prog_fetch(s,(uint16_t)(s->pc-3)), prog_fetch(s,(uint16_t)(s->pc-2)),
                    prog_fetch(s,(uint16_t)(s->pc-1)), prog_fetch(s,s->pc),
                    prog_fetch(s,(uint16_t)(s->pc+1)), prog_fetch(s,(uint16_t)(s->pc+2)));
        }
    }
    /* SONDE Phase B SEED-WR : qui ecrit la base de pile data[0x5ac8..0x5acc] =
     * l'adresse de retour que le RET du handler no-op (0xab38) depile ? Si
     * jamais ecrit (que le memset 0x88 en dessous) -> seed retour-scheduler
     * jamais pose -> idle-path correct mais init manquante (#2). Cap 40. */
    if (addr >= 0x5ac8 && addr <= 0x5acc) {
        static unsigned sw = 0;
        if (sw++ < 40)
            fprintf(stderr, "[c54x] SEED-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
    }
    /* [2026-07-22] VECWATCH (gated CALYPSO_AR0_DEBUG) : attrape la VALEUR 0x71f4
     * (vecteur go-live trampoline) ecrite N'IMPORTE OU, depuis insn 0. Tranche :
     * si 0x71f4 est ecrit a une addr != 0x5ac8 -> write egare (bug SP/adressage) ;
     * si jamais ecrit -> init au reset non modelisee. Meme fonction que SEED-WR
     * (prouve firing). Cap 40. */
    if (val == 0x71f4) {
        static int vw_en = -1;
        if (vw_en < 0) vw_en = calypso_gate("CALYPSO_AR0_DEBUG", 0);
        if (vw_en) {
            static unsigned vw = 0;
            if (vw++ < 40)
                fprintf(stderr, "[c54x] VECWATCH 0x71f4 -> data[0x%04x] PC=0x%04x "
                        "SP=0x%04x AR0=0x%04x AR1=0x%04x insn=%u\n",
                        addr, s->pc, s->sp, s->ar[0], s->ar[1], s->insn_count);
        }
    }

    /* === NDB-CTL-WR : trace ARM-side writes to NDB control flags in
     * [data[0x08F8]..data[0x0900]] = d_fb_det, d_fb_mode, a_sync_demod[],
     * d_sb_ext, etc. The firmware writes mode flags before scheduling
     * SB task — finding which flag toggles SB vs FB tells the DSP
     * dispatcher selector. Capped 50. */
    {
        bool ndb_ctl = (addr >= 0x08F8 && addr <= 0x0900);
        /* Filter on s->pc to identify ARM-side writes : ARM has no PC
         * concept here (it writes via MMIO callback), so s->pc reflects
         * the DSP PC at the moment. ARM-side calls land via calypso_dsp_write
         * which does direct s->data[] write, NOT data_write_locked → so
         * this probe sees only DSP-side writes to NDB. Both paths are
         * useful to discriminate. */
        if (ndb_ctl) {
            static unsigned ndb_log = 0;
            if (ndb_log++ < 50) {
                if (calypso_debug_enabled("NDB-CTL-WR")) fprintf(stderr,
                        "[c54x] NDB-CTL-WR data[0x%04x] <- 0x%04x "
                        "(was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count);
            }
        }
    }

    /* === SYNC-DEMOD-WR probe (2026-05-28) ===
     * Trace writes to a_sync_demod cells [0x08FA..0x08FD] (= TOA/PM/ANGLE/SNR
     * per NDB layout, indices D_TOA=0 D_PM=1 D_ANGLE=2 D_SNR=3).
     * Filter OUT les PCs stale-AR connus (0x821a 0x8213 0x8217) qui sont
     * juste du AR4-walk garbage. Le but est de capturer le VRAI publisher
     * (= la routine FB-det code DSP qui calcule les vraies valeurs). Si
     * apres tout le run on ne voit que les 3 PCs garbage → le vrai code
     * n'est jamais atteint. Cap 200 entries. */
    if ((addr >= 0x08FA && addr <= 0x08FD) ||
        (addr >= 0x0830 && addr <= 0x0833) ||
        (addr >= 0x0844 && addr <= 0x0847)) {
        {   /* [2026-08-22] FILTRE PC RETIRE : on VEUT voir les writers 0x821a/0x8213/
             * 0x8217 (supposes « garbage ») — ce sont peut-etre les VRAIS writers du TOA
             * que lit le firmware. + a_serv_demod (0x830/0x844, lu par FB0/FB1_SEARCH). */
            static unsigned sd_log;
            const unsigned LIMIT = 200;
            if (sd_log < LIMIT) {
                const char *name = (addr==0x08FA||addr==0x0830||addr==0x0844) ? "TOA"
                                 : (addr==0x08FB||addr==0x0831||addr==0x0845) ? "PM"
                                 : (addr==0x08FC||addr==0x0832||addr==0x0846) ? "ANGLE"
                                 : "SNR";
                fprintf(stderr,
                        "[c54x] SYNC-DEMOD-WR #%u %s[0x%04x] <- 0x%04x "
                        "(was 0x%04x) PC=0x%04x op=0x%04x "
                        "A=0x%010llx B=0x%010llx "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        sd_log, name, addr, val, s->data[addr],
                        s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                        s->insn_count);
                sd_log++;
                if (sd_log == LIMIT)
                    fprintf(stderr,
                            "[c54x] SYNC-DEMOD-WR log capped at %u\n", LIMIT);
            }
        }
    }

    /* === FB-DET-WR probe (2026-05-28) ===
     * Trace specifically writes to d_fb_det (DSP word 0x08F8) by PC.
     * Run post-option-A shows d_fb_det stuck at 0x1255 (96×), no varied
     * lock signature. Question : ONE site écrit toujours 0x1255, OU plusieurs
     * sites écrivent (mais ARM ne consume que celui qui produit 0x1255) ?
     * Snapshot par-événement : PC + B accumulator (= source du STH/STL),
     * prev_op (= instruction précédente, contexte d'addressing), trail des
     * 4 PCs précédents pour identifier la routine. Cap 300. */
    if (addr == 0x08F8) {
        static unsigned fbdet_log;
        const unsigned LIMIT = 300;
        if (fbdet_log < LIMIT) {
            fprintf(stderr,
                    "[c54x] FB-DET-WR #%u data[0x08F8] <- 0x%04x "
                    "PC=0x%04x op=0x%04x prev=0x%04x "
                    "B=0x%010llx A=0x%010llx SP=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    fbdet_log, val,
                    s->pc, s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc - 1)],
                    (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->sp,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    s->insn_count);
            fbdet_log++;
            if (fbdet_log == LIMIT)
                fprintf(stderr, "[c54x] FB-DET-WR log capped at %u\n", LIMIT);
        }
    }

    /* === A_SCH-WR probe : trace DSP writes to a_sch[0..4] in both db_r
     * pages. If DSP never writes these cells, firmware reads stale RAM
     * → random BSIC in FBSB_CONF. If DSP writes garbage, the SCH demod
     * path is broken upstream. Helps discriminate the SB sync root cause.
     * Capped at 50 logged hits to avoid spam. */
    {
        bool a_sch_p0 = (addr >= 0x0837 && addr <= 0x083B);  /* page 0 a_sch[0..4] */
        bool a_sch_p1 = (addr >= 0x084B && addr <= 0x084F);  /* page 1 a_sch[0..4] */
        if (a_sch_p0 || a_sch_p1) {
            static unsigned a_sch_log = 0;
            if (a_sch_log++ < 50) {
                if (calypso_debug_enabled("A_SCH-WR")) fprintf(stderr,
                        "[c54x] A_SCH-WR data[0x%04x] <- 0x%04x page=%d "
                        "idx=%d PC=0x%04x insn=%u\n",
                        addr, val,
                        a_sch_p0 ? 0 : 1,
                        (int)(addr - (a_sch_p0 ? 0x0837 : 0x084B)),
                        s->pc, s->insn_count);
            }
        }
    }

    /* === BLOB-WR diagnostic for dsp_blobs/ test harness ===
     * Logs writes either targeting scratch [0x2000..0x200F] (dsp-deadbeef
     * etc.) or carrying a known blob signature value. Self-throttled to
     * 5000 hits per process. Zero impact when no blob test is running. */
    {
        static unsigned blob_wr_count = 0;
        bool is_scratch = (addr >= 0x2000 && addr <= 0x200F);
        bool is_magic = (val == 0xCAFE || val == 0xBEEF || val == 0xDEAD ||
                         val == 0x2B2B || val == 0x4906 || val == 0x1B00 ||
                         val == 0x7080 || val == 0x4000);
        if ((is_scratch || is_magic) && blob_wr_count < 5000) {
            blob_wr_count++;
            fprintf(stderr,
                    "[c54x] BLOB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
        }
    }

    /* [2026-07-22] WATCH-VEC : writes vers table de vecteurs IT (0x0080-0x00FF)
     * + dispatch 0x013b (0x0138-0x013c). Tranche (a) jamais ecrit / (b) ecrit-
     * droppe / (c) corrompu. Env dediee CALYPSO_WATCH_VEC. */
    {
        static int wv = -1;
        if (wv < 0) { const char *e = getenv("CALYPSO_WATCH_VEC"); wv = (e && *e != 0) ? 1 : 0; }
        if (wv && ((addr >= 0x0080 && addr <= 0x00FF) || (addr >= 0x0138 && addr <= 0x013C))) {
            static unsigned wvn = 0;
            if (wvn++ < 100)
                fprintf(stderr, "[c54x] WATCH-VEC data[0x%04x] <- 0x%04x (was 0x%04x) "
                        "PC=0x%04x op=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, prog_fetch(s, s->pc), s->insn_count);
        }
    }

    /* === SP-CATASTROPHE fix : DROM[0x9187] silicon-correct read-only ===
     * Per SPRU172C : when PMST.DROM=1, the DSP ROM in data space is
     * read-only. Our emulator previously allowed writes which corrupted
     * data[0x9187] (0xFF86 → 0xF6B7) via a walking `STH B,*AR2+` inside
     * the RPTB body [0x815E..0x8176]. Dispatcher at 0x8341..0x8353 then
     * read the corrupted value and computed CALAD-A target = 0x70C3
     * (= the CALA-A opcode itself in PROM0) instead of the legit MAC
     * routine 0x8261, falling into a self-call loop. Each iteration
     * pushed one word, SP eventually reached MMR_SP (0x0018), the push
     * aliased SP itself → SP-CATASTROPHE Δ=+28843.
     *
     * Surgical : only data[0x9187] needs protection (the dispatcher LUT
     * slot). Wider ranges break other firmware paths (calibrated against
     * historical SARAM-overlay writability). Drop silently — matches
     * silicon. Probe first 20 attempts for diag. */
    /* 2026-05-29 v2 : protège la COLONNE LUT du dispatcher dans la DROM, PAS
     * toute la DROM (le full-DROM v1 bloquait le scratch firmware 0x9380/
     * 0x93c2/0xd4xx → readback stale → DSP coincé en 0xebf0). Le dispatcher
     * lit une LUT par-tâche à data[(DP<<7)|0x07] = 0x9187, 0x9207, 0x9287…
     * (toujours offset 0x07 ; DP = numéro de tâche). Le `STH B,*AR2+` walking
     * (RPTB 0x815E-0x8176) corrompait 0x9207 (natif 0xff72 → 0xf6b7) →
     * CALAD-A=0x70c3 au lieu de 0x8239 → self-CALA → SP drain → snr=0.
     * On bloque uniquement les writes DROM à offset 0x07 (la colonne LUT) :
     * le scratch firmware est à d'autres offsets (0x00/0x42/0x60…), préservé. */
    /* DROM-LUT column protection — garde PMST.DROM RETIRÉE 2026-05-29.
     * Preuve (run 459M, /root/qemu.log) : PMST=0x70c4 vu 149× = PMST(MMR 0x1D)
     * lui-même clobbé par le self-CALA 0x70c3 (SP drain) → bit DROM(0x08)=0 →
     * la garde se désactivait elle-même → LUT 0x9207 re-corrompue → 0x70c3 →
     * boucle auto-entretenue (148838× le self-CALA). DROM-W-DROP n'avait fiché
     * que 2× (DROM encore =1) puis silence. Le firmware ne tourne JAMAIS
     * légitimement en DROM=0 (PMST légit = 0xffa8/0xffb8, DROM=1 dans les deux)
     * → offset 0x07 read-only SANS garde DROM. */
    if (addr >= 0x9000 && addr <= 0xDFFF && (addr & 0x7F) == 0x07) {
        static unsigned drom_w_attempts = 0;
        if (drom_w_attempts++ < 40) {
            if (calypso_debug_enabled("DROM-W-DROP")) fprintf(stderr,
                    "[c54x] DROM-W-DROP data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u pmst=0x%04x (LUT col, read-only)\n",
                    addr, val, s->data[addr], s->pc, s->insn_count, s->pmst);
        }
        return;
    }

    /* FBDB-PROBE write to 0x3DC0 (= SARAM flag polled by fc63 BITF).
     * Env CALYPSO_FBDB_PROBE=1. Logs old→new + which bits set, with focus
     * on bit 4 (= 0x0010) since that's the bit fc63 tests via BITF. */
    if (addr == 0x3DC0 && g_fbdb_probe_enabled > 0) {
        fbdb_probe_write_3dc0(addr, s->data[addr], val, s->pc, s->insn_count);
    }
    /* COEFFS-WR probe : watch-write sur la zone [0x2bc0..0x2bff] (64 mots).
     * 2026-05-14 evening — COEFFS-DUMP a montré une séquence init→clear→use :
     *   insn=2M  PC=0x9b05  vraies coeffs (f320, a660, ...)
     *   insn=3M  PC=0x9abc  pattern uniforme 0x0001 (suspect)
     *   insn=4M  PC=0x9abd  ALL ZERO (clear)
     *   insn=9M+ PC=0x8f51  ALL ZERO toujours (read par correlator)
     *
     * v2 (run précédent cap 200) : 3 clusters identifiés —
     *   0x8216 (wk=OTHER, 23 hits) = vraies coeffs
     *   0x9ace (wk=8, 64 hits)    = clear partiel
     *   0x9abf (wk=8x, 113 hits)  = pattern 0x0001
     * Cap atteint à insn=1.65M. On veut savoir si 0x8216 refire après.
     *
     * v3 (ce patch) :
     *   - Per-PC counter global sur tout le run (jamais capé)
     *   - Throttled log : 500 premiers + transitions PC + 1/100k insns
     *   - Summary périodique tous les 5M insns dump tous les PCs avec count>0
     * Tranche (1) re-fire 0x8216 manqué vs (2) one-shot définitif. */
    /* Timing trackers par cluster (utilisés par la sonde 0x8f51) — gardés
     * en dehors du helper car cluster-specific. Le watch_write_zone_check
     * factorise le compteur per-PC + log + summary. */
    if (addr >= 0x2bc0 && addr <= 0x2bff) {
        uint16_t exec_pc = s->last_exec_pc;
        if (exec_pc == 0x8216 || exec_pc == 0x8217 || exec_pc == 0x8218) {
            g_fb_det_timing.last_compute_insn = s->insn_count;
            g_fb_det_timing.last_compute_addr = addr;
        } else if (exec_pc == 0x9ace) {
            g_fb_det_timing.last_clear_insn = s->insn_count;
            g_fb_det_timing.last_clear_addr = addr;
        } else if (exec_pc == 0x9abf) {
            g_fb_det_timing.last_pattern_insn = s->insn_count;
            g_fb_det_timing.last_pattern_addr = addr;
        }
    }
    {
        static WatchWriteState wws_coeffs;
        watch_write_zone_check(s, addr, val, "COEFFS", 0x2bc0, 0x2bff, &wws_coeffs);
    }
    /* INVARIANT (gate CALYPSO_INVARIANTS, defaut off) : pointeurs du correlateur.
     * AR4 = write ptr -> doit prendre >2 valeurs distinctes (sinon boucle 2-mots).
     * AR5 = read ptr I/Q -> doit vivre dans le buffer [0x2a00..0x2b27] (sinon le
     * correlateur lit hors buffer = kernel 0xa076 jamais atteint, AR5=0xdb7b). */
    if (addr >= 0x2bc0 && addr <= 0x2bff) {
        static uint32_t pw;
        static uint16_t ar4_seen[8];
        static int ar4_n;
        uint16_t a4 = s->ar[4], a5 = s->ar[5];
        if (ar4_n < 8) {
            int f = 0;
            for (int i = 0; i < ar4_n; i++) { if (ar4_seen[i] == a4) { f = 1; break; } }
            if (!f) ar4_seen[ar4_n++] = a4;
        }
        if (++pw == 2000) {
            calypso_invariant("correlator_ar4_sweeps", ar4_n > 2,
                              "AR4 (write ptr) : %d valeur(s) distincte(s) sur %u writes",
                              ar4_n, pw);
        }
        calypso_invariant("correlator_ar5_in_iq_buffer",
                          a5 >= 0x2a00 && a5 <= 0x2b27,
                          "AR5 (read ptr I/Q) = 0x%04x HORS buffer [0x2a00..0x2b27]", a5);
    }
    /* A_CD-WR : a_cd[15] in NDB starts at DSP word 0x09D0 (= API byte 0x03A0,
     * = NDB byte offset 0x1F8). 15 words = [0x09D0..0x09DE].
     * Cible : tracker si le DSP CCCH demod (DSP_TASK_ALLC) écrit ses résultats. */
    {
        static WatchWriteState wws_a_cd;
        /* [2026-08-04] BORNES RE-CORRIGEES vers 0x09D2..0x09E0. La "correction"
         * du 30/07 vers 0x09D0..0x09DE etait une REGRESSION : elle surveillait
         * deux mots morts d'a_ramp et AVEUGLAIT a_cd[13] et a_cd[14], les deux
         * derniers mots des 23 octets remontes en L2. Quatre ancres independantes
         * donnent la base a 0x09D2 : (1) dsp_api.h preprocesse avec les vraies
         * macros de l1_environment.h -> offset 254 mots = 0xFE ; (2) ndb.h
         * (reverse IDA du binaire DSP charge) donne le meme 0xFE ; (3) B_BLUD se
         * teste sur a_cd[0] (prim_tch.c:667) et le DSP ecrit 0x8000 en 0x09D2 ;
         * (4) le bloc-copie de 12 mots vise 0x09D5 et l'ARM lit 23 octets a
         * &a_cd[3]. Coherent avec d_dsp_state=0x08E2 et NDB_D_FB_DET=0x08F8. */
        watch_write_zone_check(s, addr, val, "A_CD", 0x09d2, 0x09e0, &wws_a_cd);
    }
    /* ═════════════════════════════════════════════════════════════════════════
     * [2026-08-04] BLK-SRC — la SOURCE du bloc-copie, zone 0x2c3c..0x2c47.
     *
     * POURQUOI ICI. Depuis FIX_ALU3_DST, l'en-tete d'a_cd est repare (B_BLUD
     * survit au `or` de 0x9719, mesure : 86 x 0x8000, zero 0x0000 en 0x09d2).
     * Mais la CHARGE UTILE reste nulle, et elle ne vient pas de l'en-tete :
     * desassemblage du firmware en 0x971e..0x9723 —
     *     0x971e  stm #0x2c3c    ; AR2 <- source
     *     0x9720  stm #0x09d5    ; AR3 <- a_cd[3]
     *     0x9722  rpt #0x0b      ; 12 fois
     *     0x9723  mvdd           ; *AR2+ -> *AR3+
     * Les 12 mots de charge utile sont donc une COPIE de 0x2c3c..0x2c47. Si
     * a_cd est nul, c'est que cette zone l'est — ou que personne ne l'ecrit.
     *
     * CE QUE LA SONDE TRANCHE :
     *   - aucune ligne          -> personne n'ecrit la zone : le demod ne
     *                              publie pas son resultat, chercher en amont
     *                              (la boucle 0x81xx ecrit via *AR5+, pas a une
     *                              adresse fixe : suivre AR5) ;
     *   - des lignes a val=0    -> le demod ecrit, mais du vide : le probleme
     *                              est dans le calcul, pas dans le transport ;
     *   - des valeurs non nulles-> la zone est bonne et c'est le `mvdd` ou son
     *                              instant qui fautent.
     *
     * ⚠️ Meme helper que A_CD-WR, donc MEME condition d'emission
     * (`total <= 500 || exec_pc != last || delta_insn > 100000`) : ne JAMAIS
     * tirer un taux du nombre de lignes, lire le SUMMARY cumulatif.
     * ⚠️ `exec_pc` nomme l'instruction PRECEDENTE, pas l'ecrivain — corriger
     * mentalement avec `cur_pc`, comme pour 0x96db/0x96dd.
     * ═════════════════════════════════════════════════════════════════════════ */
    {
        static WatchWriteState wws_blk_src;
        watch_write_zone_check(s, addr, val, "BLK-SRC", 0x2c3c, 0x2c47, &wws_blk_src);
    }
    /* ═════════════════════════════════════════════════════════════════════════
     * [2026-08-04] DISP-TBL — la TABLE DE DISPATCH DES TACHES, 0x43d5..0x43d9.
     *
     * STRUCTURE ETABLIE PAR DESASSEMBLAGE (PROM0) :
     *   0xb4be  stm #0x43d5 ; ld #0xab35 ; rpt #0x02 ; reada *AR1+
     *           -> copie TROIS mots prog[0xab35..0xab37] vers data[0x43d5..0x43d7]
     *   0xbb00  st *(0x43d8), #0xab38      -> entree 3 = RET partage, par INIT
     *   0xb0e5  add #0x43d5, A ; ld *AR3, A ; cala A
     *           -> dispatch sur table[idx], avec garde 0 <= idx <= 3
     *   0xb01c  ld *(0x43d8), A ; cala A
     *           -> lit SPECIFIQUEMENT l'entree 3
     *
     * ⚠️ CE QUE CA CORRIGE DANS NOTRE LECTURE. Les 35 000 `CALAA tgt=0xab38`
     * ne sont PAS un dispatcher casse : `0x43d8` est l'entree de la TACHE 3, que
     * le chargeur ne copie volontairement pas (il n'en copie que 3), et que l'init
     * met a un RET. Un no-op par conception. Le vrai dispatch des taches 0..2
     * passe par 0xb0e5..0xb0ec.
     *
     * ⚠️ L'INDEX DE 0xb0e5 N'EST PAS `d_task_md`. Il vient de l'accumulateur A
     * APRES le `cala` de 0xb0d7 — c'est la valeur de RETOUR du dispatch precedent.
     * Ne pas comparer directement aux 1/5 que l'ARM ecrit en 0x0804.
     *
     * CE QUE LA SONDE TRANCHE : quelles valeurs contiennent reellement les quatre
     * entrees, et QUI les ecrit (y compris en adressage indirect, que le balayage
     * de litteraux de la ROM ne peut pas voir). Si 0x43d5..0x43d7 contiennent des
     * adresses plausibles de handlers, la table est saine et le probleme est dans
     * l'index ; si elles contiennent 0 ou des RET, c'est le chargeur qu'il faut
     * instruire.
     *
     * ⚠️ Meme helper que A_CD-WR, donc MEME condition d'emission — lire le
     * SUMMARY cumulatif, jamais le nombre de lignes.
     * ═════════════════════════════════════════════════════════════════════════ */
    {
        static WatchWriteState wws_disp_tbl;
        /* [2026-08-04] fenetre ELARGIE a 0x43d0..0x43dc : la version 0x43d5..0x43d9
         * ne voyait que 2 des 3 copies attendues, et ne pouvait pas dire si la
         * 3e manquait ou tombait HORS zone (decalage de pointeur). */
        watch_write_zone_check(s, addr, val, "DISP-TBL", 0x43d0, 0x43dc, &wws_disp_tbl);
    }
    /* ═════════════════════════════════════════════════════════════════════════
     * [2026-08-04] TRAMPO — le tremplin du vecteur 30, data[0x0158..0x015f].
     *
     * CHAINE ETABLIE PAR LA MESURE, maillon par maillon :
     *   1. le DMA leve INT10n en fin de transfert          -> 15 500 appels ✓
     *   2. le DSP ENTRE dans le vecteur 30                 -> mesure avec
     *      CALYPSO_DEBUG=C54X, `vec=30` present ✓
     *   3. le tremplin 0x0158 ne s'execute que 8 fois      -> ✗
     *   4. son corps `0x728a` ne s'execute JAMAIS          -> ✗
     * Les files du DSP debordent donc (PEND : 116 ecritures en 0x434e/0x434f,
     * TASK : 0) et il leve DSP_ERR_DMA_PEND — le temoin qu'on cherche a
     * supprimer en fournissant la fonction, pas en le masquant.
     *
     * CE QUE LA SONDE TRANCHE : qui ecrit la cellule du tremplin, quand, et avec
     * quelle valeur. La zone API recouvre l'espace PROGRAMME (CAL000 §7.2.1 :
     * « mixed data program memory », « API overlay over the program area »),
     * donc une ecriture de donnee y installe du CODE.
     *   - aucune ecriture -> le tremplin n'est jamais installe, et les 8
     *     executions viennent d'un residu ; chercher pourquoi 0xa5cd ne pose pas.
     *   - ecritures puis ecrasement -> quelqu'un le detruit entre deux prises du
     *     vecteur ; le PC ecrivain le nommera.
     *
     * ⚠️ Meme helper que A_CD-WR : MEME condition d'emission. Lire le SUMMARY
     * cumulatif, jamais le nombre de lignes.
     * ═════════════════════════════════════════════════════════════════════════ */
    {
        static WatchWriteState wws_trampo;
        watch_write_zone_check(s, addr, val, "TRAMPO", 0x0158, 0x015f, &wws_trampo);
        /* A_CD-BY-BURST : corrélation a_cd[] writes avec d_burst_d courant.
         * Si DSP fait burst 0→1→2→3 → ~25% des writes par burst_id.
         * Si on voit 0 writes avec burst=3 → DSP n'écrit jamais la fin de
         * séquence, d'où firmware ARM nb_resp bail (sous-cause #3). */
        if (addr >= 0x09d0 && addr <= 0x09de) {
            static uint64_t a_cd_by_burst[16];
            static uint64_t a_cd_corr_total;
            static uint64_t a_cd_corr_last_log;
            uint16_t b = g_last_d_burst_d & 0xF;
            a_cd_by_burst[b]++;
            a_cd_corr_total++;
            if (a_cd_corr_total - a_cd_corr_last_log >= 1000) {
                a_cd_corr_last_log = a_cd_corr_total;
                if (calypso_debug_enabled("A_CD-BY-BURST")) fprintf(stderr,
                        "[c54x] A_CD-BY-BURST total=%llu "
                        "burst[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                        (unsigned long long)a_cd_corr_total,
                        (unsigned long long)a_cd_by_burst[0],
                        (unsigned long long)a_cd_by_burst[1],
                        (unsigned long long)a_cd_by_burst[2],
                        (unsigned long long)a_cd_by_burst[3],
                        (unsigned long long)(a_cd_corr_total -
                                             a_cd_by_burst[0] - a_cd_by_burst[1] -
                                             a_cd_by_burst[2] - a_cd_by_burst[3]));
            }
        }
    }
    /* D_BURST_D probe (2026-05-15 midi) — watch d_burst_d à 0x0829 (page 0)
     * et 0x083D (page 1). Mesures : per-PC counter, transition matrix,
     * histogramme. Tranche la sous-cause :
     *   0,1,2,3 séquentiel → DSP signale correct, bug ARM-side
     *   0,1,2 jamais 3     → DSP cale au 4e burst (sous-cause #3)
     *   pas de write       → DSP n'écrit pas cette cellule du tout
     */
    if (addr == 0x0829 || addr == 0x083D) {
        static uint64_t db_total[2];
        static uint64_t db_per_pc[2][0x10000];
        static uint16_t db_prev[2];
        static uint64_t db_trans[2][16][16];
        static uint64_t db_last_log[2];
        static uint64_t db_last_summary[2];
        int page = (addr == 0x083D) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = db_prev[page];
        uint16_t curr_val = val & 0xF;
        db_total[page]++;
        db_per_pc[page][exec_pc]++;
        if (prev_val < 16 && curr_val < 16) {
            db_trans[page][prev_val][curr_val]++;
        }
        db_prev[page] = curr_val;
        g_last_d_burst_d = curr_val;  /* propage pour A_CD-BY-BURST */
        bool should_log = db_total[page] <= 200
            || (s->insn_count - db_last_log[page]) > 100000;
        if (should_log) {
            if (calypso_debug_enabled("D_BURST_D-WR")) fprintf(stderr,
                    "[c54x] D_BURST_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=%u curr=%u insn=%u\n",
                    page, (unsigned long long)db_total[page], addr, val,
                    exec_pc, prev_val, curr_val, s->insn_count);
            db_last_log[page] = s->insn_count;
        }
        if (s->insn_count - db_last_summary[page] >= 5000000) {
            db_last_summary[page] = s->insn_count;
            if (calypso_debug_enabled("D_BURST_D-SUMMARY")) fprintf(stderr,
                    "[c54x] D_BURST_D-SUMMARY page=%d total=%llu trans:",
                    page, (unsigned long long)db_total[page]);
            for (int p = 0; p < 8; p++) {
                for (int c = 0; c < 8; c++) {
                    if (db_trans[page][p][c]) {
                        fprintf(stderr, " %u->%u=%llu",
                                p, c, (unsigned long long)db_trans[page][p][c]);
                    }
                }
            }
            fprintf(stderr, "\n");
        }
    }
    /* D_TASK_D probe (2026-05-15 fin journée) — watch d_task_d à 0x0828
     * (page 0) et 0x083C (page 1), READ side de la struct db_buf_r.
     * ARM L1 prim_rx_nb lit ce champ via dsp_api.db_r->d_task_d et bail
     * avec puts("EMPTY") si == 0. 60 EMPTY printf observés sous synth=1
     * banc d'essai déterministe : DSP n'écrit jamais cette cellule (ou
     * écrit 0). Cette probe trace : qui écrit ? quand ? avec quelle valeur ?
     * Distinguer entre :
     *   - DSP ne touche jamais 0x0828/0x083C
     *   - DSP écrit val=0 (clear/init seulement)
     *   - DSP écrit val=24 (DSP_TASK_ALLC) mais ARM lit avant le write
     */
    if (addr == 0x0828 || addr == 0x083C) {
        static uint64_t dt_total[2];
        static uint16_t dt_prev[2];
        static uint64_t dt_last_log[2];
        int page = (addr == 0x083C) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = dt_prev[page];
        uint16_t curr_val = val;
        dt_total[page]++;
        dt_prev[page] = curr_val;
        bool should_log = dt_total[page] <= 200
            || (s->insn_count - dt_last_log[page]) > 100000;
        if (should_log) {
            if (calypso_debug_enabled("D_TASK_D-WR")) fprintf(stderr,
                    "[c54x] D_TASK_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=0x%04x insn=%u\n",
                    page, (unsigned long long)dt_total[page], addr, val,
                    exec_pc, prev_val, s->insn_count);
            dt_last_log[page] = s->insn_count;
        }
    }
    /* DATA-W-MMR : log every write into the low MMR window (addr <= 0x1F)
     * with full attribution context. Goal : disambiguate the IMR-W trace
     * cascade observed at PC=0x8eb9 (op=0xf3e1) and PC=0x9ad0 (op=0x8192).
     * The writer_kind field tells us *which path* triggered the write
     * (opcode family / IRQ ack / ARM MMIO / resolve_smem side effect).
     * Cap at 200 distinct events to avoid log flood. */
    if (addr <= 0x1F) {
        static unsigned mmrw_log;
        if (mmrw_log++ < 200) {
            const char *wk_name[] = {
                "UNK", "F3", "8x", "77", "76", "PSHM",
                "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
            };
            uint8_t wk = s->writer_kind;
            const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                              ? wk_name[wk] : "??";
            if (calypso_debug_enabled("DATA-W-MMR")) fprintf(stderr,
                    "[c54x] DATA-W-MMR addr=0x%02x val=0x%04x "
                    "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x "
                    "xpc=%d wk=%s "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "SP=%04x DP=%d INTM=%d insn=%u\n",
                    addr, val,
                    s->last_exec_pc, s->pc, s->prog[s->pc],
                    s->xpc, wkn,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    s->sp, dp(s),
                    !!(s->st1 & ST1_INTM),
                    s->insn_count);
        }
    }

    /* WATCH-WR-ADDR (générique, gated CALYPSO_DEBUG=WATCH-WR + env
     * CALYPSO_WATCH_WR_ADDR=0xNNNN) : log tout write vers une adresse data
     * arbitraire — pour tracer qui écrit (ou jamais) une cellule pointeur-
     * dispatcher SARAM (ex. 0x3af7) qui tombe à 0 → CALA→0 → bootstub. */
    /* [2026-07-29] Accepte desormais une LISTE d adresses separees par des
     * virgules ou des espaces : CALYPSO_WATCH_WR_ADDR=0x098b,0x098d. Une seule
     * adresse restait compatible. Motif : le dispatcher de taches du DSP
     * (0xddfd/0xde01) teste DEUX masques, d[0x098b] et d[0x098d], tous deux a
     * zero en permanence — il faut savoir si quelqu un les ecrit, et les deux
     * dans le meme run sinon on compare des runs differents. Max 8. */
    {
        static int wr_init = 0;
        static uint16_t wr_list[8];
        static int wr_n = 0;
        if (!wr_init) {
            wr_init = 1;
            const char *e = getenv("CALYPSO_WATCH_WR_ADDR");
            if (e && *e) {
                const char *p = e;
                while (*p && wr_n < 8) {
                    while (*p == ',' || *p == ' ' || *p == '\t') p++;
                    if (!*p) break;
                    char *fin = NULL;
                    long v = strtol(p, &fin, 0);
                    if (fin == p) break;          /* rien de lisible : on arrete */
                    wr_list[wr_n++] = (uint16_t)v;
                    p = fin;
                }
            }
        }
        for (int _i = 0; _i < wr_n; _i++) {
            if (addr == wr_list[_i]) {
                C54_DBG("WATCH-WR",
                    "WATCH-WR data[0x%04x] <- 0x%04x (was 0x%04x) PC=0x%04x "
                    "DP=0x%03x insn=%u",
                    addr, val, s->data[addr], s->pc, (s->st0 & 0x1FF),
                    (unsigned)s->insn_count);
                break;
            }
        }
    }

    /* [2026-09-18] WATCH-WR-PLAGE : journal chronologique de TOUTE ecriture
     * dans une plage data [CALYPSO_WATCH_WR_LO, CALYPSO_WATCH_WR_HI], plafonne
     * a CALYPSO_WATCH_WR_N lignes (defaut 600). Motif : etablir par la MESURE
     * l ordre des ecritures du tampon de bits souples 0x2a00-0x2a8d (qui ecrit
     * quoi, dans quel ordre, et si des zeros arrivent APRES les valeurs), au
     * lieu de le deduire d un comptage agrege.
     * Regle du PC (doc/SONDES.md) : le dispatcheur a deja avance le PC quand la
     * sonde l observe, donc on imprime pc=<vu> et ecr=<vu-1> = ecrivain probable.
     * Sortie inconditionnelle sur stderr (pas de C54_DBG) pour rester lisible
     * meme sans CALYPSO_DEBUG ; inerte tant que LO/HI ne sont pas poses. */
    {
        static int wrp_init = 0;
        static long wrp_lo = -1, wrp_hi = -1, wrp_max = 600;
        static long wrp_seen = 0;
        if (!wrp_init) {
            wrp_init = 1;
            const char *lo = getenv("CALYPSO_WATCH_WR_LO");
            const char *hi = getenv("CALYPSO_WATCH_WR_HI");
            const char *nn = getenv("CALYPSO_WATCH_WR_N");
            if (lo && *lo) wrp_lo = strtol(lo, NULL, 0);
            if (hi && *hi) wrp_hi = strtol(hi, NULL, 0);
            if (nn && *nn) wrp_max = strtol(nn, NULL, 0);
            if (wrp_lo >= 0 && wrp_hi < 0) wrp_hi = wrp_lo;
        }
        if (wrp_lo >= 0 && addr >= (uint16_t)wrp_lo && addr <= (uint16_t)wrp_hi) {
            wrp_seen++;
            if (wrp_seen <= wrp_max)
                fprintf(stderr, "[c54x] WR-PLAGE #%ld data[0x%04x] idx=%d <- 0x%04x "
                        "(was 0x%04x) pc=0x%04x ecr=0x%04x insn=%u\n",
                        wrp_seen, addr, (int)(addr - (uint16_t)wrp_lo), val,
                        s->data[addr], s->pc, (uint16_t)(s->pc - 1),
                        (unsigned)s->insn_count);
            else if (wrp_seen == wrp_max + 1)
                fprintf(stderr, "[c54x] WR-PLAGE ... plafond %ld atteint, suite muette\n",
                        wrp_max);
        }
    }

    /* WATCH-WRITE on the same mailbox slots tracked in data_read.
     * Whoever writes them — DSP or ARM via api_ram alias — gets logged
     * so we can attribute the source of the value the firmware polls. */
    /* WATCH-WRITE 0x3dd2 — la cellule sur laquelle 0x75db poll en boucle
     * (37M reads/15s). Identifier qui écrit (et qui ne le fait pas).
     * Cas 1 : zéro write → un bloc compute ne fire jamais.
     * Cas 2 : write boot only → init OK mais set steady-state manquant.
     * Cas 3 : writes périodiques avec valeur jamais matchée par le test
     *         à 0x75db → bug dans le compute en amont. */
    /* WATCH-3FBE — observ. writes sur la zone [0x3fb0..0x3fbf] qui inclut
     * 0x3fbe (le slot pop=0 du bootstub-entry insn=3995013). Le BSP DMA
     * bypasse data_write_locked (direct s->data[]) donc ce hook ne voit
     * QUE les writes via instructions DSP (STL/STM/STLM/STH). Si zéro
     * write vu avant insn=3995013 → le firmware ne pousse jamais à cet
     * addr → trajectoire SP divergente (RETD à 0x8ed1 sans CALL apparié).
     * Env-gated CALYPSO_WATCH_3FBE=1, zéro coût sinon. */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static int      w3fbe_enabled = -1;
        static unsigned w3fbe_total = 0;
        if (w3fbe_enabled < 0) {
            const char *e = cdbg_env("WATCH-3FBE");
            w3fbe_enabled = (e && *e == '1') ? 1 : 0;
            if (w3fbe_enabled) {
                fprintf(stderr,
                    "[c54x] WATCH-3FBE enabled — range [0x3fb0..0x3fbf] "
                    "(DSP-side writes only, BSP DMA bypassed)\n");
            }
        }
        if (w3fbe_enabled > 0) {
            w3fbe_total++;
            if (w3fbe_total <= 100 || (w3fbe_total % 5000) == 0) {
                fprintf(stderr,
                    "[c54x] WATCH-3FBE #%u addr=0x%04x val=0x%04x "
                    "(was 0x%04x) PC=0x%04x insn=%u\n",
                    w3fbe_total, addr, val, s->data[addr],
                    s->pc, s->insn_count);
            }
        }
    }

    if (addr == 0x3dd2) {
        static unsigned w3dd2;
        w3dd2++;
        if (w3dd2 <= 100 || (w3dd2 % 1000) == 0) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE 0x3dd2 #%u <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u INTM=%d\n",
                    w3dd2, val, s->data[addr], s->pc, s->insn_count,
                    !!(s->st1 & ST1_INTM));
        }
    }
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x01F0) {
        static unsigned wcount;
        if (wcount++ < 30) {
            if (calypso_debug_enabled("WATCH-WRITE")) fprintf(stderr,
                    "[c54x] WATCH-WRITE data[0x%04x] <- 0x%04x  (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher pointer at data[0x3f65] — `LD *(0x3f65),A; CALA A` at
     * DARAM 0x008a-0x008c. When this slot holds 0xfff8/0x0000/garbage the
     * CALA jumps into PROM1 vec or boot stub NOPs and the SP runs away.
     * Trace every write so we can identify who populates / corrupts it. */
    if (addr == 0x3f65) {
        static unsigned dpw;
        if (dpw++ < 100) {
            if (calypso_debug_enabled("DISP-PTR")) fprintf(stderr,
                    "[c54x] DISP-PTR data[0x3f65] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher poll addresses — log ANY write so we identify the
     * code path that should populate them. Currently 0 PORTR PA=0xF430
     * fires because dispatcher reads 0 here forever. */
    if (addr == 0x4359 || addr == 0x3fab) {
        static unsigned dispw;
        if (dispw++ < 50) {
            if (calypso_debug_enabled("DISP-WRITE")) fprintf(stderr,
                    "[c54x] DISP-WRITE data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* CALAD source zone 0x4180-0x41FF — LD-A-TRACE shows the firmware
     * reads 0x4189 (DP=0x83) but our emulation has it as 0. Log every
     * write to this range so we can tell whether (a) anyone is meant to
     * populate it and we missed the path, or (b) DP=0x83 is itself a
     * symptom upstream of an unrelated bug. */
    if (addr >= 0x4180 && addr <= 0x41FF) {
        static unsigned cwz;
        if (cwz++ < 5000) {
            if (calypso_debug_enabled("CALAD-ZONE-W")) fprintf(stderr,
                    "[c54x] CALAD-ZONE-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dedicated watch on 0x4189 — never capped. The LD-A loop reads this
     * slot in the CALAD trap; we want to know if/when *anyone* finally
     * writes a non-zero value, and from which PC. */
    if (addr == 0x4189) {
        fprintf(stderr,
                "[c54x] *** WR-0x4189 *** data[0x4189] <- 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                val, s->data[addr], s->pc, s->insn_count);
    }
    /* === DARAM[0x40..0x90] watch — dispatcher flag area ===
     * The PROM0 idle dispatcher (0xCC62..0xCC6F) polls data[0x62] and
     * other slots in [0x60..0x70]. FORCE-DARAM62=1 (env) proves that
     * setting data[0x62]=1 makes the DSP escape and reach 0x770c, so
     * this range gates the runtime task pipeline. ARM-side writes to
     * the API page mirror at +0x0800 (calypso_trx.c calypso_dsp_write)
     * but never to DARAM 0x40..0x90 — so any value here must come from
     * DSP-self stores (ST/STH/STM/...) or stay zero forever. Capture
     * EVERY write with PC+INTM+insn so we can attribute the source.
     * INTM annotation lets us tell ISR-context writes from main code. */
    if (addr >= 0x0040 && addr <= 0x0090) {
        static unsigned daram_disp_w;
        if (daram_disp_w++ < 1000) {
            if (calypso_debug_enabled("DISP-FLAG-W")) fprintf(stderr,
                    "[c54x] DISP-FLAG-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x INTM=%d IFR=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc,
                    !!(s->st1 & ST1_INTM), s->ifr, s->insn_count);
            if (daram_disp_w == 1000) {
                if (calypso_debug_enabled("DISP-FLAG-W")) fprintf(stderr,
                        "[c54x] DISP-FLAG-W log capped at 1000 — pattern visible above\n");
            }
        }
    }
    /* Timer registers (0x0024-0x0026) — before MMR check */
    if (addr == TCR_ADDR) {
        /* TRB: write 1 → reload TIM from PRD, PSC from TDDR */
        if (val & TCR_TRB) {
            s->data[TIM_ADDR] = s->data[PRD_ADDR];
            s->timer_psc = val & TCR_TDDR_MASK;
        }
        /* Store TCR without TRB (TRB is write-only, always reads 0) */
        s->data[TCR_ADDR] = val & ~TCR_TRB;
        return;
    }
    if (addr == TIM_ADDR) { s->data[TIM_ADDR] = val; return; }
    if (addr == PRD_ADDR) { s->data[PRD_ADDR] = val; return; }

    /* MMR region */
    if (addr < 0x20) {
        /* === BOOT-MMR-WR probe : reach+effect test for boot init STMs ===
         * The DSP boot is supposed to STM #imm into SP, IMR, AR0..7, BK,
         * BRC, PMST shortly after the jump from 0xb418→0x76f8. Observed
         * runtime says SP/IMR/AR4/AR5 never receive their init values, so
         * either (a) PC never reaches the STM, or (b) the STM handler writes
         * to the wrong target. This probe answers BOTH : every write to
         * MMR 0..0x1E during boot phase is logged with PC + opcode + delta,
         * so we can see what got written, when, by what instruction. */
        if (s->insn_count <= 300000) {
            static unsigned bmw_log;
            const unsigned LIMIT = 800;
            if (bmw_log < LIMIT) {
                static const char *names[0x20] = {
                    "IMR","IFR","??02","??03","??04","??05","ST0","ST1",
                    "AL","AH","AG","BL","BH","BG","T","TRN",
                    "AR0","AR1","AR2","AR3","AR4","AR5","AR6","AR7",
                    "SP","BK","BRC","RSA","REA","PMST","XPC","??1F",
                };
                uint16_t old_val = (addr == MMR_IMR) ? s->imr
                                 : (addr == MMR_IFR) ? s->ifr
                                 : (addr == MMR_SP)  ? s->sp
                                 : (addr >= MMR_AR0 && addr <= MMR_AR7) ? s->ar[addr - MMR_AR0]
                                 : s->data[addr];
                fprintf(stderr,
                        "[c54x] BOOT-MMR-WR #%u insn=%u PC=%04x op=%04x "
                        "MMR[%02x %s] %04x → %04x\n",
                        bmw_log, s->insn_count, s->pc, s->prog[s->pc],
                        (unsigned)addr, names[addr],
                        old_val, val);
                bmw_log++;
                if (bmw_log == LIMIT) {
                    fprintf(stderr,
                            "[c54x] BOOT-MMR-WR log capped at %u\n", LIMIT);
                }
            }
        }
        switch (addr) {
        case MMR_IMR:
            /* IMR-ARM (2026-06-24, RO, INCONDITIONNEL) : historique COMPLET des
             * ecritures IMR (old->new), etat bit12=vec28(scheduler)/bit3=vec19/
             * bit5=vec21. Run a montre IMR=0x52fd (bit12 SET!) efface par 0xb37e
             * -> on veut QUI ecrit 0x52fd (writer non-STM) + si un re-arm suit. Cap 80. */
            if ((uint16_t)val != s->imr) {
                static unsigned ia = 0;
                if (ia++ < 80)
                    fprintf(stderr, "[c54x] IMR-ARM 0x%04x -> 0x%04x (b12/vec28=%d "
                            "b3/vec19=%d b5/vec21=%d) PC=0x%04x op=0x%04x insn=%u\n",
                            s->imr, (uint16_t)val,
                            !!(val&(1<<12)), !!(val&(1<<3)), !!(val&(1<<5)),
                            s->pc, s->prog[s->pc], s->insn_count);
            }
            if (val != s->imr) {
                static unsigned imr_log = 0;
                /* Always log transitions TO zero (mask-everything) — that
                 * is the cascade root suspected in 2026-05-08 v2 diag :
                 * IMR=0 → INT3 IFR pending forever → RPTB at 0xe9ac never
                 * exits. We need the PC + opcode of every IMR=0 write,
                 * uncapped, so we can identify the buggy code path. */
                bool to_zero = (val == 0);
                if (imr_log++ < 50 || to_zero) {
                    if (calypso_debug_enabled("IMR-W")) fprintf(stderr,
                            "[c54x] IMR-W %s 0x%04x → 0x%04x PC=0x%04x "
                            "op=0x%04x prev_op=0x%04x SP=0x%04x INTM=%d "
                            "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                            "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x "
                            "B=0x%010llx insn=%u\n",
                            to_zero ? "*ZERO*" : "      ",
                            s->imr, val, s->pc,
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc - 1)],
                            s->sp,
                            !!(s->st1 & ST1_INTM),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->insn_count);
                }
            }
            {
                /* [2026-07-25] Le toggle IMR bit9 (0xddf9 ANDM / 0xde84 ORM) est
                 * PILOTE PAR LE TPU (masquage dynamique par fenetre burst), PAS a
                 * figer statiquement -> tentative PIN-IMR retiree (diag user).
                 * IMR = ce que le firmware/TPU ecrit (fidele). Le seul re-arm
                 * conserve est le fix bit5/BRINT0 dynamique (voir KEEP-IMR bloc
                 * ~13230) qui restaure bit5 uniquement quand il TOMBE. */
            }
            s->imr = val; return;
        case MMR_IFR: {
            /* [2026-07-23] IFR-CLEAR-W probe (unconditional, capped) : est-ce
             * que le code go-live ecrit directement IFR (write-1-to-clear) sur
             * bit5(BRINT0)/bit12(frame) SANS jamais dispatcher -- acquittement
             * logiciel qui jette le pending au lieu de le servir ? */
            static unsigned _ifrw = 0;
            if ((val & 0x1020) && _ifrw < 100) {
                _ifrw++;
                fprintf(stderr, "[c54x] IFR-CLEAR-W #%u val=0x%04x (clears bit5=%d bit12=%d) "
                        "ifr_before=0x%04x -> after=0x%04x PC=0x%04x insn=%u\n",
                        _ifrw, val, !!(val & 0x20), !!(val & 0x1000),
                        s->ifr, (uint16_t)(s->ifr & ~val), s->pc, s->insn_count);
            }
            {   /* [2026-07-30] IFR-CLEAR-WHO : site amont du helper, inline. */
                uint16_t _av = s->ifr;
                s->ifr &= (uint16_t)~val;
                uint16_t _pd = (uint16_t)(_av & ~s->ifr & s->imr);
                if (_pd) { static unsigned _n = 0;
                    if (_n++ < 40 || (_n % 5000) == 0)
                        fprintf(stderr, "[c54x] IFR-CLEAR-WHO #%u site=mmio-write "
                                "perdus=0x%04x IFR 0x%04x->0x%04x IMR=0x%04x "
                                "INTM=%d PC=0x%04x insn=%u\\n", _n, _pd, _av,
                                s->ifr, s->imr, (s->st1 & ST1_INTM) ? 1 : 0,
                                s->pc, s->insn_count);
                }
            }
            return;
        }
        case MMR_ST0:  s->st0 = val;
            /* DISP-ENTRY : trace restauration ST0 entière (POPM ST0/STLM) =
             * chemin NON-LDP qui change DP. C'est ICI que DP devient 0x087. */
            g_last_st0w_pc = s->pc; g_last_st0w_val = val;
            g_last_st0w_op = prog_fetch(s, s->pc); g_last_st0w_xpc = s->xpc;
            g_last_st0w_prev = g_prev_pc;
            st0_ring_rec(s, val, 'p'); /* pop/write ST0 (C-sweep) */
            if (g_orphan_on > 0 && (s->pc == 0xf48b || s->pc == 0x7737 || (val & 0x1FF) == 0x124)) {
                /* POPM ST0 @0xf48b : le slot juste poppé = data[sp-1]. Cherche
                 * son dernier écrivain dans le ring pile. NO-WRITER = slot stale
                 * → SP désaligné (POP sans PUSH apparié, cas b de CC). */
                uint16_t slot = (uint16_t)(s->sp - 1);
                fprintf(stderr, "[c54x] ORPHAN@%04x SP=0x%04x slot=0x%04x val=0x%04x(DP=%03x)",
                        s->pc, s->sp, slot, val, (unsigned)(val & 0x1FF));
                int found = 0;
                unsigned rn = g_stkw_idx < STKW_RING_N ? g_stkw_idx : STKW_RING_N;
                for (unsigned i = 0; i < rn; i++) {
                    StkwEv *e = &g_stkw_ring[(g_stkw_idx - 1 - i) % STKW_RING_N];
                    if (e->addr == slot) {
                        fprintf(stderr, "  WRITER@%04x op=%04x val=%04x", e->pc, e->op, e->val);
                        found = 1; break;
                    }
                }
                if (!found)
                    fprintf(stderr, "  NO-WRITER → slot STALE → SP désaligné (POP sans PUSH)");
                fprintf(stderr, " insn=%u\n", s->insn_count);
                /* SP-EVENTS ring : les push/pop récents → le return RET-family
                 * déséquilibré (pop delta>0 sans push apparié) qui décale SP. */
                fprintf(stderr, "[c54x]   ORPHAN-SP-RING (anciens→récents, pc:op±delta) :");
                for (int k = 28; k >= 1; k--) {
                    struct sp_evt *e = &g_spring[(g_spring_idx - k) & 63];
                    fprintf(stderr, " %04x:%04x%+d", e->pc, e->op, e->delta);
                }
                fprintf(stderr, "\n");
            }
            return;
        case MMR_ST1:  s->st1 = val; return;
        case MMR_AL:   s->a = (s->a & ~0xFFFF) | val; return;
        case MMR_AH:   s->a = (s->a & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_AG:   s->a = (s->a & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_BL:   s->b = (s->b & ~0xFFFF) | val; return;
        case MMR_BH:   s->b = (s->b & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_BG:   s->b = (s->b & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_T:    s->t = val; return;
        case MMR_TRN:  s->trn = val; return;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            ar_write_track(s, addr - MMR_AR0, val);  /* unified probe AR0..AR7 */
            s->ar[addr - MMR_AR0] = val; return;
        case MMR_SP:
            if (val >= 0x0800 && val < 0x0900) {
                if (calypso_debug_enabled("SP-GUARD")) fprintf(stderr,
                        "[c54x] SP-GUARD: refused MMR_SP write 0x%04x "
                        "(API mailbox); keeping 0x%04x PC=0x%04x\n",
                        val, s->sp, s->pc);
                return;
            }
            sp_abs_track(s, val, 0);  /* site=0 : MMR_SP via STL/STM/STLM */
            s->sp = val;
            return;
        case MMR_BK:
            /* PROBE 2026-06-01 : qui écrit BK ? (BK=0 casse l'adressage circulaire
             * → runaway AR2 0xfa98/0xf17c). Nomme le writer + valeur. À RETIRER. */
            {
                static uint32_t bkw_n = 0;
                if (bkw_n < 40) {
                    fprintf(stderr, "[c54x] BK-WR (MMR) 0x%04x→0x%04x PC=0x%04x op=0x%04x "
                            "%s insn=%u\n", s->bk, val, s->pc, prog_fetch(s, s->pc),
                            (val == 0) ? "<<< BK=0 (casse circular!)" : "", s->insn_count);
                    bkw_n++;
                }
            }
            s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            {
                /* PMST-WR (revival dsp 2026-06-22, RO) : un-gated — tranche « IPTR
                 * relocalisé 0x140 perdu vs jamais émis ». 300 premiers + TOUJOURS si 0x140. */
                {
                    static unsigned pmst_n = 0;
                    uint16_t niptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                    if (pmst_n < 300 || niptr == 0x140) {
                        pmst_n++;
                        fprintf(stderr, "[c54x] PMST-WR #%u val=0x%04x IPTR=0x%03x PC=0x%04x insn=%u%s\n",
                                pmst_n, val, niptr, s->pc, s->insn_count,
                                niptr == 0x140 ? "  <<< IPTR=0x140 EMITTED" : "");
                    }
                }
                static unsigned pmst_wr_attempts = 0;
                if (pmst_wr_attempts++ < 100)
                    C54_LOG("PMST WR attempt #%u: val=0x%04x cur=0x%04x PC=0x%04x insn=%u",
                            pmst_wr_attempts, val, s->pmst, s->pc, s->insn_count);
            }
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                {
                    static unsigned pmst_log = 0;
                    if (pmst_log++ < 100)
                        C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u #%u/100",
                                s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count, pmst_log);
                }

                static uint16_t last_dumped_iptr = 0xFFFF;
                static unsigned vecdump_count = 0;
                /* Cap at 8 dumps total — firmware may oscillate between 2-3
                 * IPTR values thousands of times during a session, and each
                 * dump emits 32 fprintf lines. Without cap : 250k+ log lines
                 * = saturates host I/O = bridge stops emitting CLK INDs =
                 * BTS shutdown "No more clock from transceiver". */
                if (new_iptr != last_dumped_iptr && vecdump_count < 8) {
                    vecdump_count++;
                    last_dumped_iptr = new_iptr;
                    uint32_t base = (uint32_t)new_iptr << 7;
                    uint16_t saved_pmst = s->pmst;
                    s->pmst = val;
                    C54_LOG("VECDUMP IPTR=0x%03x base=0x%04x (32 vectors) #%u/8:",
                            new_iptr, (uint16_t)base, vecdump_count);
                    for (int vec = 0; vec < 32; vec++) {
                        uint32_t a = base + vec * 4;
                        uint16_t w0 = prog_read(s, a + 0);
                        uint16_t w1 = prog_read(s, a + 1);
                        uint16_t w2 = prog_read(s, a + 2);
                        uint16_t w3 = prog_read(s, a + 3);
                        fprintf(stderr,
                                "[c54x] vec %2d @ 0x%04x : %04x %04x %04x %04x\n",
                                vec, (uint16_t)a, w0, w1, w2, w3);
                    }
                    s->pmst = saved_pmst;
                }
            }
            s->pmst = val; return;
        case MMR_XPC:
            {
                static int xpc_log = 0;
                if (xpc_log++ < 50)
                    C54_LOG("MMR_XPC WR val=0x%04x (was %d) PC=0x%04x SP=0x%04x insn=%u",
                            val, s->xpc, s->pc, s->sp, s->insn_count);
            }
            s->xpc = val & 3;
            return;
        default: return;
        }
    }

    /* [2026-07-29] Contrôleur DMA (calypso_dma.c) : il intercepte AVANT le
     * décodage historique ci-dessous et rend true s'il a traité l'écriture.
     * Éteint par défaut (CALYPSO_DMA), auquel cas il rend false et rien ne
     * change — c'est ce qui rend l'ajout sans risque de régression.
     * ⚠️ Les deux décodages ne s'accordent pas sur le mapping : SPRU131 place
     * DMPREC en 0x54, le code historique y met DMSA. Voir calypso_dma.h. */
    if (calypso_dma_mmr_write(s, addr, val)) {
        return;
    }

    /* DMA sub-register bank (C54x DMA controller).
     * DMSA (0x0054): sets the sub-register address.
     * DMSDI (0x0055): writes sub-register data, auto-increments DMSA.
     * DMSDN (0x0057): writes sub-register data, no auto-increment.
     * DMA channel 0 sub-registers (BSP receive DMA):
     *   sub 0x00=DMSRC0, 0x01=DMDST0, 0x02=DMCTR0, 0x03=DMMCR0 */
    if (addr == 0x0054) {
        s->dma_subaddr = val;
        s->data[0x0054] = val;
        return;
    }
    if (addr == 0x0055 || addr == 0x0057) {
        uint16_t sa = s->dma_subaddr;
        if (sa < 24) {  /* 6 channels × 4 regs */
            s->dma_subregs[sa] = val;
            int ch = sa / 4;
            int reg = sa % 4;
            static const char *rnames[] = {"SRC","DST","CTR","MCR"};
            C54_LOG("DMA ch%d %s = 0x%04x (sub 0x%02x) PC=0x%04x",
                    ch, rnames[reg], val, sa, s->pc);
        }
        s->data[addr] = val;
        if (addr == 0x0055) s->dma_subaddr++;  /* auto-increment */
        return;
    }

    /* McBSP sub-register bank (serial port extended config).
     * SPSA (0x0038): sub-address. SPSD (0x0039): sub-data. */
    if (addr == 0x0038 || addr == 0x0039) {
        if (addr == 0x0038) s->spsa = val;
        else {
            C54_LOG("McBSP sub[0x%02x] = 0x%04x PC=0x%04x", s->spsa, val, s->pc);
        }
        s->data[addr] = val;
        return;
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        uint16_t woff = addr - C54X_API_BASE;
        /* [2026-08-03] ARBITRAGE SAM / HOM — CAL000 §7.2.1, CAL207 §9.1.
         *
         * « In HOM mode (Host Only Mode), the API RAM is dedicated to external
         * access under the control of either the ARM or the DMA controller. »
         * Autrement dit : en HOM, le DSP N'A PLUS ACCES a cette fenetre. Le
         * modele n'en avait aucune notion — les deux cotes ecrivaient sans arbitre.
         *
         * MESURE qui a motive ce bloc : le firmware DSP bascule HOM<->SAM UNE FOIS
         * PAR TRAME (API_CONF=0x0002 en 0xa693, retour a 0x0000 en 0xa4e7).
         *
         * DEUX ETAGES, deliberement separes :
         *   - OBSERVATION (CALYPSO_API_HOM_WATCH, defaut 1) : on COMPTE et on
         *     journalise les ecritures DSP faites pendant HOM, sans rien changer.
         *     Tant qu'on ne sait pas si ca arrive, arbitrer serait speculer.
         *   - APPLICATION (CALYPSO_API_HOM_STRICT, defaut 0) : on ABANDONNE
         *     l'ecriture, comme le ferait le silicium. C'est un vrai changement de
         *     comportement, donc opt-in et a valider sous charge.
         * Le doc ne dit PAS ce que le DSP lit en HOM ; on ne touche donc pas au
         * chemin de lecture, inventer une valeur serait fabriquer une mesure. */
        {
            static int watch = -1, strict = -1;
            if (watch < 0) {
                watch  = calypso_gate("CALYPSO_API_HOM_WATCH", 1);
                strict = calypso_gate("CALYPSO_API_HOM_STRICT", 0);
                if (strict)
                    fprintf(stderr, "[c54x] API_HOM_STRICT=1 : les ecritures DSP "
                            "dans la fenetre API sont ABANDONNEES pendant HOM "
                            "(CAL000 §7.2.1). Changement de comportement — a "
                            "valider sous charge.\n");
            }
            if ((watch || strict) && calypso_xio_api_hom()) {
                static unsigned long long n_hom = 0;
                if (n_hom++ == 0 || (n_hom % 5000) == 0)
                    fprintf(stderr, "[c54x] API-HOM : ecriture DSP dans la fenetre "
                            "API pendant HOM #%llu — data[0x%04x] <- 0x%04x "
                            "PC=0x%04x (%s)\n", n_hom, addr, val, s->pc,
                            strict ? "ABANDONNEE" : "laissee passer, observation");
                if (strict)
                    return;
            }
        }
        if (s->api_ram)
            s->api_ram[woff] = val;
        {   /* [2026-07-28] FBDET-API (a) cote DSP : voir en-tete du patch. */
            static int _fa = -1; static unsigned _fan = 0;
            if (_fa < 0) _fa = calypso_gate("CALYPSO_FBDET_API", 0);
            if (_fa && woff >= 0xF8 && woff <= 0xFD && _fan < 40) {
                _fan++;
                fprintf(stderr, "[c54x] FBDET-API DSP api_ram[0x%02x] (mot 0x%04x, %s)"
                        " <- 0x%04x PC=0x%04x insn=%u\n", woff, addr,
                        woff == 0xF8 ? "d_fb_det" : "a_sync_demod",
                        val, s->pc, s->insn_count);
            }
        }
        /* === DSP→ARM STATUS / DEMOD probe (CCCH chain tracing) ===
         * Track DSP writes to the four critical mailbox regions :
         *   (1) a_pm[3]  + a_serv_demod[4]  on read page 0  : woff 0x30..0x36
         *   (2) a_pm[3]  + a_serv_demod[4]  on read page 1  : woff 0x44..0x4A
         *   (3) a_cd[15] CCCH demod result (CLAUDE.md DWARF) : woff 0x1C2..0x1D0
         *   (4) a_cd[15] CCCH demod result (shunt DWARF v2)  : woff 0x1D2..0x1E0
         * If a_serv_demod_WP* never appears → DSP never advances past
         * cell-search. If a_cd ranges stay silent → CCCH demod never
         * publishes (= bridge GMSK not converging, or task-md chain
         * never reaches CCCH). */
        {
            bool in_serv = (woff >= 0x0030 && woff <= 0x0036)
                        || (woff >= 0x0044 && woff <= 0x004A);
            bool in_acd  = (woff >= 0x01C2 && woff <= 0x01E0);
            if (in_serv || in_acd) {
                static unsigned cd_log;
                const unsigned LIMIT = 400;
                if (cd_log < LIMIT) {
                    const char *tag;
                    if (woff >= 0x0030 && woff <= 0x0032) tag = "A_PM_WP0";
                    else if (woff >= 0x0033 && woff <= 0x0036) tag = "A_SERV_DEMOD_WP0";
                    else if (woff >= 0x0044 && woff <= 0x0046) tag = "A_PM_WP1";
                    else if (woff >= 0x0047 && woff <= 0x004A) tag = "A_SERV_DEMOD_WP1";
                    else tag = "A_CD";
                    fprintf(stderr,
                            "[c54x] DSP-API-WR #%u %s woff=0x%04x val=0x%04x "
                            "PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                            cd_log, tag, woff, val, s->pc,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                            s->insn_count);
                    cd_log++;
                    if (cd_log == LIMIT) {
                        fprintf(stderr,
                                "[c54x] DSP-API-WR log capped at %u\n", LIMIT);
                    }
                }
            }
        }
        /* Notify the ARM-side mailbox watcher (calypso_trx) so it can
         * pulse IRQ_API, mirror to dsp_ram, and run the d_fb_det hook.
         * Without this, DSP writes to NDB cells are invisible to ARM. */
        if (s->api_write_cb)
            s->api_write_cb(s->api_write_cb_opaque, woff, val);
        /* Stack-corruption watch: stack push landing in the NDB
         * mailbox region [0x0800..0x08FF]. Only fires when SP has
         * already been corrupted into that range. */
        if (addr == s->sp && addr >= 0x0800 && addr < 0x0900) {
            if (calypso_debug_enabled("STACK-IN-NDB")) fprintf(stderr,
                    "[c54x] STACK-IN-NDB addr=0x%04x val=0x%04x SP=0x%04x "
                    "PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x\n",
                    addr, val, s->sp, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* Always log writes to d_dsp_page (0x08D4 ; 0x08E2 = d_dsp_state) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }

        /* d_spcx_rif (NDB word 2 = DSP data 0x08D6) — BSP serial port config */
        if (addr == 0x08D6) {
            C54_LOG("DSP WR d_spcx_rif = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_fb_det (NDB word 36 = DSP data 0x08F8). The DSP correlator
         * output here is treated as Q15-signed by the firmware FB-det
         * path — small unsigned BSIC was a wrong assumption. Log every
         * write unconditionally (thinned past 200) and dump the
         * adjacent NDB cells [0x08F0..0x0900] so we can see correlator
         * + flag + a_sync_demod fields together. */
        /* Silent NDB cells watch — d_fb_mode (binary "FB matched" flag,
         * THE actual trigger ARM tests), a_sync_PM (power), a_sync_SNR
         * (SNR). All read as 0 by ARM during 200M run despite d_fb_det
         * varying. Confirms: DSP never declares valid detection.
         * Three discriminating outcomes:
         *   (α) never written → "FB confirmed" code path unreached
         *   (β) written =0 explicitly → DSP scans, never matches threshold
         *   (γ) written !=0 but ARM reads 0 → coherence bug */
        /* W1C latch system removed 2026-05-28 (FBSB host-side synth purge).
         * The only env-gated override on the a_sync_demod read path is now
         * CALYPSO_FORCE_ANGLE_ZERO (calypso_trx.c). DSP writes pass
         * straight through to s->data[] and ARM reads them direct. */
        /* Full a_sync_demod + d_fb_mode WR watch — every cell, no PC
         * filter (so we catch real-fb-det writes AND stomp candidates).
         * Stomp zone PC=0x06xx tagged for easy grep. */
        if (addr == 0x08F9 || addr == 0x08FA ||
            addr == 0x08FB || addr == 0x08FC || addr == 0x08FD) {
            static unsigned ts_log[5] = {0};
            static uint16_t prev_d_fb_mode = 0xFFFF;
            int idx = (addr == 0x08F9) ? 0 :
                      (addr == 0x08FA) ? 1 :
                      (addr == 0x08FB) ? 2 :
                      (addr == 0x08FC) ? 3 : 4;
            const char *name = (idx == 0) ? "d_fb_mode"  :
                               (idx == 1) ? "a_sync_TOA" :
                               (idx == 2) ? "a_sync_PM"  :
                               (idx == 3) ? "a_sync_ANG" : "a_sync_SNR";
            ts_log[idx]++;
            bool transition = (idx == 0) &&
                              (prev_d_fb_mode != 0xFFFF) &&
                              (prev_d_fb_mode != val) &&
                              (val != 0 || prev_d_fb_mode != 0);
            bool stomp_zone = (s->pc >= 0x0600 && s->pc < 0x0700);
            bool log_it = transition ||
                          (idx == 0 && val != 0) ||
                          (val != 0 && ts_log[idx] <= 50) ||
                          (ts_log[idx] % 1000) == 0;
            if (log_it) {
                C54_LOG("DSP WR %s = 0x%04x (s=%d) PC=0x%04x%s insn=%u #%u%s",
                        name, val, (int)(int16_t)val, s->pc,
                        stomp_zone ? " [STOMP?]" : "",
                        s->insn_count, ts_log[idx],
                        transition ? " *TRANSITION*" : "");
            }
            if (idx == 0) prev_d_fb_mode = val;
        }
        if (addr == 0x08F8) {
            static unsigned fbd_log = 0;
            /* Filter out stack-stomp at d_fb_det: only PCs known to be
             * actual fb-det correlator stores (0x8d33, 0x8eb9, 0x8f51) get
             * the full per-write log + NDB+DARAM dumps. Other PCs (e.g.
             * 0xb906 push site, 0x7763/0x7764 SP-overflow) get a counted
             * one-line tag so we don't lose visibility on them, but they
             * stop polluting the watch stream. */
            bool real_fbdet = (s->pc == 0x8d33 || s->pc == 0x8eb9 ||
                               s->pc == 0x8f51);
            /* FBDET-DIVERSITY: count distinct values per 1M-insn window.
             * 1 = DSP pegged on stale data. >5 = real scan. Discriminates
             * "BSP delivers fresh I/Q" from "DSP recorrelates same window". */
            if (real_fbdet) {
                static uint16_t recent_vals[8] = {0};
                static unsigned next_window = 1000000;
                static int n_distinct = 0;
                int seen = 0;
                for (int i = 0; i < 8; i++) {
                    if (recent_vals[i] == val) { seen = 1; break; }
                }
                if (!seen) {
                    recent_vals[n_distinct & 7] = val;
                    n_distinct++;
                }
                if (s->insn_count >= next_window) {
                    C54_LOG("FBDET-DIVERSITY window=%uM distinct=%d",
                            next_window / 1000000, n_distinct);
                    n_distinct = 0;
                    for (int i = 0; i < 8; i++) recent_vals[i] = 0;
                    next_window = (s->insn_count / 1000000 + 1) * 1000000;
                }
            }
            if (real_fbdet && (fbd_log < 200 || (fbd_log % 1000) == 0)) {
                C54_LOG("DSP WR d_fb_det = 0x%04x (s=%d) PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                        val, (int)(int16_t)val, s->pc, s->insn_count,
                        s->prog[(uint16_t)(s->pc - 2)],
                        s->prog[(uint16_t)(s->pc - 1)],
                        s->prog[s->pc],
                        s->prog[(uint16_t)(s->pc + 1)]);
                C54_LOG("  NDB[0x08F0..0x0900]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        s->data[0x08F0], s->data[0x08F1], s->data[0x08F2], s->data[0x08F3],
                        s->data[0x08F4], s->data[0x08F5], s->data[0x08F6], s->data[0x08F7],
                        val,             s->data[0x08F9], s->data[0x08FA], s->data[0x08FB],
                        s->data[0x08FC], s->data[0x08FD], s->data[0x08FE], s->data[0x08FF],
                        s->data[0x0900]);
                if (fbd_log < 5) {
                    C54_LOG("  DARAM[0x3FB0..0x3FBF]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->data[0x3FB0], s->data[0x3FB1], s->data[0x3FB2], s->data[0x3FB3],
                            s->data[0x3FB4], s->data[0x3FB5], s->data[0x3FB6], s->data[0x3FB7],
                            s->data[0x3FB8], s->data[0x3FB9], s->data[0x3FBA], s->data[0x3FBB],
                            s->data[0x3FBC], s->data[0x3FBD], s->data[0x3FBE], s->data[0x3FBF]);
                }
            } else if (!real_fbdet) {
                static unsigned other_pc_count = 0;
                other_pc_count++;
                if (other_pc_count == 1 || other_pc_count == 100 ||
                    other_pc_count == 10000 || other_pc_count == 1000000) {
                    C54_LOG("d_fb_det NON-FBDET-PC write #%u val=0x%04x PC=0x%04x SP=0x%04x",
                            other_pc_count, val, s->pc, s->sp);
                }
            }
            /* === D_FB_DET ZERO-OVERRIDE TRACE ===
             * Race-window observed (memory `project_fbdet_threshold_blocker`):
             * DSP writes high SNR (e.g. 0x7902, 0x7766) at fb-det PCs, then
             * SOMETHING zeroes d_fb_det before ARM reads. ARM sees 200×
             * 0x0000 → no FB found → endless L1CTL_FBSB_REQ retries.
             *
             * Capture EVERY write of val=0 to 0x08F8 with full context so
             * we identify the zero-ifying PCs and reconstruct the condition
             * (threshold check, post-correlation reset, error path, etc.).
             * Cap 200 events. */
            if (val == 0) {
                static unsigned zero_log = 0;
                if (zero_log < 200) {
                    C54_DBG("FBDET", "D_FB_DET ZERO-WR #%u PC=0x%04x op=0x%04x prev=0x%04x "
                            "A=%010llx B=%010llx T=0x%04x ST0=0x%04x ST1=0x%04x insn=%u",
                            zero_log + 1,
                            s->pc, s->prog[s->pc],
                            s->data[0x08F8],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                            s->t, s->st0, s->st1, s->insn_count);
                    zero_log++;
                }
            }
            /* Transition trace : non-zero → zero (the override moment).
             * Logs whenever d_fb_det was non-zero just before this write
             * but the new write makes it zero. Cap 100. */
            if (val == 0 && s->data[0x08F8] != 0) {
                static unsigned override_log = 0;
                if (override_log < 100) {
                    C54_DBG("FBDET", "D_FB_DET OVERRIDE #%u prev=0x%04x → 0 PC=0x%04x op=0x%04x "
                            "A=%010llx ST0=0x%04x insn=%u",
                            override_log + 1,
                            s->data[0x08F8], s->pc, s->prog[s->pc],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            s->st0, s->insn_count);
                    override_log++;
                }
            }
            /* === NEW 2026-05-15 : SET non-zero trace + SET→CLEAR delta ===
             *
             * Symétrique au ZERO-WR. Capture chaque write de val != 0 à 0x08F8
             * pour identifier QUI set le FB found, et combien de cycles ça
             * tient avant qu'un PC clear ne l'écrase.
             *
             * Si delta SET→CLEAR < 100 insn → bug opcode tape immédiatement
             * (style POPM fix). Si delta = milliers d'insn → race timing
             * légitime entre DSP set et ARM read.
             */
            {
                static uint64_t last_set_insn;
                static uint16_t last_set_val;
                static uint16_t last_set_pc;
                static unsigned set_log_n = 0;
                static unsigned delta_log_n = 0;
                if (val != 0) {
                    /* SET event */
                    if (set_log_n < 500) {
                        C54_DBG("FBDET", "D_FB_DET SET #%u val=0x%04x PC=0x%04x op=0x%04x "
                                "prev=0x%04x A=%010llx insn=%u",
                                set_log_n + 1,
                                val, s->pc, s->prog[s->pc],
                                s->data[0x08F8],
                                (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                                s->insn_count);
                        set_log_n++;
                    }
                    last_set_insn = s->insn_count;
                    last_set_val = val;
                    last_set_pc = s->pc;
                } else if (s->data[0x08F8] != 0 && last_set_insn != 0) {
                    /* CLEAR after non-zero — log delta */
                    uint64_t delta = (uint64_t)s->insn_count - last_set_insn;
                    if (delta_log_n < 100) {
                        C54_LOG("D_FB_DET SET-TO-CLEAR-DELTA #%u "
                                "set_PC=0x%04x set_insn=%llu set_val=0x%04x "
                                "clear_PC=0x%04x clear_insn=%u delta=%llu cycles",
                                delta_log_n + 1,
                                last_set_pc, (unsigned long long)last_set_insn,
                                last_set_val, s->pc, s->insn_count,
                                (unsigned long long)delta);
                        delta_log_n++;
                    }
                }
            }
            fbd_log++;
        }
    }

    /* Log DARAM writes to code target area and count total */
    if (addr >= 0x0020 && addr < 0x0800) {
        static int dw_total = 0;
        dw_total++;
        if (addr >= 0x1200 && addr <= 0x1240) {
            C54_LOG("DARAM WR [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                    addr, val, s->pc, s->insn_count);
        }
        if (dw_total == 1 || dw_total == 100 || dw_total == 1000 || dw_total == 10000)
            C54_LOG("DARAM write count: %d (last: [0x%04x]=0x%04x)", dw_total, addr, val);
    }

    /* PROBE 2026-05-31 frame-IT : qui écrit les flags polled par le DSP wedgé
     * (data[0x006e], data[0x585f]) ? Tranche (a) ISR-relocate vs (b) HW-write.
     * Si AUCUN write ou valeur jamais "attendue" → flag jamais posé = deadlock.
     * À RETIRER après diag. */
    if (addr == 0x006e || addr == 0x585f || addr == 0x8a44) {
        static uint32_t fw_n = 0;
        if (fw_n < 80) {
            fprintf(stderr, "[c54x] FLAGWR data[0x%04x] 0x%04x→0x%04x PC=0x%04x "
                    "INTM=%d insn=%u\n", addr, s->data[addr], val, s->pc,
                    !!(s->st1 & ST1_INTM), s->insn_count);
            fw_n++;
        }
    }

    /* SBSLOT-WR probe (revival dsp 2026-06-22, read-only) : QUI ecrit le slot
     * SB a_serv_demod[D_TOA] db_r (p0 data[0x0830] / p1 data[0x0844]) et a_sch[3]
     * (p0 data[0x083a] / p1 data[0x084e]) ? Trou de couverture qui a induit le
     * faux "stale". Tranche scatter-write (PC=0xa1d6 ?) vs autre vs jamais ecrit.
     * Logue PC, op, A.low, val. Cape 300. */
    if (addr == 0x0830 || addr == 0x0844 || addr == 0x083a || addr == 0x084e) {
        static unsigned sbw_n = 0;
        if (sbw_n < 300) {
            const char *what = (addr == 0x0830) ? "SERV_TOA_p0" :
                               (addr == 0x0844) ? "SERV_TOA_p1" :
                               (addr == 0x083a) ? "A_SCH3_p0"   : "A_SCH3_p1";
            fprintf(stderr, "[c54x] SBSLOT-WR %s data[0x%04x] 0x%04x->0x%04x "
                    "PC=0x%04x op=0x%04x A=0x%010llx insn=%u\n",
                    what, addr, s->data[addr], val, s->pc,
                    prog_fetch(s, s->pc),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->insn_count);
            sbw_n++;
        }
    }

    /* PROBE 2026-05-31 : qui écrit d_fb_mode (0x08f9) avec du GARBAGE (>1) ?
     * Le détecteur tourne mais d_fb_mode=0x435b (≠0/1) → fenêtre/scaling faux.
     * Hypothèse : runaway AR2/BK=0 le corrompt. Nomme le corrupteur. À RETIRER. */
    if (addr == 0x08f9 && val > 1) {   /* GARBAGE uniquement (val ≠ 0/1) */
        static uint32_t fm_n = 0;
        if (fm_n < 40) {
            fprintf(stderr, "[c54x] FBMODE-GARBAGE data[0x08f9] 0x%04x→0x%04x PC=0x%04x "
                    "op=0x%04x SP=0x%04x AR2=%04x AR3=%04x BK=%04x %s insn=%u\n",
                    s->data[0x08f9], val, s->pc, prog_fetch(s, s->pc),
                    s->sp, s->ar[2], s->ar[3], s->bk,
                    (s->sp == 0x08f9) ? "<<< SP-PUSH" :
                    (s->ar[2] == 0x08f9 || s->ar[2] == 0x08f8) ? "<<< AR2-STORE" : "?",
                    s->insn_count);
            fm_n++;
        }
    }

    /* [2026-07-30] DISPATCH_INSTALL mode « init » — la bequille se GREFFE sur la
     * routine d'init au lieu de LUTTER contre elle.
     *
     * Pourquoi : en mode « dispatch » (l'historique), elle reinstalle data[0x43d8]
     * a CHAQUE passage du dispatcheur (exec_pc == 0xb01c), pendant que la routine
     * d'init 0xbb00 y remet son 0xab38. Mesure du 30/07 : 2894 reecritures par
     * 0xbb00 = ~1447 re-initialisations, et 156550 POST-BOOTSTUB-RET a PC=0x0000.
     * A/B decisif le meme soir : SANS la bequille, ZERO des deux. Le storm etait
     * donc AUTO-INFLIGE par la lutte, pas un defaut du firmware.
     *
     * Ici on intercepte l'ECRITURE d'init : 0xbb00 ecrit 0xab38, cette valeur est
     * vue telle quelle par le moniteur mailbox et par WATCH-WR (places plus haut,
     * donc la trace reste honnete), puis on substitue juste avant le store. Une
     * seule ecriture par init, aucun conflit, ordonnancement deterministe.
     *
     * @BEQUILLE — DISPATCH_INSTALL_AT=init  (avec CALYPSO_DISPATCH_INSTALL=0xNNNN)
     *   c'en est une : le slot ne porte plus ce que la ROM y a mis.
     *   masque  : la routine qui devrait installer un vrai handler dans 0x43d8.
     *   retirer : des qu'on sait QUI doit peupler 0x43d8 — le gate se tait deja si
     *             une valeur autre que 0xab38 apparait, donc il ne masquera pas
     *             l'arrivee d'un vrai installateur.
     */
    if (addr == 0x43d8) {
        static int _dg = -1, _dgv = -1; static unsigned _dgn = 0;
        if (_dg < 0) {
            const char *m = getenv("CALYPSO_DISPATCH_INSTALL_AT");
            _dg = (m && strcmp(m, "init") == 0) ? 1 : 0;
            const char *v = getenv("CALYPSO_DISPATCH_INSTALL");
            _dgv = (v && *v) ? (int)strtoul(v, NULL, 0) : -1;
            if (_dg && _dgv >= 0)
                fprintf(stderr, "[c54x] DISPATCH-GRAFT arme : toute ecriture de 0xab38 "
                        "dans data[0x43d8] devient 0x%04x (mode init, BEQUILLE — le "
                        "mode dispatch est desactive)\n", (unsigned)_dgv);
        }
        if (_dg && _dgv >= 0 && val == 0xab38) {
            if (_dgn++ < 20)
                fprintf(stderr, "[c54x] DISPATCH-GRAFT #%u : l'init (PC=0x%04x) ecrit "
                        "0xab38 dans data[0x43d8] -> greffe 0x%04x insn=%u\n",
                        _dgn, s->pc, (unsigned)_dgv, s->insn_count);
            val = (uint16_t)_dgv;
        }
    }

    s->data[addr] = val;
}

/* 23-bit program address translation : honors XPC for ≥0x8000 (extended
 * program memory / banked area), passes through for <0x8000 (common bank 0).
 * Shared by prog_fetch and prog_read so they cannot diverge again.
 * OVLY (DARAM mirror) is handled at call site because it routes to s->data[]. */
inline uint32_t c54x_prog_xlate(const C54xState *s, uint16_t addr16)
{
    /* FIX 2026-06-02 (ROOT CAUSE #2 — runaway 0xee00) : seule la fenêtre
     * 0x8000-0xDFFF est bankée par XPC (overlay PROM0/2/3). 0xE000-0xFFFF est
     * de la ROM FIXE (PROM1, mirror à prog[0xE000+]) — NON bankée sur le
     * silicium. L'ancien `>= 0x8000` appliquait XPC à 0xE000+ aussi → quand
     * XPC=3, PC=0xee00 fetchait prog[0x3ee00] (au-delà des 8K de PROM3) = vide
     * → exécution de PROM nulle (op=0x0000) → runaway de l'étage FB
     * post-corrélateur. La ROM haute doit ignorer XPC. */
    if (addr16 >= 0x8000 && addr16 < 0xE000) {
        return (((uint32_t)s->xpc << 16) | addr16) & (C54X_PROG_SIZE - 1);
    }
    return addr16;   /* 0x0000-0x7FFF on-chip + 0xE000-0xFFFF PROM1 ROM (XPC-indépendant) */
}

uint16_t prog_fetch(C54xState *s, uint16_t pc)
{
    if ((s->pmst & PMST_OVLY) && pc >= c54x_ovly_bas() && pc < 0x2800)
        return s->data[pc];
    return s->prog[c54x_prog_xlate(s, pc)];
}

/* [2026-08-22] Plancher de l alias OVLY. 0x0000-0x005F = registres mappes,
 * 0x0060-0x007F = SCRATCH-PAD DARAM. Le banc de filtres du SCH remplit
 * data[0x0060..0x0066] par MVDD (prologue 0x8336) puis relit ces coefficients
 * par FIRS avec pmad=0x0061..0x0064, donc via l espace PROGRAMME : cela n a de
 * sens que si le scratch-pad y est visible. Avec le plancher a 0x80 la lecture
 * retombait sur la ROM non chargee et rendait 0xF4E4 (mesure : FIRS #1
 * pmad=0x0063 coef(prog)=0xf4e4). Gate CALYPSO_OVLY_SCRATCH, defaut 1.
 * ⚠️ EFFET GLOBAL : l alias sert a tout ce qui s execute ou se lit en overlay. */
uint16_t c54x_ovly_bas(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_OVLY_SCRATCH", 1);
        fprintf(stderr, "[c54x] OVLY-SCRATCH %s : plancher de l alias programme "
                "a 0x%04x (scratch-pad DARAM 0x0060-0x007F %s)\n",
                g ? "ACTIF" : "INACTIF", g ? 0x0060 : 0x0080,
                g ? "VISIBLE en programme" : "hors alias");
    }
    return g ? 0x0060 : 0x0080;
}

/* [2026-08-23] CALYPSO_ISA_DUAL_SETT (defaut 1) — la famille duale
 * (mpy/macsu/mac/macr Xmem,Ymem) doit-elle ecrire T ?
 * binutils ne mentionne pas OP_T dans sa liste d operandes, alors que les
 * formes a operande unique l exposent (ld 0x3000 {OP_Smem,OP_T}). Or le
 * `mpy Smem,dst` de 0x81e4 DEPEND de T et le trouve a zero, alors que T est
 * charge en 0x815e : quelque chose l ecrase entre les deux.
 * A 0, la famille duale PRESERVE T -> on voit si le T de 0x815e survit.
 * ⚠️ Effet global : T sert a toutes les formes a operande unique. */
int c54x_dual_sett(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_ISA_DUAL_SETT", 1);
        fprintf(stderr, "[c54x] ISA-DUAL-SETT %s : la famille duale %s T\n",
                g ? "ACTIF (defaut)" : "INACTIF",
                g ? "ECRIT" : "PRESERVE");
    }
    return g;
}

/* [2026-08-23] Gates du lot de correctifs ISA issu du balayage exhaustif.
 * Chacun isole une famille pour permettre la bissection. Tous ont un EFFET
 * GLOBAL : ce sont des opcodes courants du DSP. */
int c54x_mpy_fam(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_ISA_MPY_FAM", 1);
        fprintf(stderr, "[c54x] ISA-MPY-FAM %s : mpyr 0x22xx / mpyu 0x24xx / "
                "squr 0x26xx de-permutes (T : mpyu ne l ecrit plus, squr l ecrit)\n",
                g ? "ACTIF" : "INACTIF (ancien decodage)");
    }
    return g;
}

int c54x_ld_par(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_ISA_LD_PAR", 1);
        fprintf(stderr, "[c54x] ISA-LD-PAR %s : 0xA800-0xAFFF = "
                "LD Xmem,dst || MAC/MACR/MAS/MASR Ymem, UN MOT (etait 2 mots "
                "sur trois des quatre -> desynchronisation)\n",
                g ? "ACTIF" : "INACTIF (ancien decodage)");
    }
    return g;
}

int c54x_mas_dual(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_ISA_MAS_DUAL", 1);
        fprintf(stderr, "[c54x] ISA-MAS-DUAL %s : mas/masr = src - Xmem*Ymem "
                "et T = Xmem (au lieu de T*Xmem puis T = Ymem ; masr etait POLY)\n",
                g ? "ACTIF" : "INACTIF (ancien decodage)");
    }
    return g;
}

uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    if ((s->pmst & PMST_OVLY) && addr16 >= c54x_ovly_bas() && addr16 < 0x2800)
        return s->data[addr16];
    return s->prog[c54x_prog_xlate(s, addr16)];
}

void prog_write(C54xState *s, uint32_t addr, uint16_t val)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* PROM1 (0xE000-0xFFFF) is ROM — reject writes */
    if (addr16 >= 0xE000) return;
    if ((s->pmst & PMST_OVLY) && addr16 >= c54x_ovly_bas() && addr16 < 0x2800)
        s->data[addr16] = val;
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        s->prog[ext] = val;
    }
    s->prog[addr16] = val;
}

/* ================================================================
 * Addressing mode helpers
 * ================================================================ */

/* MOD-MISMATCH probe helper (2026-06-01) : adressage circulaire canonique
 * C54x (SPRU172/tic54x-dis.c). step ±1 ou ±AR0, |step| <= BK attendu.
 * BK=0 → linéaire (règle #6396 : STM #0,BK délibéré, on ne wrappe pas).
 * NB : pur calcul de référence pour la sonde — N'ALTÈRE PAS l'exécution. */
/* [2026-07-29] FIX ISA — base du tampon circulaire.
 *
 * L'ancien calcul « base = ar - (ar % bk) » pose la base sur une grille de
 * multiples de BK. Le C54x ne fonctionne PAS ainsi (SPRU172, adressage
 * circulaire) : le tampon doit commencer à une adresse dont les N bits bas sont
 * nuls, avec 2^N >= BK ; l'index est ARn & (2^N - 1) et la base ARn & ~(2^N-1).
 *
 * Effet mesuré le 2026-07-29, file de requêtes DMA du firmware (BK=14) :
 *     anneau en 0x4340   ancien : 0x4340 % 14 = 10 -> base 0x4336, index 10
 *                        correct : masque 0x0F     -> base 0x4340, index 0
 * L'emulateur croyait donc l'anneau en 0x4336..0x4343, A CHEVAL sur 0x433e et
 * 0x433f — les deux pointeurs de l'AUTRE file. Le pointeur de la file 2 se
 * promenait dans 0x433d..0x433f et ecrasait la file 1 : comptabilite corrompue,
 * puis « orm *(0x3f92), #0x0008 » en 0xaa83 = DSP_ERR_DMA_PROG, remonte a l'ARM
 * (« DSP Error Status: 8 », 605 fois).
 *
 * ⚠️ Comme tout correctif d'ISA, l'effet est GLOBAL : l'adressage circulaire
 * sert aussi aux filtres et aux tampons I/Q. Bascule A/B pour comparer :
 * CALYPSO_CIRC_BASE_MOD=1 restaure l'ancien calcul. Verifier camp+LU+SMS en
 * shunt_legit, comme pour LDK8_SHIFT16. */
uint16_t c54x_circ_ref(uint16_t ar, int step, uint16_t bk)
{
    if (bk == 0) return (uint16_t)(ar + step);

    static int _vieux = -1;
    if (_vieux < 0) _vieux = calypso_gate("CALYPSO_CIRC_BASE_MOD", 0);

    uint16_t base, masque;
    int idx;

    if (_vieux) {
        base = (uint16_t)(ar - (ar % bk));
        idx  = (int)(ar % bk) + step;
    } else {
        unsigned n = 0;
        while ((1u << n) < (unsigned)bk) n++;   /* N tel que 2^N >= BK */
        masque = (uint16_t)((1u << n) - 1u);
        base   = (uint16_t)(ar & (uint16_t)~masque);
        idx    = (int)(ar & masque) + step;
    }
    if (idx >= (int)bk) idx -= bk;
    else if (idx < 0)   idx += bk;
    return (uint16_t)(base + idx);
}

/* [2026-08-22] Post-modification d'un AR pour les familles dual-operand et
 * parallele (SPRU131G Table 5-8) :
 *   00 = *ARi (aucune)   01 = *ARi-   10 = *ARi+   11 = *ARi+0% (circulaire BK)
 * Utilise c54x_circ_ref, la reference canonique corrigee le 2026-07-29 (base
 * alignee sur 2^N, PAS sur une grille de multiples de BK). */
void c54x_par_postmod(C54xState *s, int ar, int mod)
{
    switch (mod) {
    case 1: s->ar[ar]--; break;
    case 2: s->ar[ar]++; break;
    case 3: s->ar[ar] = c54x_circ_ref(s->ar[ar], +(int16_t)s->ar[0], s->bk); break;
    default: break;
    }
}

/* ================================================================
 * [2026-08-22] FIX ISA — adressage a BITS INVERSES (reverse carry)
 *     mode 7 = *ARn+0B      mode 4 = *ARn-0B
 *
 * RACINE. Ces deux modes etaient traites comme un +/-AR0 PLAT. Le commentaire
 * « GAP bitrev » qui les accompagnait posait deja le diagnostic et la condition
 * de levee : « tout chemin FCCH/SCH bit-reverse mal-adresserait EN SILENCE.
 * A implementer si une sonde le montre exerce en bitrev. »
 * La sonde manquante : PDROM 0xf1b3 `mar *AR2+0B` (smem=0xBA -> mod=7, AR2),
 * DANS la region du correlateur FB/SB, juste apres la boucle de correlation
 * (0xf16c..0xf17e) et la passe `norm` de 512 mots (0xf195..0xf19b) : c'est le
 * bit-reversal d'une FFT.
 *
 * POURQUOI CA COLLE AU SYMPTOME. Une FFT dont la retenue inversee est ignoree
 * rend des magnitudes et une phase plausibles mais un ORDRE DE BINS faux :
 * l'angle converge (mesure : -6 Hz stable) tandis que la POSITION du pic est
 * fausse ou collee au bord de fenetre (mesure : TOA=39 = bord r39, ou aberrant).
 *
 * SEMANTIQUE (SPRU131G) : la retenue se propage vers les bits de POIDS FAIBLE
 * au lieu des poids forts. Equivalent exact et sans boucle : inverser les bits
 * des deux operandes, additionner normalement, re-inverser le resultat.
 *
 * /!\ EFFET GLOBAL : ces modes servent a toute FFT/IFFT du firmware.
 * Bascule A/B : CALYPSO_ISA_BITREV=0 restaure le +/-AR0 plat.
 * ================================================================ */
static int c54x_bitrev_on(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_ISA_BITREV", 1);
        fprintf(stderr, "[c54x] ISA-BITREV %s : *ARn+0B / *ARn-0B = retenue "
                "INVERSEE (etait un +/-AR0 plat)\n",
                g ? "ACTIF (fidele SPRU131G)" : "INACTIF (ancien +/-AR0 plat)");
    }
    return g;
}

static inline uint16_t c54x_bitrev16(uint16_t v)
{
    v = (uint16_t)(((v & 0xAAAAu) >> 1) | ((v & 0x5555u) << 1));
    v = (uint16_t)(((v & 0xCCCCu) >> 2) | ((v & 0x3333u) << 2));
    v = (uint16_t)(((v & 0xF0F0u) >> 4) | ((v & 0x0F0Fu) << 4));
    v = (uint16_t)(((v & 0xFF00u) >> 8) | ((v & 0x00FFu) << 8));
    return v;
}

uint16_t c54x_revcarry(C54xState *s, uint16_t ar, uint16_t ar0, int sub)
{
    uint16_t plat = (uint16_t)(sub ? (ar - ar0) : (ar + ar0));
    uint16_t r;
    if (!c54x_bitrev_on())
        return plat;
    r = c54x_bitrev16(ar);
    r = (uint16_t)(sub ? (r - c54x_bitrev16(ar0)) : (r + c54x_bitrev16(ar0)));
    r = c54x_bitrev16(r);
    {   /* Sonde : prouve que le mode est REELLEMENT exerce, et montre l'ecart
         * avec l'ancien calcul plat. Plafonnee — ne jamais en tirer un taux. */
        static unsigned n = 0;
        if (n < 24) {
            n++;
            fprintf(stderr, "[c54x] BITREV #%u PC=0x%04x AR=0x%04x %c AR0=0x%04x "
                    "-> 0x%04x (plat: 0x%04x) insn=%u\n",
                    n, s->pc, ar, sub ? '-' : '+', ar0, r, plat, s->insn_count);
        }
    }
    return r;
}

/* Resolve Smem operand: direct or indirect addressing.
 * Returns the data memory address. */
