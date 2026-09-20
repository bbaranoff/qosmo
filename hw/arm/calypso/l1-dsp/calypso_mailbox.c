/*
 * calypso_mailbox.c - full ARM <-> DSP mailbox monitor.
 *
 * See calypso_mailbox.h for the rationale. Configuration:
 *
 *   CALYPSO_MAILBOX=1|all     everything crossing the mailbox window
 *                             (0x0800..0x0FFF)
 *   CALYPSO_MAILBOX=w         writes only (both directions)
 *   CALYPSO_MAILBOX=r         reads only
 *   CALYPSO_MAILBOX_CELLS=0x098b,0x43d8,...   ADDS cells outside the window,
 *                             watched in both directions and on both sides
 *   CALYPSO_MAILBOX_RANGES=lo-hi,...          ADDS address ranges, bounds
 *                             included
 *   CALYPSO_MAILBOX_ONLY=1    trace ONLY the cells and ranges listed above
 *   CALYPSO_MAILBOX_BRUT=1    disable folding, log every event
 *   CALYPSO_MAILBOX_FILE=...  defaults to $LOG_DIR/mailbox.log, else
 *                             /tmp/calypso/logs/mailbox.log
 *   CALYPSO_MAILBOX_MAX=N     safety cap (default 5 000 000 lines, 0 = none)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_mailbox.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hw/arm/calypso/calypso_debug.h"

int calypso_mbx_actif = -1;   /* -1 = initialise on first access */

static FILE    *g_f;
static int      g_init;
static int      g_ecr = 1, g_lec = 1;      /* writes / reads */
static int      g_only;                    /* 1 = _CELLS/_RANGES only */
static uint16_t g_cells[32];
static int      g_ncells;
/* Arbitrary RANGES: the API window alone left whole regions of the model out of
 * reach (the correlation reference at 0x2cea.., the burst buffer at 0x2a00..,
 * the scratch-pad at 0x0060..), and 32 single cells cannot cover a 128-word
 * region. */
static struct { uint16_t lo, hi; } g_ranges[16];
static int      g_nranges;
static unsigned long g_n, g_max = 5000000UL;

/* PER-CELL folding: log on CHANGE, not on non-consecutive repeats.
 *
 * Folding identical CONSECUTIVE events folds nothing here: the DSP background
 * loop polls two cells in alternation (0xde86 ld *(0x098c) then 0xddfd ld
 * *(0x098d)), so no two successive lines are ever identical. Measured that way:
 * 291 MB and 3 690 446 lines in a few minutes, and QEMU killed.
 *
 * Per cell is the right granularity: a read returning the SAME value at the
 * SAME PC carries no new information, even with other accesses interleaved. So
 * keep, per word, the last logged value and context, and emit only on change,
 * summarising the run just closed as "x N". CALYPSO_MAILBOX_BRUT=1 turns all
 * folding off. */
static int       g_brut;
static uint16_t *g_dval;      /* last logged value, per word */
static uint32_t *g_dctx;      /* last logged context, per word */
static uint32_t *g_drep;      /* length of the run in progress, per word */
static uint8_t  *g_dvu;       /* has this word been logged at all? */
static uint8_t  *g_dsens;     /* direction of the last logged event */

/* Names for the cells that keep coming up in diagnosis: a readable trace saves
 * looking up "0x08fa, what was that again" on every read. */
static const char *mbx_nom(uint16_t m)
{
    switch (m) {
    case 0x0804: return "d_task_md/wp0";
    case 0x0818: return "d_task_md/wp1";
    case 0x0828: return "d_task_d/rp0";
    case 0x0829: return "d_burst_d/rp0";
    case 0x083C: return "d_task_d/rp1";
    case 0x083D: return "d_burst_d/rp1";
    case 0x0810: return "d_ctrl_system";
    /* NDB+0 = 0x08D4 = d_dsp_page; NDB+14 = 0x08E2 = d_dsp_state, where the
     * ARM writes 3 = C_DSP_IDLE3 from dsp.c:215. See calypso_fbsb.h. */
    case 0x08D4: return "d_dsp_page";
    case 0x08D5: return "d_error_status";
    case 0x08E2: return "d_dsp_state";
    case 0x08F8: return "d_fb_det";
    case 0x08F9: return "d_fb_mode";
    case 0x08FA: return "a_sync[TOA]";
    case 0x08FB: return "a_sync[PM]";
    case 0x08FC: return "a_sync[ANGLE]";
    case 0x08FD: return "a_sync[SNR]";
    case 0x098A: return "d_backgnd_en";
    case 0x098B: return "d_backgnd_?b";
    case 0x098C: return "d_backgnd_st";
    case 0x098D: return "d_backgnd_?d";
    case 0x098E: return "tab_handlers";
    case 0x0FFF: return "d_task_word";
    case 0x43D8: return "slot_handler";
    default:     return "";
    }
}

static void mbx_ecrire(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                       uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn,
                       unsigned long rep);

void calypso_mbx_init(void)
{
    const char *e;

    if (g_init) {
        return;
    }
    g_init = 1;

    e = calypso_getenv("CALYPSO_MAILBOX");
    if (!e || !*e || !strcmp(e, "0")) {
        calypso_mbx_actif = 0;        /* off: zero cost on the hot paths */
        return;
    }
    if (!strcmp(e, "w") || !strcmp(e, "W")) {
        g_lec = 0;
    } else if (!strcmp(e, "r") || !strcmp(e, "R")) {
        g_ecr = 0;
    }

    e = calypso_getenv("CALYPSO_MAILBOX_CELLS");
    if (e && *e) {
        const char *p = e;
        while (*p && g_ncells < 32) {
            char *fin = NULL;
            long v;
            while (*p == ',' || *p == ' ') {
                p++;
            }
            if (!*p) {
                break;
            }
            v = strtol(p, &fin, 0);
            if (fin == p) {
                break;                /* nothing parseable: stop here */
            }
            g_cells[g_ncells++] = (uint16_t)v;
            p = fin;
        }
    }
    /* CALYPSO_MAILBOX_RANGES=lo-hi,lo-hi,... , bounds INCLUDED. Same tolerant
     * parse as _CELLS: stop at the first unreadable token rather than guess.
     * Bounds are swapped back into order when needed. */
    e = calypso_getenv("CALYPSO_MAILBOX_RANGES");
    if (e && *e) {
        const char *p = e;
        while (*p && g_nranges < 16) {
            char *fin = NULL;
            long lo, hi;
            while (*p == ',' || *p == ' ') {
                p++;
            }
            if (!*p) {
                break;
            }
            lo = strtol(p, &fin, 0);
            if (fin == p) {
                break;
            }
            p = fin;
            if (*p == '-' || *p == ':') {
                p++;
                hi = strtol(p, &fin, 0);
                if (fin == p) {
                    break;
                }
                p = fin;
            } else {
                hi = lo;              /* a lone bound = a single cell */
            }
            if (lo > hi) { long t = lo; lo = hi; hi = t; }
            g_ranges[g_nranges].lo = (uint16_t)lo;
            g_ranges[g_nranges].hi = (uint16_t)hi;
            g_nranges++;
        }
    }

    e = calypso_getenv("CALYPSO_MAILBOX_ONLY");
    g_only = (e && *e == '1');

    e = calypso_getenv("CALYPSO_MAILBOX_BRUT");
    g_brut = (e && *e == '1');

    e = calypso_getenv("CALYPSO_MAILBOX_MAX");
    if (e && *e) {
        g_max = strtoul(e, NULL, 0);
    }

    e = calypso_getenv("CALYPSO_MAILBOX_FILE");
    if (!e || !*e) {
        static char def[512];
        /* The repo exports LOG_DIR (paths.env). CALYPSO_LOG_DIR is accepted as
         * a fallback, but LOG_DIR wins. */
        const char *d = calypso_getenv("LOG_DIR");
        if (!d || !*d) d = calypso_getenv("CALYPSO_LOG_DIR");
        snprintf(def, sizeof(def), "%s/mailbox.log",
                 (d && *d) ? d : "/tmp/calypso/logs");
        e = def;
    }
    g_f = fopen(e, "w");
    if (!g_f) {
        fprintf(stderr, "[mbx] ouverture impossible : %s — moniteur INACTIF\n", e);
        calypso_mbx_actif = 0;
        return;
    }
    /* FULL buffering, and wide. Under _IOLBF every line was a syscall, and
     * 3.7 M write() calls were enough to bring QEMU to its knees. Flush
     * periodically (see mbx_ecrire) so an abrupt exit loses one window at
     * most. */
    setvbuf(g_f, NULL, _IOFBF, 1 << 20);

    g_dval = calloc(0x10000, sizeof(*g_dval));
    g_dctx = calloc(0x10000, sizeof(*g_dctx));
    g_drep = calloc(0x10000, sizeof(*g_drep));
    g_dvu  = calloc(0x10000, sizeof(*g_dvu));
    g_dsens = calloc(0x10000, sizeof(*g_dsens));
    if (!g_dval || !g_dctx || !g_drep || !g_dvu || !g_dsens) {
        fprintf(stderr, "[mbx] allocation impossible — moniteur INACTIF\n");
        calypso_mbx_actif = 0;
        return;
    }

    fprintf(g_f,
        "# moniteur mailbox ARM<->DSP — un evenement par ligne, format stable\n"
        "# sens : ARM>WR (commande)  ARM<RD (resultat)  DSP>WR (resultat)  DSP<RD (commande)\n"
        "# %-10s %-8s %-6s %-6s %-14s %s\n",
        "insn", "fn", "sens", "mot", "nom", "valeur");
    fflush(g_f);

    fprintf(stderr, "[mbx] moniteur mailbox ACTIF -> %s "
            "(ecr=%d lec=%d cellules_sup=%d only=%d max=%lu)\n",
            e, g_ecr, g_lec, g_ncells, g_only, g_max);

    /* Announce what is ACTUALLY covered: without this there is no telling
     * "the cell never moved" from "the cell was never watched". */
    {
        int i;
        fprintf(stderr, "[mailbox] couverture : %s",
                g_only ? "SEULEMENT les cellules et plages listees"
                       : "fenetre API 0x0800-0x0FFF");
        for (i = 0; i < g_ncells; i++) {
            fprintf(stderr, " + 0x%04x", g_cells[i]);
        }
        for (i = 0; i < g_nranges; i++) {
            fprintf(stderr, " + [0x%04x-0x%04x, %u mots]",
                    g_ranges[i].lo, g_ranges[i].hi,
                    (unsigned)(g_ranges[i].hi - g_ranges[i].lo + 1));
        }
        fprintf(stderr, " | sens : %s%s | garde-fou : %lu ligne(s)%s\n",
                g_ecr ? "ecritures " : "", g_lec ? "lectures" : "",
                g_max, g_max ? "" : " (illimite)");
    }
    calypso_mbx_actif = 1;
}

/* In the mailbox window, or in the extra cell/range list? */
static int mbx_retenu(uint16_t mot)
{
    int i;

    for (i = 0; i < g_ncells; i++) {
        if (g_cells[i] == mot) {
            return 1;
        }
    }
    for (i = 0; i < g_nranges; i++) {
        if (mot >= g_ranges[i].lo && mot <= g_ranges[i].hi) {
            return 1;
        }
    }
    if (g_only) {
        return 0;
    }
    return (mot >= 0x0800 && mot <= 0x0FFF);
}

void calypso_mbx_evt(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                     uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn)
{
    int est_ecr;

    if (!g_init) {
        calypso_mbx_init();
    }
    if (!g_f) {
        return;
    }

    est_ecr = (sens == MBX_ARM_WR || sens == MBX_DSP_WR);
    if ((est_ecr && !g_ecr) || (!est_ecr && !g_lec)) {
        return;
    }
    if (!mbx_retenu(mot)) {
        return;
    }

    if (g_max && g_n >= g_max) {
        if (g_n == g_max) {
            g_n++;
            fprintf(g_f, "# PLAFOND %lu lignes atteint — trace interrompue. "
                         "CALYPSO_MAILBOX_MAX=0 pour l'enlever.\n", g_max);
            fflush(g_f);
        }
        return;
    }
    g_n++;

    if (g_brut) {
        mbx_ecrire(sens, mot, val, avant, ctx, fn, insn, 0);
        return;
    }

    /* A write that CHANGES the value is always an event: it passes
     * unconditionally and becomes the new reference.
     *
     * Writes that change nothing do not: measured on a native run, 87 174 of
     * 92 150 lines were the DSP rewriting 0x0000 -> 0x0000 from the same PC
     * (@0xb446), once per frame. A write with no effect from the same PC
     * carries no more information than a read, so both follow the same rule. */
    if (est_ecr && avant != val) {
        if (g_drep[mot] > 1) {
            mbx_ecrire(g_dsens[mot], mot, g_dval[mot], g_dval[mot], g_dctx[mot],
                       fn, insn, g_drep[mot] - 1);
        }
        g_drep[mot] = 0;
        mbx_ecrire(sens, mot, val, avant, ctx, fn, insn, 0);
        g_dval[mot]  = val;
        g_dctx[mot]  = ctx;
        g_dsens[mot] = sens;
        g_dvu[mot]   = 1;
        return;
    }

    /* Otherwise - a read, or a write with no effect: nothing new if both the
     * value and the context match. Just count it. */
    if (g_dvu[mot] && g_dval[mot] == val && g_dctx[mot] == ctx) {
        g_drep[mot]++;
        g_n--;                        /* does not count against the cap */
        return;
    }

    /* Change: close the previous run with a summary, then log. */
    if (g_drep[mot] > 1) {
        mbx_ecrire(g_dsens[mot], mot, g_dval[mot], g_dval[mot], g_dctx[mot],
                   fn, insn, g_drep[mot] - 1);
    }
    g_drep[mot]  = 1;
    g_dval[mot]  = val;
    g_dctx[mot]  = ctx;
    g_dsens[mot] = sens;
    g_dvu[mot]   = 1;
    mbx_ecrire(sens, mot, val, avant, ctx, fn, insn, 0);
}

static void mbx_ecrire(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                       uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn,
                       unsigned long rep)
{
    static const char *nom_sens[] = { "ARM>WR", "ARM<RD", "DSP>WR", "DSP<RD" };
    char suffixe[32] = "";

    if (rep) {
        snprintf(suffixe, sizeof(suffixe), "  x%lu", rep + 1);
    }
    if ((g_n & 0x3FF) == 0) {
        fflush(g_f);                  /* at worst a 1024-line window */
    }
    if (sens == MBX_ARM_WR || sens == MBX_DSP_WR) {
        fprintf(g_f, "%-12u %-8u %-6s 0x%04x %-14s 0x%04x -> 0x%04x  @0x%04x%s\n",
                insn, fn, nom_sens[sens], mot, mbx_nom(mot), avant, val, ctx,
                suffixe);
    } else {
        fprintf(g_f, "%-12u %-8u %-6s 0x%04x %-14s = 0x%04x            @0x%04x%s\n",
                insn, fn, nom_sens[sens], mot, mbx_nom(mot), val, ctx, suffixe);
    }
}
