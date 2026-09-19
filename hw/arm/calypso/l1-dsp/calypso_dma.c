/*
 * calypso_dma.c - internal DMA controller of the TMS320C54x.
 *
 * See calypso_dma.h for the rationale, the mapping conflict and the explicit
 * list of what is NOT modelled.
 *
 * Configuration:
 *   CALYPSO_DMA=1            enable the module (default OFF)
 *   CALYPSO_DMA_VEC_BASE=30  DMA interrupt vector (default 30 = INT10n = IMR
 *                            bit 14 + 16, CAL000 5.1; shared by the 4 channels)
 *   CALYPSO_DMA_MAX_MOTS=... per-transfer safety cap (default 4096)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_dma.h"
#include "calypso_c54x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int calypso_dma_actif = -1;   /* -1 = initialise on first access */

static int      g_init;
/* Per CAL000 5.1. Do NOT take the DMAC0 vector from the GENERIC C54x table
 * (SPRU131): on Calypso, IMR bit 7 is INT4n = MCSI TRANSMIT, and no DMAC0..3
 * vectors exist. The DSP sees a SINGLE DMA line, INT10n = bit 14 = vec 30, and
 * the four channels (CAL000 3.5.15) share it; the channel is read from the
 * registers, not from the interrupt number. */
static int      g_vec_base = C54X_IT_DMA_VEC;   /* INT10n, shared by the 4 channels */
static unsigned g_max_mots = 4096;
static unsigned g_log;

/* Module-local state: the sub-register bank in calypso_c54x.c only holds 4
 * registers per channel, so the missing ones are kept here. */
static uint16_t g_dmprec;                       /* last DMPREC written */
static uint16_t g_subaddr;                      /* DMSA */
static uint16_t g_sub[C54X_DMA_CANAUX * 5];     /* SRC/DST/CTR/SFC/MCR */
static uint8_t  g_arme[C54X_DMA_CANAUX];        /* channel enabled, transfer due */

void calypso_dma_init(void)
{
    const char *e;

    if (g_init) {
        return;
    }
    g_init = 1;

    e = getenv("CALYPSO_DMA");
    if (!e || !*e || !strcmp(e, "0")) {
        calypso_dma_actif = 0;
        return;
    }
    e = getenv("CALYPSO_DMA_VEC_BASE");
    if (e && *e) {
        g_vec_base = (int)strtol(e, NULL, 0);
    }
    e = getenv("CALYPSO_DMA_MAX_MOTS");
    if (e && *e) {
        g_max_mots = (unsigned)strtoul(e, NULL, 0);
    }

    calypso_dma_actif = 1;
    fprintf(stderr, "[dma] contrôleur DMA c54x ACTIF — mapping SPRU131 "
            "(DMPREC=0x54 DMSA=0x55 DMSDI=0x56 DMSDN=0x57), DMAC0=vec%d, "
            "plafond %u mots/transfert.\n"
            "[dma] ⚠️ transfert EN BLOC au tick de trame : la synchronisation "
            "par événement (DMSFC) n'est PAS modélisée.\n",
            g_vec_base, g_max_mots);
}

/* Readable name of a sub-register, for traces. */
static const char *dma_nom_sub(unsigned reg)
{
    static const char *n[5] = { "SRC", "DST", "CTR", "SFC", "MCR" };
    return (reg < 5) ? n[reg] : "?";
}

bool calypso_dma_mmr_write(C54xState *s, uint16_t addr, uint16_t val)
{
    if (calypso_dma_actif < 0) {
        calypso_dma_init();
    }
    if (!calypso_dma_actif) {
        return false;            /* off: the historical decode applies */
    }

    switch (addr) {
    case C54X_DMPREC: {
        uint16_t avant = g_dmprec;
        g_dmprec = val;
        /* Bits 0..5: DE0..DE5, per-channel enable. A rising edge arms the
         * transfer; it is NOT started here but at the tick, so the firmware
         * has finished writing its sub-registers first. */
        for (unsigned c = 0; c < C54X_DMA_CANAUX; c++) {
            uint16_t bit = (uint16_t)(1u << c);
            if ((val & bit) && !(avant & bit)) {
                g_arme[c] = 1;
                if (g_log < 40) {
                    g_log++;
                    fprintf(stderr, "[dma] canal %u ARMÉ par DMPREC=0x%04x "
                            "(SRC=0x%04x DST=0x%04x CTR=%u) PC=0x%04x\n",
                            c, val, g_sub[c * 5 + 0], g_sub[c * 5 + 1],
                            g_sub[c * 5 + 2], s->pc);
                }
            }
        }
        s->data[addr] = val;
        return true;
    }

    case C54X_DMSA:
        g_subaddr = val;
        s->data[addr] = val;
        return true;

    case C54X_DMSDI:
    case C54X_DMSDN:
        if (g_subaddr < C54X_DMA_CANAUX * 5) {
            g_sub[g_subaddr] = val;
            if (g_log < 40) {
                g_log++;
                fprintf(stderr, "[dma] canal %u %s = 0x%04x (sous-adr 0x%02x) "
                        "PC=0x%04x\n", g_subaddr / 5,
                        dma_nom_sub(g_subaddr % 5), val, g_subaddr, s->pc);
            }
        }
        s->data[addr] = val;
        if (addr == C54X_DMSDI) {
            g_subaddr++;          /* auto-increment; DMSDN has none */
        }
        return true;

    default:
        return false;
    }
}

void calypso_dma_tick(C54xState *s)
{
    if (calypso_dma_actif < 0) {
        calypso_dma_init();
    }
    if (!calypso_dma_actif || !s) {
        return;
    }

    for (unsigned c = 0; c < C54X_DMA_CANAUX; c++) {
        if (!g_arme[c]) {
            continue;
        }
        uint16_t src = g_sub[c * 5 + 0];
        uint16_t dst = g_sub[c * 5 + 1];
        uint16_t ctr = g_sub[c * 5 + 2];
        unsigned n   = ctr;

        /* Safety net: a bogus CTR (sub-registers not written yet, or a wrong
         * mapping) must not sweep the whole DSP memory. */
        if (n == 0 || n > g_max_mots) {
            if (g_log < 40) {
                g_log++;
                fprintf(stderr, "[dma] canal %u : CTR=%u hors plage [1..%u] — "
                        "transfert IGNORÉ (sous-registres incomplets ?)\n",
                        c, n, g_max_mots);
            }
            g_arme[c] = 0;
            continue;
        }

        /* Block transfer, DATA space only (see the header: program and I/O
         * spaces are not modelled). */
        for (unsigned i = 0; i < n; i++) {
            s->data[(uint16_t)(dst + i)] = s->data[(uint16_t)(src + i)];
        }

        g_sub[c * 5 + 2] = 0;     /* CTR exhausted */
        g_arme[c] = 0;
        g_dmprec = (uint16_t)(g_dmprec & ~(1u << c));   /* channel disarms itself */
        s->data[C54X_DMPREC] = g_dmprec;

        if (g_log < 40) {
            g_log++;
            fprintf(stderr, "[dma] canal %u TRANSFÉRÉ %u mots 0x%04x -> 0x%04x, "
                    "interruption DMA INT10n (canal %u, vec %d, IMR bit %d)\n",
                    c, n, src, dst, c, g_vec_base, g_vec_base - 16);
        }

        /* Completion signal. CAL000 5.1: one INT10n line for all 4 channels,
         * IMR bit = vec - 16 (formula from calypso_c54x.h). */
        c54x_interrupt_ex(s, g_vec_base, g_vec_base - 16);
    }
}
