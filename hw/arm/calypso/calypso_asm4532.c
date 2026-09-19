/*
 * calypso_asm4532.c - ASM4532 antenna switch.
 *
 * Driven by the TSPACT lines, not by the serial TSP. Line mapping taken
 * as-is from osmocom-bb board/compal/rffe_dualband.c (C123 board):
 *   TRENA    = TSPACT(6)  Transmit Enable (antenna switch)  -- ACTIVE LOW
 *   GSM_TXEN = TSPACT(8)  GSM as opposed to DCS             -- ACTIVE LOW
 *
 * This model is PASSIVE: it latches the state, counts TX windows and reports
 * inconsistencies. It gates nothing, because the downlink is synthetic and
 * gating Rx on the switch position would only break working benches. It
 * becomes useful once the uplink is really transmitted: it is what will tell
 * whether the antenna was switched over during the burst.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdio.h>
#include "hw/arm/calypso/calypso_asm4532.h"
#include "hw/arm/calypso/calypso_debug.h"

/* TSPACT lines, ported from the firmware (TSPACT(n) = 1 << n). */
#define ASM_TRENA       (1u << 6)   /* active LOW */
#define ASM_GSM_TXEN    (1u << 8)   /* active LOW */

#define ASM_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TPU")) \
        fprintf(stderr, "[asm4532] " fmt "\n", ##__VA_ARGS__); } while (0)

static struct {
    bool     init;
    uint16_t tspact;        /* last state seen */
    bool     tx;            /* TRENA asserted    -> antenna on the PA */
    bool     gsm;           /* GSM_TXEN asserted -> GSM900 band */
    uint32_t tx_windows;    /* number of TX windows opened */
} asm4532;

void calypso_asm4532_tspact_update(uint16_t tspact, uint32_t fn)
{
    bool tx_new, gsm_new;

    /* ACTIVE LOW: the line is asserted when the bit reads 0. */
    tx_new  = !(tspact & ASM_TRENA);
    gsm_new = !(tspact & ASM_GSM_TXEN);

    if (!asm4532.init) {
        asm4532.init = true;
        ASM_LOG("etat initial : tspact=0x%04x TX=%d bande=%s fn=%u",
                tspact, tx_new, gsm_new ? "GSM900" : "DCS1800", fn);
    }

    if (tx_new != asm4532.tx) {
        if (tx_new) {
            asm4532.tx_windows++;
        }
        ASM_LOG("TRENA %s -> antenne sur %s (bande %s) fn=%u  [fenetre #%u]",
                tx_new ? "ASSERTEE" : "relachee",
                tx_new ? "le PA (TX)" : "le chemin Rx",
                gsm_new ? "GSM900" : "DCS1800", fn, asm4532.tx_windows);
    } else if (gsm_new != asm4532.gsm && tx_new) {
        /* Band change DURING a TX window: the firmware does not do this,
         * rffe_mode() sets both lines together. Report it rather than
         * absorbing it silently. */
        ASM_LOG("⚠ bande changee PENDANT la fenetre TX : %s -> %s fn=%u",
                asm4532.gsm ? "GSM900" : "DCS1800",
                gsm_new ? "GSM900" : "DCS1800", fn);
    }

    asm4532.tspact = tspact;
    asm4532.tx     = tx_new;
    asm4532.gsm    = gsm_new;
}

bool     calypso_asm4532_tx_connected(void) { return asm4532.tx; }
bool     calypso_asm4532_band_gsm(void)     { return asm4532.gsm; }
uint32_t calypso_asm4532_tx_windows(void)   { return asm4532.tx_windows; }
