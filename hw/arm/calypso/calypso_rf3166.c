/*
 * calypso_rf3166.c - RF3166 power amplifier.
 *
 * Enable = PA_ENABLE = TSPACT(1), ACTIVE HIGH (osmocom-bb rffe_dualband.c).
 * Order imposed by rffe_mode(): at rest `tspact &= ~PA_ENABLE`; to transmit
 * `tspact &= ~TRENA` (antenna switch) then `tspact |= PA_ENABLE`, so the PA
 * only comes up AFTER the antenna has been switched over. That ordering is
 * what this model checks.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdio.h>
#include "hw/arm/calypso/calypso_rf3166.h"
#include "hw/arm/calypso/calypso_asm4532.h"
#include "hw/arm/calypso/calypso_debug.h"

#define PA_ENABLE   (1u << 1)   /* TSPACT(1), active HIGH */

#define PA_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TPU")) \
        fprintf(stderr, "[rf3166] " fmt "\n", ##__VA_ARGS__); } while (0)

/* Ramp model: 0..255 -> 0..33 dBm. ⚠️ MADE UP, not calibrated. */
#define RF3166_MAX_DBM  33

static struct {
    bool     on;
    bool     apc_known;
    uint8_t  apc;
    uint32_t bursts;        /* number of PA activations */
    uint32_t faults;        /* activations with the switch not in TX position */
} pa;

void calypso_rf3166_tspact_update(uint16_t tspact, uint32_t fn)
{
    bool on_new = !!(tspact & PA_ENABLE);

    if (on_new == pa.on) {
        return;
    }

    if (on_new) {
        pa.bursts++;
        /* Cross-check against the switch: the firmware switches BEFORE
         * powering up. If the antenna is not on the PA, we transmit into
         * nothing. */
        if (!calypso_asm4532_tx_connected()) {
            pa.faults++;
            fprintf(stderr, "[rf3166] ⚠ PA_ENABLE assertee alors que l'antenne "
                    "n'est PAS commutee sur le PA (TRENA relachee) — fn=%u, "
                    "faute #%u\n", fn, pa.faults);
        } else {
            PA_LOG("PA ON  (bande %s) fn=%u  [burst #%u]",
                   calypso_asm4532_band_gsm() ? "GSM900" : "DCS1800",
                   fn, pa.bursts);
        }
    } else {
        PA_LOG("PA OFF fn=%u", fn);
    }

    pa.on = on_new;
}

void calypso_rf3166_set_apc(uint8_t apc_level)
{
    pa.apc = apc_level;
    pa.apc_known = true;
}

bool calypso_rf3166_enabled(void) { return pa.on; }

int32_t calypso_rf3166_out_dbm(void)
{
    if (!pa.on || !pa.apc_known) {
        return INT32_MIN;   /* off, or APC never set: unknown */
    }
    return ((int32_t)pa.apc * RF3166_MAX_DBM) / 255;
}

uint32_t calypso_rf3166_faults(void) { return pa.faults; }
