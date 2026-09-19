/*
 * calypso_mailbox.h - full monitor of the ARM <-> DSP mailbox.
 *
 * The mailbox (API RAM, DSP words 0x0800..0x0FFF) is the only contact point
 * between the ARM and the c54x: which task is commanded, who writes d_burst_d,
 * which cell the dispatcher polls, are all events on that boundary. Point
 * probes wired to one address or one value report zero when they miss, and
 * zero reads like an answer; this module traces the whole flow, both
 * directions, all the time.
 *
 * Output goes to its own file, not stderr: a full stream on stderr drowns
 * qemu.log and truncates the neighbouring probes (measured once at 480898
 * lines / 35 MB with the surrounding traces lost). The two journals stay
 * independent.
 *
 * The record format is uniform so two runs are diffable: capture a sequence
 * under `shunt_legit`, which works and therefore exercises the real command
 * sequence, then capture it natively; the delta is the list of what is not
 * wired yet.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_MAILBOX_H
#define CALYPSO_MAILBOX_H

#include <stdint.h>

/* Access direction, seen from the mailbox. */
typedef enum {
    MBX_ARM_WR = 0,   /* ARM writes  (command) */
    MBX_ARM_RD,       /* ARM reads   (result)  */
    MBX_DSP_WR,       /* DSP writes  (result)  */
    MBX_DSP_RD,       /* DSP reads   (command) */
} CalypsoMbxSens;

/* Tested inline on the hot paths (the c54x data_read / data_write run on every
 * instruction): with the monitor off the cost is one int load.
 *
 * Three states are required: -1 = not initialised, 0 = off, 1 = on. Init is
 * lazy, so a flag starting at 0 would keep the inline wrapper from ever calling
 * calypso_mbx_evt(), calypso_mbx_init() would never run, the flag would stay 0
 * and no file would be created. With -1 the first access goes through, init
 * decides, and later accesses are filtered. */
extern int calypso_mbx_actif;

/* Opens the file and reads the configuration. Idempotent, called lazily on the
 * first event, so there is nothing to schedule at startup. */
void calypso_mbx_init(void);

/* Records one event. `avant` is meaningful for writes only (0 otherwise).
 * `ctx` is the DSP PC, or the MMIO offset on the ARM side. */
void calypso_mbx_evt(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                     uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn);

/* Inline wrapper: the flag test avoids the call when the monitor is off. */
static inline void calypso_mbx(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                               uint16_t avant, uint32_t ctx, uint32_t fn,
                               uint32_t insn)
{
    if (calypso_mbx_actif) {
        calypso_mbx_evt(sens, mot, val, avant, ctx, fn, insn);
    }
}

#endif /* CALYPSO_MAILBOX_H */
