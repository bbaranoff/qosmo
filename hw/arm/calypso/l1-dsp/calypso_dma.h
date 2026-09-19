/*
 * calypso_dma.h - TMS320C54x on-chip DMA controller.
 *
 * Why this module exists. The DSP firmware queues transfer requests in a
 * 14-entry ring (read pointer data[0x433e], write pointer data[0x433f],
 * producer at 0xaa75, consumer at 0xaa87). [2026-07-29] The producer pushes,
 * the read pointer NEVER advances, the ring fills, the write pointer catches
 * the read pointer and the firmware raises DSP_ERR_DMA_PROG (bit 3 of
 * data[0x3f92], published in d_error_status = 0x08d5, printed by the ARM as
 * "DSP Error Status: 8", 605 times).
 *
 * The instructions in that sequence are all emulated correctly; what is
 * missing is the DMA controller itself. The core model accepts the
 * programming (sub-register bank in calypso_c54x.c) but performs no transfer
 * and never raises DMAC0..5, so nothing signals completion, the consumer
 * never has a reason to pop, and the queue saturates.
 *
 * ⚠️ MAPPING CONFLICT, to be settled by measurement.
 *   SPRU131 places DMPREC=0x54 DMSA=0x55 DMSDI=0x56 DMSDN=0x57.
 *   calypso_c54x.c uses 0x54 as DMSA, one register off; the DMAWATCH probe
 *   already flags this with its "DMPREC?(modele:DMSA)" label. This module
 *   implements the MANUAL mapping and stays OPTIONAL (CALYPSO_DMA=1, default
 *   OFF): while it is off, the existing decode runs unchanged.
 *
 * ⚠️ NOT MODELLED, and to stay that way until measured:
 *   - event synchronisation (DMSFC: McBSP, timer, external line) - transfers
 *     are done IN ONE BLOCK on the frame tick, not at the real rate;
 *   - transfers to/from PROGRAM space and I/O space (DMMCR);
 *   - priority interleaving between channels (DPRC);
 *   - ABU mode (hardware circular addressing per channel).
 *   Each of these is a deliberate approximation, not an oversight. The only
 *   goal here is to close the "program -> transfer -> signal" loop.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_DMA_H
#define CALYPSO_DMA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct C54xState C54xState;

/* MMR addresses per SPRU131. */
#define C54X_DMPREC   0x0054   /* control/priority, bits 0..5 = channel enable */
#define C54X_DMSA     0x0055   /* sub-address */
#define C54X_DMSDI    0x0056   /* data, auto-increments DMSA */
#define C54X_DMSDN    0x0057   /* data, no auto-increment */

/* Five sub-registers per channel (SPRU131, DMA chapter). The existing bank in
 * calypso_c54x.c only has four (SRC/DST/CTR/MCR): DMSFC is missing, which is
 * why event synchronisation cannot be modelled. */
#define C54X_DMA_CANAUX   6

/* Whether the module is enabled; tested inline on hot paths. */
extern int calypso_dma_actif;

/* Read the configuration (CALYPSO_DMA). Idempotent, called lazily. */
void calypso_dma_init(void);

/* Intercept an MMR write. Returns true when this module handled it, in which
 * case the caller does nothing more. Returns false when the module is off,
 * leaving the historical decode in place. */
bool calypso_dma_mmr_write(C54xState *s, uint16_t addr, uint16_t val);

/* Call once per TDMA frame: run the active channels and raise the
 * end-of-transfer interrupt. */
void calypso_dma_tick(C54xState *s);

#endif /* CALYPSO_DMA_H */
