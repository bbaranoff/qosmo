/*
 * calypso_rhea_dma.h - Calypso RHEA DMA controller, MCU side (FFFF:FC00).
 * Source: CAL207 11 (ti-calypso2.pdf).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_RHEA_DMA_H
#define CALYPSO_RHEA_DMA_H

#include "exec/hwaddr.h"
#include <stdint.h>
#include <stdbool.h>

#define CALYPSO_RHEA_DMA_BASE 0xFFFFFC00

uint64_t calypso_rhea_dma_read(void *opaque, hwaddr off, unsigned size);
void     calypso_rhea_dma_write(void *opaque, hwaddr off, uint64_t val, unsigned size);

/* Same register bank seen from the DSP's Rhea bus (XIO:FC00..FCFF, CAL207
 * 11.1). Returns true if PA falls inside the DMA window. */
bool     calypso_rhea_dma_xio(bool write, uint16_t pa, uint16_t *val, uint16_t pc);

/* Receive request: the RIF signals that a burst is available and that the
 * firmware picked DMA mode (RDMA_MASK=0). The controller then drains the
 * receiver into API memory at DMA2_AAD, sets IRQ_STATE and raises INT10n if
 * IRQ_MODE asks for it.
 *
 * Gate CALYPSO_RHEA_DMA_XFER (default 0): without it the module stays the
 * read-only instrument it was, logging without transferring anything. Enabling
 * it changes the module's NATURE (from instrument to piece of hardware), hence
 * opt-in. */
struct C54xState;
void     calypso_rhea_dma_rx_request(struct C54xState *s);

/* Level of the INT10n line. CAL000 5.1: "INT10n (level) -> DMA interrupt". The
 * line stays asserted as long as the channel has its IRQ_STATE set; READING the
 * register is what clears it (CAL207 11.3.5). Without that, the model treats the
 * interrupt as edge-triggered and loses it whenever it arrives with INTM=1 -
 * which is 15 times out of 15 in the measured runs. */
uint16_t calypso_rhea_dma_get_daram(void);
uint16_t calypso_rhea_dma_get_len_words(void);
bool     calypso_rhea_dma_irq_level(void);

#endif /* CALYPSO_RHEA_DMA_H */
