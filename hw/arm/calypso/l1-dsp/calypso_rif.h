/*
 * calypso_rif.h - Calypso Radio InterFace (RIF), DSP view (XIO space).
 *
 * Sources: CAL207 "Register Mapping" §12 (ti-calypso2.pdf), CAL000 §3.7.1 and
 * §5.1 (ti-calypso1.pdf).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_RIF_H
#define CALYPSO_RIF_H

#include <stdint.h>
#include <stdbool.h>
#include "calypso_c54x.h"

/* XIO addresses, §12.1 (Table 19). */
#define RIF_XIO_DXR    0x0000   /* Transmit Data Register  - 16b R/W          */
#define RIF_XIO_DRR    0x0001   /* Receive Data Register   - 16b R            */
#define RIF_XIO_SPCX   0x0002   /* Control Register (TX)   - 15b, reset 0x059E */
#define RIF_XIO_SPCR   0x0003   /* Control Register (RX)   - 15b, reset 0x3CA2 */

/* True when the module is enabled (CALYPSO_RIF_XIO, default 1). */
bool calypso_rif_on(void);

/* PORTR PA,Smem / PORTW Smem,PA: return true when PA belongs to the RIF.
 * portr stores the value read in *out. */
bool calypso_rif_portr(C54xState *s, uint16_t pa, uint16_t *out);
bool calypso_rif_portw(C54xState *s, uint16_t pa, uint16_t val);

/* A burst arrives from the radio front end: the words are staged and flow into
 * the receive FIFO as the DSP reads DRR. */
void calypso_rif_rx_burst(C54xState *s, const uint16_t *w, int n);

/* DMA drain path. In DMA mode (RDMA_MASK=0) the DSP does not read DRR word by
 * word: the RHEA controller empties the receiver into API memory (CAL000
 * §3.7.1, "an end-DMA request is sent"). This is the only way
 * calypso_rhea_dma.c takes samples out; the FIFO and the staging area stay
 * private to the RIF.
 *
 * Returns the number of words actually copied (0 when the receiver is empty)
 * and refills the FIFO from staging, exactly as a DRR read does. No side effect
 * on SPCR: notification is the caller's decision, not the RIF's. */
int calypso_rif_drain(uint16_t *dst, int max);

#endif /* CALYPSO_RIF_H */
