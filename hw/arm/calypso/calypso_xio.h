/*
 * calypso_xio.h - DSP XIO windows not modelled elsewhere (API Control F900,
 * DSP INTH FA00). Source: CAL207 7.2.2 and 11.3.5 (note), CAL000 3.7.6.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_XIO_H
#define CALYPSO_XIO_H

#include <stdint.h>
#include <stdbool.h>

/* Returns true when PA falls in one of the windows handled here. */
bool calypso_xio_misc(bool write, uint16_t pa, uint16_t *val, uint16_t pc);

/* True when the DSP has put the API RAM in HOM (Host Only Mode, 9.1 bit 1):
 * the API window is then reserved for the ARM and the DMA, CAL000 7.2.1. */
bool calypso_xio_api_hom(void);

#endif /* CALYPSO_XIO_H */
