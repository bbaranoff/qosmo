/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CALYPSO_GMSK_H
#define CALYPSO_GMSK_H
#include <stdint.h>
/* Modulates n bits as GMSK BT=0.3 (3GPP 45.004), one complex sample per symbol,
 * interleaved int16 I,Q. phase0 = initial phase (rad); decalage = sampling
 * instant inside the symbol, as a fraction in [0,1[ (0.5 = centre). */
void gmsk_moduler(const uint8_t *bits, int n, int amp, double phase0, double decalage, int16_t *iq);
#endif
