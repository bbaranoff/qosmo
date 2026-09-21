/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CALYPSO_GMSK_H
#define CALYPSO_GMSK_H
#include <stdint.h>
/* Modulates n bits as GMSK BT=0.3 (3GPP 45.004), one complex sample per symbol,
 * interleaved int16 I,Q. phase0 = initial phase (rad); decalage = sampling
 * instant inside the symbol, as a fraction in [0,1[ (0.5 = centre). */
void gmsk_moduler(const uint8_t *bits, int n, int amp, double phase0, double decalage, int16_t *iq);
/* [2026-09-21] Symmetric widening of a 1-sample/symbol burst, in place:
 * y[n] = (x[n] + a (x[n-1] + x[n+1])) / (1 + 2a), n = number of complex samples.
 * Why: the ROM's NB equaliser zeroes every channel tap whose energy is below
 * 1/16 of the window energy (PROM0 0x7f0c-0x7f1c, i.e. 25 % in amplitude);
 * the +-1 taps of a plain GMSK at 1 sps sit exactly on that threshold and the
 * decision flips with the data. a = 0.3 puts them near 55 %. A receiver's
 * channel filter does the same on silicon. Measured: SI1-4 decode with it,
 * ~40 % of the bursts without. */
void gmsk_elargir(int16_t *iq, int n, double a);
#endif
