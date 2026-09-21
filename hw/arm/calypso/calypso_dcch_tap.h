/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_DCCH_TAP_H
#define HW_ARM_CALYPSO_DCCH_TAP_H

#include <stdint.h>

/* Octet emis par le firmware sur l'UART modem (sercomm). */
void calypso_dcch_tap_tx_byte(uint8_t ch);
/* Le firmware a remis sa couche 1 a zero : le canal dedie est libere. */
void calypso_dcch_tap_reset(void);

#endif
