/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_DCCH_TAP_H
#define HW_ARM_CALYPSO_DCCH_TAP_H

#include <stdint.h>

/* Byte emitted by the firmware on the modem UART (sercomm). */
void calypso_dcch_tap_tx_byte(uint8_t ch);
/* The firmware reset its layer 1: the dedicated channel is released. */
void calypso_dcch_tap_reset(void);

#endif
