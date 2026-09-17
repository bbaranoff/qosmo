/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_L1CTL_TAP_H
#define HW_ARM_CALYPSO_L1CTL_TAP_H

#include <stdint.h>

void calypso_l1ctl_tap_tx_byte(uint8_t byte);
void calypso_l1ctl_tap_channel_released(void);

#endif
